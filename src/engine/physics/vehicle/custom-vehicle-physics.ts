import RAPIER from '@dimforge/rapier3d-compat';
import { Matrix, Quaternion, Vector3, Scene } from '@babylonjs/core';
import { VehicleSpec } from '../../types/vehicle-spec';
import { ChassisRig } from './rigs/chassis-rig';
import { Bounds, WheelKey, WHEEL_KEYS, WheelOffsets, WheelSizes } from './rigs/wheel.types';
import { RawSuspensionSettings } from '../../types/vehicle-spec';
import { GRAVITY, TRACK_COLLIDER_FRICTION } from '../../constants';
import { clamp } from '../../utils/math';
import { resolveVehicleHandling, type VehicleHandlingResolved } from './vehicle-handling-control';
import { ABS_ASSIST, CASTER_ASSIST } from '../../configs/driver-assist';
import {
  combinedSlipForces,
  effectiveMu,
  longitudinalSlipStiffness,
  resolveTireModel,
  type TireModelParams,
} from './wheels/tire-model';
import {
  applyAeroAndRollingForces,
  applyMassProperties,
  applyYawRateDamping,
  bodyPosition,
  bodyRotation,
  bodySlipAngleRad,
  clampBodyAngularVelocity,
  ComDebugGizmo,
  computeLowestWheelTreadPlaneY,
  drivenWheelKeys,
  ForcesTelemetry,
  lateralAccelMs2,
  LANDING_BOOST_DURATION_S,
  planarSpeedMs,
  resolveWheelOffsets,
  resolveWheelSizes,
  steeredWheelKeys,
  VY_FAST_FALL,
  VY_VERY_FAST_FALL,
  type VehicleDriveControls,
  type VehicleForces,
  type VehicleTractionSnapshot,
  type WheelTractionDebug,
} from './vehicle-physics-shared';

/**
 * Custom wheel/tire model replacing RAPIER.DynamicRayCastVehicleController.
 *
 * What it does honestly (unlike the raycast controller):
 *  - suspension in newtons: N = preload + k·x + c·ẋ, progressive bump stop;
 *  - per-wheel/axle ω is integrated state => longitudinal slip ratio exists;
 *  - lateral force from slip angle via a combined-slip friction circle
 *    (one curve, one μ_eff·N budget for both axes) + relaxation length;
 *  - spool (locked diff): the driven axle shares ONE ω for both wheels;
 *  - brakes are wheel torque with a fixed bias; lockup emerges on its own;
 *  - sublinear load sensitivity μ(N).
 *
 * Rapier keeps the body and collisions: forces go through addForceAtPoint at
 * contact points, so weight transfer and roll/pitch moments follow from geometry.
 */

type ChassisMesh = { positions: number[]; indices: number[] };
type PhysicsConfig = VehicleSpec['physics'];

type ResolvedSuspension = {
  restLength: number;
  /** Spring rate as configured (clamped), N/m - before the motion ratio. */
  springRateNM: number;
  motionRatio: number;
  wheelRateNM: number;
  /** null = auto: per-corner critical-damping defaults in refreshWheelDampers. */
  compressionDampingNsM: number | null;
  reboundDampingNsM: number | null;
  bumpTravel: number;
  reboundTravel: number;
  bumpStopRateNM: number;
  bumpStopRangeM: number;
  /** Anti-roll bars, N/m of wheel travel difference. Roll split = car balance. */
  arbFrontNM: number;
  arbRearNM: number;
};

/** Effective suspension values for the debug UI - what the sim actually runs. */
export type SuspensionResolvedView = {
  restLength: number;
  springRate: number;
  compressionDamping: number;
  reboundDamping: number;
  bumpTravel: number;
  reboundTravel: number;
  motionRatio: number;
  arbFrontNM: number;
  arbRearNM: number;
  bumpStopRateMult: number;
  bumpStopRangeM: number;
};

type CWheel = {
  key: WheelKey;
  centerOffset: { x: number; y: number; z: number };
  hardPointOffset: { x: number; y: number; z: number };
  radius: number;
  isSteering: boolean;
  isDriven: boolean;
  isFront: boolean;
  /**
   * Static load of THIS corner, N - from the CoM projected onto wheel geometry
   * (bilinear axle/track), NOT m·g/4. Suspension preload and the μ(N) reference
   * are per-corner, so config weight distribution actually works.
   */
  staticN: number;
  /** Per-corner dampers: config value, or critical-damping default from staticN. */
  compressionDampingNsM: number;
  reboundDampingNsM: number;

  // suspension
  suspensionLength: number;
  prevCompression: number;
  inContact: boolean;
  normalN: number;
  contactPoint: { x: number; y: number; z: number };
  contactNormal: { x: number; y: number; z: number };

  // tire
  fyRelaxedN: number;
  spinAngleRad: number;

  // last-substep telemetry
  lastSlipRatio: number;
  lastFxN: number;
  lastFyN: number;
  lastRequestedEngineN: number;
  lastAppliedEngineN: number;
};

/** ω-integration group: driven axle is a spool (one ω), free wheels are individual. */
type WheelGroup = {
  keys: WheelKey[];
  isDriven: boolean;
  omegaRadS: number;
  inertia: number;
};

// ── solver constants ─────────────────────────────────────────────

/** Inner ω-integration steps per physics substep (tire is stiff in slip). */
const TIRE_INNER_STEPS = 4;
/**
 * Tread speed cap ω·r against numeric runaway. Uses the wheel's own radius -
 * a rad/s constant would cap karts/small wheels at a lower speed.
 */
const MAX_WHEEL_TREAD_SPEED_MS = 160;
/** Progressive bump stop: stack stiffness relative to the spring. */
const BUMP_STOP_RATE_MULT = 10;
/**
 * Depth over which the bump stop keeps adding force past bumpTravel, m. Wide on
 * purpose: a hard compression clamp zeroed damper velocity and caused restitution.
 */
const BUMP_STOP_RANGE_M = 0.15;
/**
 * Bump stop is plastic: pushes only while compressing (ẋ > threshold), silent on
 * rebound. Explicit integration of the stiff stack otherwise returns more energy
 * than it stored - the car bounces like a ball after landing.
 */
const BUMP_STOP_RELEASE_VEL_MS = -0.05;
/** Sanity cap on suspension force per wheel, N. */
const ABS_MAX_SUSPENSION_FORCE_PER_WHEEL = 150_000;
/** Compression velocity cap for the damper, m/s (first contact frame after a jump). */
const MAX_DAMPER_VEL_MS = 8;
/** Slip ratio denominator floor (below this the low-speed model takes over). */
const SLIP_DENOM_MIN_MS = 1.0;
/**
 * Lateral force blend kinematic => slip model over |vLong|, m/s. The wide zone is
 * needed: relaxation lag + steep Fy(α) at low speed make an underdamped ~20 Hz
 * oscillator near the 60 Hz body-step Nyquist. Below 8 m/s the lag-free
 * kinematic model holds the lateral axis.
 */
const FY_BLEND_START_MS = 0.4;
const FY_BLEND_END_MS = 8.0;
/** Default front brake bias (NASCAR ~0.55-0.60). */
const DEFAULT_BRAKE_BIAS_FRONT = 0.58;

// Auto counter-steer (caster) and ABS-lite: δ_eff = δ_driver + k·(β - deadzone);
// past peak slip the brake torque backs off. Knobs live in configs/driver-assist.ts
// (CASTER_ASSIST / ABS_ASSIST) and are read directly for live debug tuning.

export class CustomVehiclePhysics {
  private rigidBody?: RAPIER.RigidBody;
  private chassisRig: ChassisRig;
  private wheels: Record<WheelKey, CWheel> | null = null;
  private groups: WheelGroup[] = [];
  private chassisSize = { x: 1, y: 1, z: 1 };
  private suspension: ResolvedSuspension;
  private readonly tireParams: TireModelParams;
  private readonly handling: VehicleHandlingResolved;
  private readonly brakeBiasFront: number;

  private driveControls: VehicleDriveControls = {
    engineForce: 0,
    brakeTotalForceN: 0,
    steering: 0,
    engineRunning: false,
  };

  private landingBoostRemaining = 0;
  private readonly comDebug: ComDebugGizmo;
  private readonly forcesTelemetry = new ForcesTelemetry();
  /** Smoothed auto counter-steer (caster), rad. Added to front-axle driver steering. */
  private casterCorrectionRad = 0;

  private lastWheelTraction: VehicleTractionSnapshot | null = null;

  constructor(
    private world: RAPIER.World,
    private scene: Scene,
    private physics: PhysicsConfig
  ) {
    this.chassisRig = new ChassisRig(this.world);
    this.handling = resolveVehicleHandling(this.physics.handling);
    this.comDebug = new ComDebugGizmo(this.scene);
    this.tireParams = resolveTireModel(this.physics.tires.model);
    this.suspension = this.resolveSuspension(this.physics.suspension);
    const bias = this.physics.brakes?.bias;
    this.brakeBiasFront =
      typeof bias === 'number' && Number.isFinite(bias)
        ? Math.min(0.8, Math.max(0.2, bias))
        : DEFAULT_BRAKE_BIAS_FRONT;
  }

  // ── creation ───────────────────────────────────────────────────

  public create(
    initialPosition: { x: number; y: number; z: number } = { x: 0, y: 2, z: 0 },
    bounds?: Bounds,
    wheelOffsets?: WheelOffsets,
    wheelSizes?: WheelSizes,
    chassisMesh?: ChassisMesh,
    spawnYawRad = 0
  ): void {
    const mass = this.physics.mass;
    const size = bounds?.size ?? { x: 1, y: 1, z: 1 };
    const center = bounds?.center ?? { x: 0, y: 0, z: 0 };
    this.chassisSize = { x: Math.abs(size.x), y: Math.abs(size.y), z: Math.abs(size.z) };

    const rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(initialPosition.x, initialPosition.y, initialPosition.z)
      .setRotation({ x: 0, y: Math.sin(spawnYawRad / 2), z: 0, w: Math.cos(spawnYawRad / 2) })
      .setLinvel(0, 0, 0)
      .setAngvel({ x: 0, y: 0, z: 0 })
      .setLinearDamping(0)
      // yaw is damped by tires only
      .setAngularDamping(0)
      .setCcdEnabled(true)
      .setCanSleep(false);

    this.rigidBody = this.world.createRigidBody(rigidBodyDesc);

    const rigBounds = { size, center };
    this.chassisRig.dispose();

    const resolvedWheelOffsets = resolveWheelOffsets(this.physics, rigBounds, wheelOffsets);
    const resolvedWheelSizes = resolveWheelSizes(this.physics, wheelSizes);
    const hullVertexMinLocalY = computeLowestWheelTreadPlaneY(resolvedWheelOffsets, resolvedWheelSizes);

    this.chassisRig.create({
      rigidBody: this.rigidBody,
      bounds: rigBounds,
      chassisMesh,
      hullVertexMinLocalY
    });

    applyMassProperties(this.rigidBody, mass, this.chassisSize, this.physics.centerOfMass);
    this.createWheels(resolvedWheelOffsets, resolvedWheelSizes);
    this.landingBoostRemaining = LANDING_BOOST_DURATION_S;
  }

  private createWheels(offsets: WheelOffsets, sizes: WheelSizes): void {
    const drivenSet = new Set(drivenWheelKeys(this.physics.transmission.layout ?? 'rwd'));
    const steeredSet = new Set(steeredWheelKeys(this.physics.steeredAxle ?? 'front'));
    const susp = this.suspension;

    const wheels = {} as Record<WheelKey, CWheel>;
    for (const key of WHEEL_KEYS) {
      const centerOffset = offsets[key];
      const radius = Math.max(0.05, Math.abs(sizes[key].y) / 2);
      wheels[key] = {
        key,
        centerOffset,
        hardPointOffset: {
          x: centerOffset.x,
          y: centerOffset.y + susp.restLength,
          z: centerOffset.z
        },
        radius,
        isSteering: steeredSet.has(key),
        isDriven: drivenSet.has(key),
        isFront: key === 'FL' || key === 'FR',
        suspensionLength: susp.restLength,
        prevCompression: 0,
        inContact: false,
        normalN: 0,
        contactPoint: { x: 0, y: 0, z: 0 },
        contactNormal: { x: 0, y: 1, z: 0 },
        staticN: (this.physics.mass * GRAVITY) / 4,
        compressionDampingNsM: 0,
        reboundDampingNsM: 0,
        fyRelaxedN: 0,
        spinAngleRad: 0,
        lastSlipRatio: 0,
        lastFxN: 0,
        lastFyN: 0,
        lastRequestedEngineN: 0,
        lastAppliedEngineN: 0,
      };
    }
    this.wheels = wheels;
    this.recomputeStaticCornerLoads();

    // ω groups: driven axle is a spool (one ω for both wheels), the rest individual
    const wheelInertia = (k: WheelKey) =>
      0.5 * Math.max(1, this.physics.tires.wheelMass) * wheels[k].radius * wheels[k].radius;
    const drivenKeys = WHEEL_KEYS.filter((k) => wheels[k].isDriven);
    const freeKeys = WHEEL_KEYS.filter((k) => !wheels[k].isDriven);
    this.groups = [];
    if (drivenKeys.length > 0) {
      this.groups.push({
        keys: drivenKeys,
        isDriven: true,
        omegaRadS: 0,
        inertia: drivenKeys.reduce((s, k) => s + wheelInertia(k), 0),
      });
    }
    for (const k of freeKeys) {
      this.groups.push({ keys: [k], isDriven: false, omegaRadS: 0, inertia: wheelInertia(k) });
    }
  }

  /**
   * Static corner loads from CoM position relative to the wheels: front/rear
   * split by CoM z between axles, left/right by CoM x within each track.
   * Cross-weight (torsion) is not modeled - the split is bilinear.
   * Clamps guard against degenerate GLB geometry.
   */
  private recomputeStaticCornerLoads(): void {
    if (!this.wheels) return;
    const w = this.wheels;
    const totalN = Math.max(1, this.physics.mass) * GRAVITY;
    const com = this.physics.centerOfMass;

    const frontZ = (w.FL.centerOffset.z + w.FR.centerOffset.z) / 2;
    const rearZ = (w.RL.centerOffset.z + w.RR.centerOffset.z) / 2;
    const zSpan = frontZ - rearZ;
    const frontFrac =
      Math.abs(zSpan) > 0.2 ? clamp((com.z - rearZ) / zSpan, 0.16, 0.84) : 0.5;

    const sideFrac = (lKey: WheelKey, rKey: WheelKey): number => {
      const xL = w[lKey].centerOffset.x;
      const xR = w[rKey].centerOffset.x;
      const span = xR - xL;
      return Math.abs(span) > 0.2 ? clamp((xR - com.x) / span, 0.25, 0.75) : 0.5;
    };
    const leftF = sideFrac('FL', 'FR');
    const leftR = sideFrac('RL', 'RR');

    w.FL.staticN = totalN * frontFrac * leftF;
    w.FR.staticN = totalN * frontFrac * (1 - leftF);
    w.RL.staticN = totalN * (1 - frontFrac) * leftR;
    w.RR.staticN = totalN * (1 - frontFrac) * (1 - leftR);

    // corner loads changed => auto damper defaults follow
    this.refreshWheelDampers();
  }

  /** Axle z in body-local coords - the GLB origin is not always at mid-wheelbase. */
  private axleZOffsets(): { frontZ: number; rearZ: number } | undefined {
    if (!this.wheels) return undefined;
    const w = this.wheels;
    return {
      frontZ: (w.FL.centerOffset.z + w.FR.centerOffset.z) / 2,
      rearZ: (w.RL.centerOffset.z + w.RR.centerOffset.z) / 2,
    };
  }

  /** Static weight fraction on the driven axle 0…1 - for the feedforward assist. */
  public getDrivenAxleStaticWeightFrac(): number {
    if (!this.wheels) return 0.5;
    let driven = 0;
    let total = 0;
    for (const key of WHEEL_KEYS) {
      total += this.wheels[key].staticN;
      if (this.wheels[key].isDriven) driven += this.wheels[key].staticN;
    }
    return total > 1 ? clamp(driven / total, 0.1, 1) : 0.5;
  }

  // ── suspension: real units, no Rapier normalization ────────────

  private resolveSuspension(raw: RawSuspensionSettings): ResolvedSuspension {
    // clamp() passes NaN through (Math.min/max) - drop non-finite input to defaults
    const fin = (v: number | undefined | null): number | undefined =>
      typeof v === 'number' && Number.isFinite(v) ? v : undefined;
    const motionRatio = clamp(fin(raw.motionRatio) ?? 1.0, 0.4, 1.4);
    const springRate = clamp(fin(raw.springRate) ?? 120_000, 10_000, 500_000);
    const wheelRate = clamp(springRate * motionRatio * motionRatio, 10_000, 500_000);
    const compression = fin(raw.compressionDamping);
    const rebound = fin(raw.reboundDamping);
    return {
      restLength: clamp(fin(raw.restLength) ?? 0.12, 0.05, 0.35),
      springRateNM: springRate,
      motionRatio,
      wheelRateNM: wheelRate,
      // null = auto: per-corner critical damping from the real corner loads
      compressionDampingNsM: compression !== undefined ? clamp(compression, 100, 80_000) : null,
      reboundDampingNsM: rebound !== undefined ? clamp(rebound, 100, 100_000) : null,
      bumpTravel: clamp(fin(raw.bumpTravel) ?? 0.055, 0.01, 0.2),
      reboundTravel: clamp(fin(raw.reboundTravel) ?? 0.075, 0.01, 0.25),
      bumpStopRateNM: wheelRate * clamp(fin(raw.bumpStopRateMult) ?? BUMP_STOP_RATE_MULT, 2, 40),
      bumpStopRangeM: clamp(fin(raw.bumpStopRangeM) ?? BUMP_STOP_RANGE_M, 0.03, 0.3),
      // NASCAR: stiff front bar, soft rear - understeer at the limit
      arbFrontNM: clamp(fin(raw.arbFrontNM) ?? 35_000, 0, 200_000),
      arbRearNM: clamp(fin(raw.arbRearNM) ?? 9_000, 0, 200_000),
    };
  }

  /**
   * Per-corner dampers: explicit config value for all corners, or the
   * critical-damping default from THIS corner's static load - a 60/40 car
   * needs different front/rear dampers, m/4 would give one number for all.
   */
  private refreshWheelDampers(): void {
    if (!this.wheels) return;
    const s = this.suspension;
    for (const key of WHEEL_KEYS) {
      const w = this.wheels[key];
      const cornerMass = Math.max(1, w.staticN / GRAVITY);
      const critical = 2 * Math.sqrt(s.wheelRateNM * cornerMass);
      w.compressionDampingNsM = s.compressionDampingNsM ?? clamp(critical * 0.4, 100, 80_000);
      w.reboundDampingNsM = s.reboundDampingNsM ?? clamp(critical * 0.9, 100, 100_000);
    }
  }

  public updateSuspension(settings: RawSuspensionSettings): void {
    const prevRest = this.suspension.restLength;
    this.suspension = this.resolveSuspension(settings);
    if (this.wheels) {
      const dRest = this.suspension.restLength - prevRest;
      for (const key of WHEEL_KEYS) {
        const w = this.wheels[key];
        w.hardPointOffset.y = w.centerOffset.y + this.suspension.restLength;
        // rest length moved but the strut did not: shift the stored compression
        // so the damper does not see a phantom velocity spike next tick
        w.prevCompression += dRest;
      }
      this.refreshWheelDampers();
    }
  }

  /** Effective suspension for the debug UI (dampers averaged when per-corner auto). */
  public getSuspensionResolved(): SuspensionResolvedView {
    const s = this.suspension;
    const avg = (sel: (w: CWheel) => number): number =>
      this.wheels ? WHEEL_KEYS.reduce((sum, k) => sum + sel(this.wheels![k]), 0) / WHEEL_KEYS.length : 0;
    return {
      restLength: s.restLength,
      springRate: s.springRateNM,
      compressionDamping: s.compressionDampingNsM ?? avg((w) => w.compressionDampingNsM),
      reboundDamping: s.reboundDampingNsM ?? avg((w) => w.reboundDampingNsM),
      bumpTravel: s.bumpTravel,
      reboundTravel: s.reboundTravel,
      motionRatio: s.motionRatio,
      arbFrontNM: s.arbFrontNM,
      arbRearNM: s.arbRearNM,
      bumpStopRateMult: s.bumpStopRateNM / Math.max(1, s.wheelRateNM),
      bumpStopRangeM: s.bumpStopRangeM,
    };
  }

  public updateMass(mass: number): void {
    this.physics.mass = mass;
    applyMassProperties(this.rigidBody, mass, this.chassisSize, this.physics.centerOfMass);
    // critical damping depends on mass - recompute defaults
    this.suspension = this.resolveSuspension(this.physics.suspension);
    this.recomputeStaticCornerLoads();
  }

  public updateCenterOfMass(com: { x: number; y: number; z: number }): void {
    this.physics.centerOfMass = com;
    applyMassProperties(this.rigidBody, this.physics.mass, this.chassisSize, com);
    // a CoM shift changes corner loads: preload and μ(N) reference follow
    this.recomputeStaticCornerLoads();
  }

  // ── public API ─────────────────────────────────────────────────

  public getPosition(): { x: number; y: number; z: number } {
    return bodyPosition(this.rigidBody);
  }

  public getRotation(): { x: number; y: number; z: number; w: number } {
    return bodyRotation(this.rigidBody);
  }

  public getSpeedMs(): number {
    return planarSpeedMs(this.rigidBody);
  }

  public getBodySlipAngleRad(): number {
    return bodySlipAngleRad(this.rigidBody);
  }

  public getLateralAccelMs2(): number {
    return lateralAccelMs2(this.rigidBody);
  }

  public getDrivenWheelRadiusM(): number {
    if (!this.wheels) return Math.max(0.05, this.physics.wheelRadius);
    const driven = WHEEL_KEYS.map((k) => this.wheels![k]).filter((w) => w.isDriven);
    if (!driven.length) return Math.max(0.05, this.physics.wheelRadius);
    return Math.max(0.05, driven.reduce((s, w) => s + w.radius, 0) / driven.length);
  }

  /** Driven axle RPM - real spool ω state, not an estimate. */
  public getEstimatedDrivenWheelRpm(): number {
    const g = this.groups.find((gr) => gr.isDriven);
    const omega = g && Number.isFinite(g.omegaRadS) ? Math.abs(g.omegaRadS) : 0;
    return (omega * 60) / (2 * Math.PI);
  }

  public setDriveControls(controls: VehicleDriveControls): void {
    this.driveControls = {
      engineForce: Number.isFinite(controls.engineForce) ? controls.engineForce : 0,
      brakeTotalForceN: Math.max(
        0,
        Number.isFinite(controls.brakeTotalForceN) ? controls.brakeTotalForceN : 0
      ),
      steering: Number.isFinite(controls.steering) ? controls.steering : 0,
      engineRunning: controls.engineRunning,
      drivelineInertiaAtWheelsKgM2: Math.max(
        0,
        Number.isFinite(controls.drivelineInertiaAtWheelsKgM2 ?? 0)
          ? (controls.drivelineInertiaAtWheelsKgM2 ?? 0)
          : 0
      ),
    };
    this.rigidBody?.wakeUp();
  }

  public getPhysicsSubstepCountForStep(): number {
    if (!this.rigidBody || !this.wheels) return 1;
    // landing: 1/180 steps keep suspension restitution near zero on hits up to 6 m/s
    if (this.landingBoostRemaining > 0) return 3;
    const vy = this.rigidBody.linvel().y;
    if (vy < VY_VERY_FAST_FALL) return 3;
    if (vy < VY_FAST_FALL) return 2;
    return 1;
  }

  // ── main step ──────────────────────────────────────────────────

  public beforeStep(dt: number): void {
    if (!this.rigidBody || !this.wheels || dt <= 1e-8) return;

    this.rigidBody.wakeUp();
    clampBodyAngularVelocity(this.rigidBody);
    // resets body forces inside; downforce points use real axle z from the GLB
    applyAeroAndRollingForces(
      this.rigidBody,
      this.physics,
      this.getGroundedNormalLoadFraction(),
      this.axleZOffsets()
    );
    applyYawRateDamping(this.rigidBody, this.handling.yawRateDamping);
    this.updateCasterCorrection(dt);
    this.updateSuspensionAndContacts(dt);
    this.solveTireForces(dt);
    this.integrateWheelVisualSpin(dt);
    this.captureWheelTelemetry(dt);

    if (this.landingBoostRemaining > 0) {
      this.landingBoostRemaining = Math.max(0, this.landingBoostRemaining - dt);
    }
  }

  /** Caster: steering correction toward the slide, smoothed and capped by lock. */
  private updateCasterCorrection(dt: number): void {
    const v = planarSpeedMs(this.rigidBody);
    let target = 0;
    if (v > CASTER_ASSIST.minSpeedMs) {
      const beta = bodySlipAngleRad(this.rigidBody);
      const excess = Math.abs(beta) - CASTER_ASSIST.deadzoneRad;
      if (excess > 0) target = Math.sign(beta) * CASTER_ASSIST.gain * excess;
    }
    const maxLock = Math.max(0.05, this.physics.maxSteeringAngle);
    target = clamp(target, -maxLock, maxLock);
    this.casterCorrectionRad +=
      (target - this.casterCorrectionRad) * (1 - Math.exp(-dt / Math.max(1e-3, CASTER_ASSIST.tauS)));
  }

  /**
   * Effective steering angle: driver + caster, within lock. Caster
   * (self-aligning) applies to the front axle only - steering the rear axle
   * into the slide amplifies yaw instead of damping it.
   */
  private effectiveSteeringRad(withCaster: boolean): number {
    const maxLock = Math.max(0.05, this.physics.maxSteeringAngle);
    const caster = withCaster ? this.casterCorrectionRad : 0;
    return clamp(this.driveControls.steering + caster, -maxLock, maxLock);
  }

  /**
   * Ackermann: per-wheel angle from the common δ and the wheel's lateral offset.
   * The inner wheel steers tighter (one turn center for both).
   * R = L/tan(δ); δ_w = atan(L / (R - x_w)), x_w signed (+ = right).
   */
  private ackermannSteerRad(deltaCommon: number, lateralOffsetM: number): number {
    if (Math.abs(deltaCommon) < 1e-4) return deltaCommon;
    const L = Math.max(0.5, this.physics.wheelBase);
    const R = L / Math.tan(deltaCommon);
    return Math.atan(L / (R - lateralOffsetM));
  }

  // ── suspension + contacts ─────────────────────────────────────

  private updateSuspensionAndContacts(dt: number): void {
    const body = this.rigidBody!;
    const wheels = this.wheels!;
    const susp = this.suspension;
    const t = body.translation();
    const rot = body.rotation();
    const m = Matrix.Identity();
    Matrix.FromQuaternionToRef(new Quaternion(rot.x, rot.y, rot.z, rot.w), m);

    const upW = Vector3.TransformNormal(new Vector3(0, 1, 0), m);
    if (upW.lengthSquared() < 1e-10) return;
    upW.normalize();

    for (const key of WHEEL_KEYS) {
      const w = wheels[key];
      const hpLocal = new Vector3(w.hardPointOffset.x, w.hardPointOffset.y, w.hardPointOffset.z);
      const hpOff = Vector3.TransformNormal(hpLocal, m);
      const originX = t.x + hpOff.x;
      const originY = t.y + hpOff.y;
      const originZ = t.z + hpOff.z;

      // ray down the strut axis (-body up), length = full rebound + radius
      const rayLen = susp.restLength + susp.reboundTravel + w.radius;
      const dir = new RAPIER.Vector3(-upW.x * rayLen, -upW.y * rayLen, -upW.z * rayLen);
      const ray = new RAPIER.Ray(new RAPIER.Vector3(originX, originY, originZ), dir);
      const hit = this.world.castRayAndGetNormal(ray, 1.0, true, undefined, undefined, undefined, body, undefined);

      if (!hit) {
        // airborne: full rebound, lateral force relaxes to zero
        w.inContact = false;
        w.normalN = 0;
        w.suspensionLength = susp.restLength + susp.reboundTravel;
        w.prevCompression = -susp.reboundTravel;
        w.fyRelaxedN *= Math.max(0, 1 - dt * 8);
        continue;
      }

      const hitDist = hit.timeOfImpact * rayLen;
      const rawLength = hitDist - w.radius;
      const minLen = susp.restLength - susp.bumpTravel - susp.bumpStopRangeM;
      const maxLen = susp.restLength + susp.reboundTravel;
      const suspensionLength = clamp(rawLength, minLen, maxLen);
      const compression = susp.restLength - suspensionLength; // + = compressed

      const compressionVel = clamp(
        (compression - w.prevCompression) / dt,
        -MAX_DAMPER_VEL_MS,
        MAX_DAMPER_VEL_MS
      );

      // N = corner static preload (CoM weight split) + spring + bump stop + damper
      let force = w.staticN + susp.wheelRateNM * compression;
      if (compression > susp.bumpTravel && compressionVel > BUMP_STOP_RELEASE_VEL_MS) {
        force += susp.bumpStopRateNM * (compression - susp.bumpTravel);
      }
      force += (compressionVel > 0 ? w.compressionDampingNsM : w.reboundDampingNsM) * compressionVel;
      const normalN = clamp(force, 0, ABS_MAX_SUSPENSION_FORCE_PER_WHEEL);

      // contact normal; near-vertical walls give no tire basis - use body up
      let nx = hit.normal.x, ny = hit.normal.y, nz = hit.normal.z;
      const nDotUp = nx * upW.x + ny * upW.y + nz * upW.z;
      if (nDotUp < 0) { nx = -nx; ny = -ny; nz = -nz; }
      if (Math.abs(nDotUp) < 0.15) { nx = upW.x; ny = upW.y; nz = upW.z; }

      w.inContact = true;
      w.normalN = normalN;
      w.suspensionLength = suspensionLength;
      w.prevCompression = compression;
      w.contactPoint = {
        x: originX - upW.x * hitDist,
        y: originY - upW.y * hitDist,
        z: originZ - upW.z * hitDist,
      };
      w.contactNormal = { x: nx, y: ny, z: nz };
    }

    // anti-roll bars: transfer load across each axle BEFORE strut forces apply
    this.applyAntiRollBars(upW, t, m);

    // strut force along body up at the contact point => honest weight transfer and roll
    for (const key of WHEEL_KEYS) {
      const w = wheels[key];
      if (!w.inContact || w.normalN <= 0) continue;
      body.addForceAtPoint(
        new RAPIER.Vector3(upW.x * w.normalN, upW.y * w.normalN, upW.z * w.normalN),
        new RAPIER.Vector3(w.contactPoint.x, w.contactPoint.y, w.contactPoint.z),
        true
      );
    }
  }

  /**
   * Anti-roll bar: F = k·(x_L - x_R) per wheel, opposite signs. The compressed
   * side gets more load - roll is resisted, load moves across the axle.
   * A grounded wheel takes its share through the contact (affects tire grip);
   * a hanging wheel cannot, so the bar's reaction goes straight to the chassis
   * at the hard point - otherwise the axle gets a moment out of nowhere.
   */
  private applyAntiRollBars(
    upW: Vector3,
    bodyT: { x: number; y: number; z: number },
    bodyM: Matrix
  ): void {
    const wheels = this.wheels!;
    const axles: Array<[WheelKey, WheelKey, number]> = [
      ['FL', 'FR', this.suspension.arbFrontNM],
      ['RL', 'RR', this.suspension.arbRearNM],
    ];
    for (const [lKey, rKey, k] of axles) {
      if (k <= 0) continue;
      const l = wheels[lKey];
      const r = wheels[rKey];
      // prevCompression was updated this tick in updateSuspensionAndContacts
      const f = k * (l.prevCompression - r.prevCompression);
      if (f === 0) continue;
      this.applyArbSide(l, f, upW, bodyT, bodyM);
      this.applyArbSide(r, -f, upW, bodyT, bodyM);
    }
  }

  private applyArbSide(
    w: CWheel,
    signedF: number,
    upW: Vector3,
    bodyT: { x: number; y: number; z: number },
    bodyM: Matrix
  ): void {
    if (w.inContact) {
      w.normalN = clamp(w.normalN + signedF, 0, ABS_MAX_SUSPENSION_FORCE_PER_WHEEL);
      return;
    }
    const hp = Vector3.TransformNormal(
      new Vector3(w.hardPointOffset.x, w.hardPointOffset.y, w.hardPointOffset.z),
      bodyM
    );
    this.rigidBody!.addForceAtPoint(
      new RAPIER.Vector3(upW.x * signedF, upW.y * signedF, upW.z * signedF),
      new RAPIER.Vector3(bodyT.x + hp.x, bodyT.y + hp.y, bodyT.z + hp.z),
      true
    );
  }

  // ── tires: ω integration + combined slip ──────────────────────

  private solveTireForces(dt: number): void {
    const body = this.rigidBody!;
    const wheels = this.wheels!;
    const p = this.tireParams;
    const mu = this.physics.tires.friction * TRACK_COLLIDER_FRICTION;

    const t = body.translation();
    const rot = body.rotation();
    const lv = body.linvel();
    const av = body.angvel();
    const m = Matrix.Identity();
    Matrix.FromQuaternionToRef(new Quaternion(rot.x, rot.y, rot.z, rot.w), m);

    // world CoM: Rapier linvel is the center-of-mass velocity
    const comL = this.physics.centerOfMass;
    const comOff = Vector3.TransformNormal(new Vector3(comL.x, comL.y, comL.z), m);
    const comWX = t.x + comOff.x, comWY = t.y + comOff.y, comWZ = t.z + comOff.z;

    const steeringFront = this.effectiveSteeringRad(true);
    const steeringRear = this.effectiveSteeringRad(false);

    // contact kinematics per substep (body is "frozen" inside the inner loop)
    type ContactRow = {
      w: CWheel;
      fwd: { x: number; y: number; z: number };
      side: { x: number; y: number; z: number };
      vLong: number;
      vLat: number;
      slipBlendFx: number; // longitudinal kinematic=>slip blend (narrow, by contact speed)
      slipBlendFy: number; // lateral blend (wide, by |vLong| - against the oscillator)
      gripN: number;
      fxImpulse: number;
      fyImpulse: number;
    };
    const rows = new Map<WheelKey, ContactRow>();

    for (const key of WHEEL_KEYS) {
      const w = wheels[key];
      w.lastFxN = 0;
      w.lastFyN = 0;
      w.lastSlipRatio = 0;
      if (!w.inContact || w.normalN <= 0) continue;

      const delta = w.isSteering
        ? this.ackermannSteerRad(w.isFront ? steeringFront : steeringRear, w.centerOffset.x)
        : 0;
      // wheel axes with steering (yaw around body Y, same as the visual)
      const sinD = Math.sin(delta), cosD = Math.cos(delta);
      const fwdL = new Vector3(sinD, 0, cosD);
      const fwdW = Vector3.TransformNormal(fwdL, m);

      const n = w.contactNormal;
      // project forward onto the contact plane
      let fx = fwdW.x - n.x * (fwdW.x * n.x + fwdW.y * n.y + fwdW.z * n.z);
      let fy = fwdW.y - n.y * (fwdW.x * n.x + fwdW.y * n.y + fwdW.z * n.z);
      let fz = fwdW.z - n.z * (fwdW.x * n.x + fwdW.y * n.y + fwdW.z * n.z);
      const fLen = Math.sqrt(fx * fx + fy * fy + fz * fz);
      if (fLen < 1e-6) continue;
      fx /= fLen; fy /= fLen; fz /= fLen;
      // side = n × fwd (points right in Babylon's LH convention)
      const sx = n.y * fz - n.z * fy;
      const sy = n.z * fx - n.x * fz;
      const sz = n.x * fy - n.y * fx;

      // contact point velocity: v = v_com + ω × r
      const rx = w.contactPoint.x - comWX;
      const ry = w.contactPoint.y - comWY;
      const rz = w.contactPoint.z - comWZ;
      const vx = lv.x + (av.y * rz - av.z * ry);
      const vy = lv.y + (av.z * rx - av.x * rz);
      const vz = lv.z + (av.x * ry - av.y * rx);

      const vLong = vx * fx + vy * fy + vz * fz;
      const vLat = vx * sx + vy * sy + vz * sz;
      const contactSpeed = Math.hypot(vLong, vLat);
      const slipBlendFx = clamp((contactSpeed - 0.4) / p.lowSpeedBlendMs, 0, 1);
      const slipBlendFy = clamp(
        (Math.abs(vLong) - FY_BLEND_START_MS) / (FY_BLEND_END_MS - FY_BLEND_START_MS),
        0,
        1
      );
      const gripN = effectiveMu(mu, w.normalN, w.staticN, p.loadSensitivity) * w.normalN;

      rows.set(key, {
        w,
        fwd: { x: fx, y: fy, z: fz },
        side: { x: sx, y: sy, z: sz },
        vLong,
        vLat,
        slipBlendFx,
        slipBlendFy,
        gripN,
        fxImpulse: 0,
        fyImpulse: 0,
      });
    }

    // brake torques: fixed bias, lockup emerges on its own
    const totalBrakeN = this.driveControls.brakeTotalForceN;
    const brakeTorqueFor = (w: CWheel) => {
      const axleShare = w.isFront ? this.brakeBiasFront : 1 - this.brakeBiasFront;
      return totalBrakeN * axleShare * 0.5 * w.radius;
    };

    const h = dt / TIRE_INNER_STEPS;

    for (const group of this.groups) {
      const groupRows = group.keys
        .map((k) => rows.get(k))
        .filter((r): r is ContactRow => !!r);
      const avgRadius =
        group.keys.reduce((s, k) => s + wheels[k].radius, 0) / group.keys.length;
      const driveTorque = group.isDriven ? this.driveControls.engineForce * avgRadius : 0;
      const brakeTorque = group.keys.reduce((s, k) => s + brakeTorqueFor(wheels[k]), 0);
      // the driven axle carries the reflected engine inertia when the clutch is
      // engaged - without it the axle is ~10× too light and spins up in tens of ms
      const reflected = group.isDriven ? (this.driveControls.drivelineInertiaAtWheelsKgM2 ?? 0) : 0;
      const I = Math.max(0.1, group.inertia + reflected);

      for (let step = 0; step < TIRE_INNER_STEPS; step++) {
        let omega = group.omegaRadS;
        let sumFxTorque = 0;
        let sumKwTorque = 0; // Σ ∂(Fx·r)/∂ω over the group - semi-implicit denominator
        const stepFx: number[] = [];
        const stepFy: number[] = [];
        const stepKw: number[] = []; // per-wheel ∂Fx/∂ω - force correction after dOmega

        for (const r of groupRows) {
          const w = r.w;
          const denom = Math.max(Math.abs(r.vLong), SLIP_DENOM_MIN_MS);
          const slipVelLong = omega * w.radius - r.vLong;
          const slipRatio = slipVelLong / denom;
          const slipAngle = Math.atan2(r.vLat, denom);

          const slip = combinedSlipForces(slipRatio, slipAngle, w.normalN, mu, w.staticN, p);

          // low-speed model: friction ∝ slip velocity (slip formulas blow up as v=>0)
          let fxLow = r.gripN * clamp(slipVelLong / p.lowSpeedFullSlipMs, -1, 1);
          let fyLow = -r.gripN * clamp(r.vLat / p.lowSpeedFullSlipMs, -1, 1);
          const lowMag = Math.hypot(fxLow, fyLow);
          if (lowMag > r.gripN && lowMag > 1e-9) {
            const s = r.gripN / lowMag;
            fxLow *= s;
            fyLow *= s;
          }

          const fxN = fxLow + (slip.fxN - fxLow) * r.slipBlendFx;

          // relaxation length applies ONLY to the slip part of Fy: lag on the
          // kinematic part at low speed turns the axle into an oscillator
          const relaxAlpha = clamp(
            (h * Math.max(Math.abs(r.vLong), 1)) / p.relaxationLengthM,
            0,
            1
          );
          w.fyRelaxedN += (slip.fyN - w.fyRelaxedN) * relaxAlpha;
          let fyApplied = fyLow * (1 - r.slipBlendFy) + w.fyRelaxedN * r.slipBlendFy;

          // anti-overshoot: a damping Fy cannot flip vLat through zero in one body step
          const fyCapN = (this.physics.mass / 4) * Math.abs(r.vLat) / dt;
          if (fyApplied * r.vLat < 0 && Math.abs(fyApplied) > fyCapN) {
            fyApplied = Math.sign(fyApplied) * fyCapN;
          }

          // ∂Fx/∂ω for the semi-implicit step: blend of low-speed and slip derivatives
          const dLow =
            Math.abs(slipVelLong) < p.lowSpeedFullSlipMs
              ? (r.gripN * w.radius) / p.lowSpeedFullSlipMs
              : 0;
          const dSlip =
            (r.gripN * longitudinalSlipStiffness(slipRatio, slipAngle, p) * w.radius) / denom;
          const kW = dLow * (1 - r.slipBlendFx) + dSlip * r.slipBlendFx;

          stepFx.push(fxN);
          stepFy.push(fyApplied);
          stepKw.push(kW);
          sumFxTorque += fxN * w.radius;
          sumKwTorque += kW * w.radius;
          w.lastSlipRatio = slipRatio;
        }

        // semi-implicit ω step: dω = h·(T - ΣFx·r)/(I + h·Σ∂(Fx·r)/∂ω).
        // The explicit step is unstable on the linear part of the curve (λ·h ≈ 3 > 2
        // even with reflected driveline inertia) - traction saws at 30 Hz.
        // The tire-stiffness denominator makes the step monotonic.
        const dOmega = (h * (driveTorque - sumFxTorque)) / (I + h * sumKwTorque);
        for (let i = 0; i < stepFx.length; i++) {
          stepFx[i] += stepKw[i] * dOmega;
        }
        omega += dOmega;
        // ABS-lite: brake torque scales by the group's worst slip this step
        let absScale = 1;
        if (brakeTorque > 0 && groupRows.length) {
          let minSlip = 0;
          for (const r of groupRows) minSlip = Math.min(minSlip, r.w.lastSlipRatio);
          const depth = -minSlip - ABS_ASSIST.slipStart;
          if (depth > 0) {
            absScale = Math.max(
              ABS_ASSIST.minBrakeScale,
              1 - depth / Math.max(1e-3, ABS_ASSIST.slipFull - ABS_ASSIST.slipStart)
            );
          }
        }
        // brake impulse applied after: no chatter, ω = 0 is reachable
        const brakeDeltaOmega = (brakeTorque * absScale * h) / I;
        omega = Math.sign(omega) * Math.max(0, Math.abs(omega) - brakeDeltaOmega);
        const maxOmega = MAX_WHEEL_TREAD_SPEED_MS / Math.max(0.05, avgRadius);
        omega = clamp(omega, -maxOmega, maxOmega);
        group.omegaRadS = omega;

        groupRows.forEach((r, i) => {
          r.fxImpulse += stepFx[i] * h;
          r.fyImpulse += stepFy[i] * h;
        });
      }

      // mean substep forces => body at contact points
      for (const r of groupRows) {
        const fxMean = r.fxImpulse / dt;
        const fyMean = r.fyImpulse / dt;
        r.w.lastFxN = fxMean;
        r.w.lastFyN = fyMean;
        body.addForceAtPoint(
          new RAPIER.Vector3(
            r.fwd.x * fxMean + r.side.x * fyMean,
            r.fwd.y * fxMean + r.side.y * fyMean,
            r.fwd.z * fxMean + r.side.z * fyMean
          ),
          new RAPIER.Vector3(r.w.contactPoint.x, r.w.contactPoint.y, r.w.contactPoint.z),
          true
        );
      }
    }

    // requested vs applied traction telemetry
    const drivenKeys = WHEEL_KEYS.filter((k) => wheels[k].isDriven);
    const perWheelReq = drivenKeys.length ? this.driveControls.engineForce / drivenKeys.length : 0;
    for (const key of WHEEL_KEYS) {
      const w = wheels[key];
      w.lastRequestedEngineN = w.isDriven ? perWheelReq : 0;
      w.lastAppliedEngineN = w.isDriven ? w.lastFxN : 0;
    }
  }

  // ── aero + rolling resistance ──────────────────────────────────

  private getGroundedNormalLoadFraction(): number {
    if (!this.wheels) return 0;
    let sumN = 0;
    for (const key of WHEEL_KEYS) {
      sumN += this.wheels[key].inContact ? this.wheels[key].normalN : 0;
    }
    const totalWeight = Math.max(1e-6, this.physics.mass * GRAVITY);
    return Math.min(1, sumN / totalWeight);
  }

  // ── visuals ────────────────────────────────────────────────────

  private integrateWheelVisualSpin(dt: number): void {
    if (!this.wheels) return;
    for (const group of this.groups) {
      for (const key of group.keys) {
        this.wheels[key].spinAngleRad += group.omegaRadS * dt;
      }
    }
  }

  public getWheelPoses(): Record<WheelKey, { position: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } }> | null {
    if (!this.wheels || !this.rigidBody) return null;

    const chassisPos = this.rigidBody.translation();
    const bodyRot = this.rigidBody.rotation();
    const bodyQuat = new Quaternion(bodyRot.x, bodyRot.y, bodyRot.z, bodyRot.w);
    const bodyMatrix = Matrix.Identity();
    Matrix.FromQuaternionToRef(bodyQuat, bodyMatrix);

    const buildPose = (key: WheelKey) => {
      const wheel = this.wheels![key];
      const suspensionDelta = this.suspension.restLength - Math.max(0.01, wheel.suspensionLength);
      const localWheelPos = new Vector3(
        wheel.centerOffset.x,
        wheel.centerOffset.y + suspensionDelta,
        wheel.centerOffset.z
      );
      const offsetWorld = Vector3.TransformNormal(localWheelPos, bodyMatrix);
      const localWheelRot = Quaternion.RotationYawPitchRoll(
        wheel.isSteering
          ? this.ackermannSteerRad(this.effectiveSteeringRad(wheel.isFront), wheel.centerOffset.x)
          : 0,
        wheel.spinAngleRad,
        0
      );
      const worldWheelRot = bodyQuat.multiply(localWheelRot);
      return {
        position: {
          x: chassisPos.x + offsetWorld.x,
          y: chassisPos.y + offsetWorld.y,
          z: chassisPos.z + offsetWorld.z
        },
        rotation: { x: worldWheelRot.x, y: worldWheelRot.y, z: worldWheelRot.z, w: worldWheelRot.w }
      };
    };

    return { FL: buildPose('FL'), FR: buildPose('FR'), RL: buildPose('RL'), RR: buildPose('RR') };
  }

  // ── forces / telemetry ────────────────────────────────────────

  public getForces(): VehicleForces {
    return this.forcesTelemetry.compute(this.rigidBody, this.physics, this.lastWheelTraction);
  }

  private captureWheelTelemetry(dt: number): void {
    if (!this.wheels || dt <= 1e-8) return;
    const mu = this.physics.tires.friction * TRACK_COLLIDER_FRICTION;

    const omegaByKey = new Map<WheelKey, number>();
    for (const group of this.groups) {
      for (const key of group.keys) omegaByKey.set(key, group.omegaRadS);
    }

    const wheelsDbg: WheelTractionDebug[] = [];
    let totalReq = 0, totalApp = 0, sumFwd = 0, sumSide = 0;

    for (const key of WHEEL_KEYS) {
      const w = this.wheels[key];
      const gripN = w.inContact
        ? effectiveMu(mu, w.normalN, w.staticN, this.tireParams.loadSensitivity) * w.normalN
        : 0;
      totalReq += w.lastRequestedEngineN;
      totalApp += w.lastAppliedEngineN;
      sumFwd += w.lastFxN;
      sumSide += w.lastFyN;
      wheelsDbg.push({
        key,
        inContact: w.inContact,
        suspensionForceN: w.normalN,
        maxLongitudinalN: gripN,
        requestedEngineN: w.lastRequestedEngineN,
        appliedEngineN: w.lastAppliedEngineN,
        forwardImpulseNs: w.lastFxN * dt,
        sideImpulseNs: w.lastFyN * dt,
        forwardForceN: w.lastFxN,
        sideForceN: w.lastFyN,
        slipRatio: w.lastSlipRatio,
        wheelOmegaRadS: omegaByKey.get(key) ?? 0,
      });
    }

    this.lastWheelTraction = {
      mu,
      // average per corner - per-corner statics live in each wheel entry
      staticLoadPerWheelN: (this.physics.mass * GRAVITY) / 4,
      requestedEngineTotalN: totalReq,
      appliedEngineTotalN: totalApp,
      sumForwardForceN: sumFwd,
      sumSideForceN: sumSide,
      wheels: wheelsDbg,
    };
  }

  // ── CoM debug ─────────────────────────────────────────────────

  public debugCenterMass(on: boolean): void {
    this.comDebug.setEnabled(on);
  }

  public syncComDebug(): void {
    this.comDebug.sync(this.rigidBody, this.physics.centerOfMass);
  }

  public dispose(): void {
    this.comDebug.dispose();
    this.chassisRig.dispose();
    this.wheels = null;
    this.groups = [];
    this.lastWheelTraction = null;
    if (this.rigidBody) {
      this.world.removeRigidBody(this.rigidBody);
      this.rigidBody = undefined;
    }
  }

}
