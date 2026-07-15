import RAPIER from '@dimforge/rapier3d-compat';
import { Matrix, Quaternion, Vector3, MeshBuilder, StandardMaterial, Color3, Scene, Mesh } from '@babylonjs/core';
import { VehicleSpec, DriveLayout, SteeredAxle } from '../../types/vehicle-spec';
import { Bounds, WheelKey, WHEEL_KEYS, FRONT_KEYS, REAR_KEYS, WheelOffsets, WheelSizes, XYZ } from './rigs/wheel.types';
import { GRAVITY, AIR_DENSITY } from '../../constants';
import { clamp } from '../../utils/math';

/**
 * Shared helpers for CustomVehiclePhysics: body kinematics, aero/rolling forces,
 * wheel geometry, mass properties, force telemetry, CoM gizmo.
 * Model-specific parts (suspension, tire, wheel ω) stay in the class.
 */

type PhysicsConfig = VehicleSpec['physics'];
type Quat = { x: number; y: number; z: number; w: number };

// ── shared types ─────────────────────────────────────────────────

// per-wheel data for the debug panel
export type WheelTractionDebug = {
  key: WheelKey;
  inContact: boolean;
  /** Suspension normal force, N. */
  suspensionForceN: number;
  /** Max longitudinal grip force μ_eff·N, N. */
  maxLongitudinalN: number;
  /** Engine force requested for this wheel before limits, N. */
  requestedEngineN: number;
  /** Longitudinal force actually applied, N. */
  appliedEngineN: number;
  forwardImpulseNs: number;
  sideImpulseNs: number;
  forwardForceN: number;
  sideForceN: number;
  /** (ω·r - v∥) / max(|v∥|, ε) - longitudinal slip ratio. */
  slipRatio: number;
  wheelOmegaRadS: number;
};

export type VehicleTractionSnapshot = {
  /** Effective μ ≈ tires.friction × track surface. */
  mu: number;
  staticLoadPerWheelN: number;
  requestedEngineTotalN: number;
  appliedEngineTotalN: number;
  sumForwardForceN: number;
  sumSideForceN: number;
  wheels: WheelTractionDebug[];
};

export type VehicleForces = {
  gravity: number;
  drag: number;
  downforce: number;
  centrifugal: number;
  centrifugalDir: number;
  speedMs: number;
  lateralG: number;
  longitudinalG: number;
  traction: VehicleTractionSnapshot | null;
};

export type VehicleDriveControls = {
  engineForce: number;
  /** Total brake force, N. Axle split (bias) lives in the wheel model. */
  brakeTotalForceN: number;
  steering: number;
  engineRunning: boolean;
  /** Driveline inertia reflected to the driven axle (I_fly·ratio²·clutch), kg·m². */
  drivelineInertiaAtWheelsKgM2?: number;
};

// ── shared constants ─────────────────────────────────────────────

/** Body angular velocity cap: catches post-crash spin, leaves normal yaw/roll alone. */
export const MAX_BODY_ANGVEL_RAD_S = 12;
/** Vertical speed thresholds for extra physics substeps while falling. */
export const VY_FAST_FALL = -2.0;
export const VY_VERY_FAST_FALL = -8.0;
/** Suspension boost duration after spawn, seconds (not frames - substep-rate independent). */
export const LANDING_BOOST_DURATION_S = 0.6;

// ── wheel keys ───────────────────────────────────────────────────

/** Drive layout => driven wheel set. */
export function drivenWheelKeys(layout: DriveLayout): readonly WheelKey[] {
  switch (layout) {
    case 'fwd': return FRONT_KEYS;
    case 'awd': return WHEEL_KEYS;
    case 'rwd':
    default: return REAR_KEYS;
  }
}

/** Steered axle => steered wheel set. */
export function steeredWheelKeys(axle: SteeredAxle): readonly WheelKey[] {
  switch (axle) {
    case 'rear': return REAR_KEYS;
    case 'all': return WHEEL_KEYS;
    case 'front':
    default: return FRONT_KEYS;
  }
}

// ── body kinematics ──────────────────────────────────────────────

export function bodyPosition(rb?: RAPIER.RigidBody): XYZ {
  if (!rb) return { x: 0, y: 0, z: 0 };
  const t = rb.translation();
  return { x: t.x, y: t.y, z: t.z };
}

export function bodyRotation(rb?: RAPIER.RigidBody): Quat {
  if (!rb) return { x: 0, y: 0, z: 0, w: 1 };
  const r = rb.rotation();
  return { x: r.x, y: r.y, z: r.z, w: r.w };
}

/** Horizontal-plane speed, m/s. */
export function planarSpeedMs(rb?: RAPIER.RigidBody): number {
  if (!rb) return 0;
  const lv = rb.linvel();
  return Math.sqrt(lv.x * lv.x + lv.z * lv.z);
}

/** Body slip angle β in the yaw plane, signed (+ = rear stepping left). */
export function bodySlipAngleRad(rb?: RAPIER.RigidBody): number {
  if (!rb) return 0;
  const lv = rb.linvel();
  const r = rb.rotation();
  const fwdX = 2 * (r.x * r.z + r.w * r.y);
  const fwdZ = 1 - 2 * (r.x * r.x + r.y * r.y);
  const rightX = 1 - 2 * (r.y * r.y + r.z * r.z);
  const rightZ = 2 * (r.x * r.z - r.w * r.y);
  const vLong = lv.x * fwdX + lv.z * fwdZ;
  const vLat = lv.x * rightX + lv.z * rightZ;
  return Math.atan2(vLat, Math.abs(vLong));
}

/** Lateral (centripetal) acceleration |ω_y|·v, m/s². */
export function lateralAccelMs2(rb?: RAPIER.RigidBody): number {
  if (!rb) return 0;
  return Math.abs(rb.angvel().y) * planarSpeedMs(rb);
}

// ── shared body forces / limits ──────────────────────────────────

/**
 * Hard body angular velocity cap: with angularDamping = 0 a heavy impact can
 * spin the body without limit. Catches only that pathology.
 */
export function clampBodyAngularVelocity(rb?: RAPIER.RigidBody): void {
  if (!rb) return;
  const av = rb.angvel();
  const magSq = av.x * av.x + av.y * av.y + av.z * av.z;
  if (magSq > MAX_BODY_ANGVEL_RAD_S * MAX_BODY_ANGVEL_RAD_S) {
    const s = MAX_BODY_ANGVEL_RAD_S / Math.sqrt(magSq);
    rb.setAngvel({ x: av.x * s, y: av.y * s, z: av.z * s }, true);
  }
}

/**
 * Optional yaw rate damping (physics.handling.yawRateDamping). Substitutes the
 * damping real tires/suspension would give. Default 0.
 */
export function applyYawRateDamping(rb: RAPIER.RigidBody | undefined, k: number): void {
  if (k <= 0 || !rb) return;
  const av = rb.angvel();
  rb.addTorque(new RAPIER.Vector3(0, -k * av.y, 0), true);
}

/**
 * Aero (drag + downforce along body up) and rolling resistance.
 * Resets accumulated body forces/torques - call first in the substep.
 * `groundedLoadFrac` - weight fraction on the tires (0 airborne).
 */
export function applyAeroAndRollingForces(
  rb: RAPIER.RigidBody | undefined,
  physics: PhysicsConfig,
  groundedLoadFrac: number,
  /**
   * Axle z positions in body-local coords (from real GLB wheels). Fallback is
   * ±wheelBase/2 from origin - correct only if the origin sits mid-wheelbase.
   */
  axleZ?: { frontZ: number; rearZ: number }
): void {
  if (!rb) return;
  rb.resetForces(true);
  rb.resetTorques(true);

  const lv = rb.linvel();
  const speedH = Math.sqrt(lv.x * lv.x + lv.z * lv.z);
  const aero = physics.aero;
  const rho = aero.airDensity ?? AIR_DENSITY;
  const qA = 0.5 * rho * aero.frontalArea * speedH * speedH;

  if (speedH > 1e-3) {
    const invSpeedH = 1 / speedH;
    const dragMag = aero.dragCoefficient * qA;
    rb.addForce(
      new RAPIER.Vector3(-lv.x * invSpeedH * dragMag, 0, -lv.z * invSpeedH * dragMag),
      true
    );
  }

  const liftForce = aero.liftCoefficient * qA;

  const rollingCoeff = Math.max(0, physics.tires.rollingResistance);
  if (rollingCoeff > 0 && speedH > 1e-3 && groundedLoadFrac > 1e-6) {
    const invSpeedH = 1 / speedH;
    const downforceMag = Math.max(0, -liftForce);
    const normalLoadApprox = Math.max(0, physics.mass * GRAVITY + downforceMag);
    const rollingMag = rollingCoeff * normalLoadApprox * groundedLoadFrac;
    const rollingFade = Math.min(1, speedH / 0.35);
    rb.addForce(
      new RAPIER.Vector3(
        -lv.x * invSpeedH * rollingMag * rollingFade,
        0,
        -lv.z * invSpeedH * rollingMag * rollingFade
      ),
      true
    );
  }

  const rot = rb.rotation();
  const m = Matrix.Identity();
  Matrix.FromQuaternionToRef(new Quaternion(rot.x, rot.y, rot.z, rot.w), m);
  const localUp = Vector3.TransformNormal(new Vector3(0, 1, 0), m);
  if (localUp.lengthSquared() < 1e-10) return;
  localUp.normalize();

  // Aero balance: downforce at two axle points, not one force at CoM -
  // front/rear split changes balance with speed (splitter vs spoiler).
  const balanceFront = clamp(aero.balanceFront ?? 0.4, 0, 1);
  const halfBase = Math.max(0.25, physics.wheelBase / 2);
  const frontZ = axleZ?.frontZ ?? +halfBase;
  const rearZ = axleZ?.rearZ ?? -halfBase;
  const fwd = Vector3.TransformNormal(new Vector3(0, 0, 1), m);
  const t = rb.translation();
  const axlePoints: Array<[number, number]> = [
    [balanceFront, frontZ],
    [1 - balanceFront, rearZ],
  ];
  for (const [share, zOff] of axlePoints) {
    if (share <= 0) continue;
    const f = liftForce * share;
    rb.addForceAtPoint(
      new RAPIER.Vector3(localUp.x * f, localUp.y * f, localUp.z * f),
      new RAPIER.Vector3(t.x + fwd.x * zOff, t.y + fwd.y * zOff, t.z + fwd.z * zOff),
      true
    );
  }
}

// ── wheel geometry / mass properties ─────────────────────────────

export function computeLowestWheelTreadPlaneY(
  offsets: WheelOffsets,
  sizes: WheelSizes
): number | undefined {
  let minBottom = Infinity;
  for (const key of WHEEL_KEYS) {
    const cy = offsets[key].y;
    const r = Math.max(0.05, Math.abs(sizes[key].y) / 2);
    minBottom = Math.min(minBottom, cy - r);
  }
  return Number.isFinite(minBottom) ? minBottom : undefined;
}

export function resolveWheelOffsets(
  physics: PhysicsConfig,
  bounds: Bounds,
  wheelOffsets?: WheelOffsets
): WheelOffsets {
  if (wheelOffsets) return wheelOffsets;
  const halfWheelBase = physics.wheelBase / 2;
  const halfTrack = physics.trackWidth / 2;
  const wheelY = bounds.center.y - bounds.size.y / 2 + physics.wheelRadius * 0.65;
  return {
    FL: { x: bounds.center.x - halfTrack, y: wheelY, z: bounds.center.z + halfWheelBase },
    FR: { x: bounds.center.x + halfTrack, y: wheelY, z: bounds.center.z + halfWheelBase },
    RL: { x: bounds.center.x - halfTrack, y: wheelY, z: bounds.center.z - halfWheelBase },
    RR: { x: bounds.center.x + halfTrack, y: wheelY, z: bounds.center.z - halfWheelBase },
  };
}

export function resolveWheelSizes(physics: PhysicsConfig, wheelSizes?: WheelSizes): WheelSizes {
  if (wheelSizes) return wheelSizes;
  const r = physics.wheelRadius;
  const fallbackWheelSize = { x: r * 0.6, y: r * 2, z: r * 2 };
  return { FL: fallbackWheelSize, FR: fallbackWheelSize, RL: fallbackWheelSize, RR: fallbackWheelSize };
}

/**
 * The bounding-box formula spreads mass to the box corners and overshoots a real
 * car's yaw inertia by ~30-50% (engine and gear sit near the center). Too much
 * I_yaw = the car is lazy to turn in and lazy to recover from slides.
 */
const INERTIA_YAW_SCALE = 0.75;

/** Mass, box inertia and CoM on the rigid body. */
export function applyMassProperties(
  rb: RAPIER.RigidBody | undefined,
  massInput: number,
  chassisSize: XYZ,
  centerOfMass: XYZ
): void {
  if (!rb) return;
  const mass = Math.max(1, massInput);
  const sizeX = Math.max(0.1, chassisSize.x);
  const sizeY = Math.max(0.1, chassisSize.y);
  const sizeZ = Math.max(0.1, chassisSize.z);
  const inertia = new RAPIER.Vector3(
    (mass * (sizeY * sizeY + sizeZ * sizeZ)) / 12,
    (INERTIA_YAW_SCALE * mass * (sizeX * sizeX + sizeZ * sizeZ)) / 12,
    (mass * (sizeX * sizeX + sizeY * sizeY)) / 12
  );
  rb.setAdditionalMassProperties(
    mass,
    new RAPIER.Vector3(centerOfMass.x, centerOfMass.y, centerOfMass.z),
    inertia,
    { x: 0, y: 0, z: 0, w: 1 },
    true
  );
}

// ── force telemetry (smoothed G) ─────────────────────────────────

/** Keeps state between getForces calls (last-frame world velocity, smoothed G). */
export class ForcesTelemetry {
  private prevWorldVelX = 0;
  private prevWorldVelZ = 0;
  private prevForceTime = 0;
  private smoothLateralG = 0;
  private smoothLongitudinalG = 0;

  public compute(
    rb: RAPIER.RigidBody | undefined,
    physics: PhysicsConfig,
    traction: VehicleTractionSnapshot | null
  ): VehicleForces {
    const zero: VehicleForces = {
      gravity: 0, drag: 0, downforce: 0, centrifugal: 0, centrifugalDir: 0,
      speedMs: 0, lateralG: 0, longitudinalG: 0,
      traction: null,
    };
    if (!rb) return zero;

    const mass = physics.mass;
    const lv = rb.linvel();
    const rot = rb.rotation();
    const speed = Math.sqrt(lv.x * lv.x + lv.z * lv.z);

    const gravity = mass * GRAVITY;
    const aero = physics.aero;
    const rho = aero.airDensity ?? AIR_DENSITY;
    const qA = 0.5 * rho * aero.frontalArea * speed * speed;
    const drag = aero.dragCoefficient * qA;
    const downforce = Math.abs(aero.liftCoefficient) * qA;

    // world acceleration => projection onto local car axes
    const now = performance.now();
    const dt = this.prevForceTime > 0 ? (now - this.prevForceTime) / 1000 : 0;
    let latG = 0;
    let lonG = 0;

    if (dt > 0.001 && dt < 0.5) {
      const worldAccX = (lv.x - this.prevWorldVelX) / dt;
      const worldAccZ = (lv.z - this.prevWorldVelZ) / dt;
      const rw = rot.w, rx = rot.x, ry = rot.y, rz = rot.z;
      // right = rotate(1,0,0); forward = rotate(0,0,1) - same math as bodySlipAngleRad
      const rightX = 1 - 2 * (ry * ry + rz * rz);
      const rightZ = 2 * (rx * rz - rw * ry);
      const fwdX = 2 * (rx * rz + rw * ry);
      const fwdZ = 1 - 2 * (rx * rx + ry * ry);
      const rawLatG = (worldAccX * rightX + worldAccZ * rightZ) / GRAVITY;
      const rawLonG = (worldAccX * fwdX + worldAccZ * fwdZ) / GRAVITY;
      const smooth = 0.15;
      this.smoothLateralG += (rawLatG - this.smoothLateralG) * smooth;
      this.smoothLongitudinalG += (rawLonG - this.smoothLongitudinalG) * smooth;
      latG = this.smoothLateralG;
      lonG = this.smoothLongitudinalG;
    }
    this.prevWorldVelX = lv.x;
    this.prevWorldVelZ = lv.z;
    this.prevForceTime = now;

    // centrifugal = m·v²/R = m·v·|ω_y|
    const omegaY = rb.angvel().y;
    let centrifugal = 0;
    let centrifugalDir = 0;
    if (Math.abs(omegaY) > 0.01 && speed > 0.5) {
      centrifugal = mass * speed * Math.abs(omegaY);
      centrifugalDir = omegaY > 0 ? -1 : 1;
    }

    return {
      gravity, drag, downforce, centrifugal, centrifugalDir, speedMs: speed,
      lateralG: latG, longitudinalG: lonG,
      traction,
    };
  }
}

// ── CoM debug gizmo ──────────────────────────────────────────────

/** Green sphere at the center of mass (debug). */
export class ComDebugGizmo {
  private mesh?: Mesh;

  constructor(private readonly scene: Scene) {}

  public setEnabled(on: boolean): void {
    if (on && !this.mesh) {
      const sphere = MeshBuilder.CreateSphere('comDebug', { diameter: 0.25, segments: 8 }, this.scene);
      const mat = new StandardMaterial('comDebugMat', this.scene);
      mat.diffuseColor = new Color3(0, 1, 0);
      mat.emissiveColor = new Color3(0, 1, 0);
      mat.disableLighting = true;
      sphere.material = mat;
      sphere.isPickable = false;
      sphere.renderingGroupId = 1;
      this.mesh = sphere;
    }
    if (!on && this.mesh) {
      this.mesh.dispose();
      this.mesh = undefined;
    }
  }

  public sync(rb: RAPIER.RigidBody | undefined, centerOfMass: XYZ): void {
    if (!this.mesh || !rb) return;
    const t = rb.translation();
    const r = rb.rotation();
    const m = Matrix.Identity();
    Matrix.FromQuaternionToRef(new Quaternion(r.x, r.y, r.z, r.w), m);
    const worldOffset = Vector3.TransformNormal(
      new Vector3(centerOfMass.x, centerOfMass.y, centerOfMass.z),
      m
    );
    this.mesh.position.set(t.x + worldOffset.x, t.y + worldOffset.y, t.z + worldOffset.z);
  }

  public dispose(): void {
    this.mesh?.dispose();
    this.mesh = undefined;
  }
}
