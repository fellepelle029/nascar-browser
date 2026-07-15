import { ThrottleMode } from '../../../systems/input-system';
import { GRAVITY } from '../../../constants';
import { STEER_ASSIST, THROTTLE_STEER_RATE } from '../../../configs/driver-assist';
import { clampFinite } from '../../../utils/math';
import type { VehicleSpec } from '../../../types/vehicle-spec';
import {
  stepSteeringAngle,
  resolveVehicleHandling,
  type VehicleHandlingConfig,
  type VehicleHandlingResolved,
} from '../vehicle-handling-control';

// ── engine: dω/dt = (T_engine - T_drag + T_clutch) / I ──────────

const OMEGA_TO_RPM = 30 / Math.PI;
const RPM_TO_OMEGA = (2 * Math.PI) / 60;
const CLUTCH_BLEND_TAU_S = 0.07;
/**
 * Clutch is a Coulomb friction coupling with a locked/slipping state machine,
 * not a viscous spring. A viscous model (T = k·slipRpm) needs huge constant slip
 * to pass peak torque and feels like a CVT; k cannot be raised because explicit
 * integration goes unstable when τ < dt.
 */
/** Smoothing width of the Coulomb torque near sync, RPM. Kills sign chatter. */
const CLUTCH_SLIP_REF_RPM = 90;
/**
 * Lock threshold on |slip|. Must stay above the equilibrium slip of the smooth
 * zone under full torque (atanh(peak/cap)·ref), otherwise the clutch hangs in
 * slipping forever under throttle. Computed per vehicle in the Engine ctor.
 */
const CLUTCH_LOCK_SLIP_MIN_RPM = 45;
const CLUTCH_LOCK_SLIP_MARGIN = 1.6;
/** Lock only when the clutch is almost fully engaged. */
const CLUTCH_LOCK_ENGAGE_MIN = 0.98;
/** Default clutch capacity: race clutch holds ~1.4× peak torque. */
const CLUTCH_CAPACITY_PEAK_MULT = 1.4;
/** Rev limiter: fuel cut at redline, resumes below by hysteresis (gives bounce). */
const LIMITER_HYST_FRAC = 0.028;
const LIMITER_HYST_MIN_RPM = 120;
/** Idle governor: P-controller adds torque when RPM drops below idle. */
const IDLE_GOV_SPREAD_RPM = 250;
const IDLE_GOV_TORQUE_FRAC = 0.25;
const IDLE_GOV_TARGET_OFFSET_RPM = 40;
/** Hard RPM floor of a running engine (deep launch bog). */
const MIN_RUNNING_RPM_FRAC_OF_IDLE = 0.55;
const AUTO_SHIFT_RPM_STEP = 50;
const AUTO_UPSHIFT_POWER_MATCH = 0.985;
const AUTO_DOWNSHIFT_POWER_GAIN = 1.1;

type TorqueCurvePoint = {
  rpm: number;
  torqueNm: number;
};

function sampleLegacyTorqueFormula(
  cfg: VehicleSpec['physics']['engine'],
  rpm: number
): number {
  const peakTorque = Math.max(1, cfg.peakTorque ?? 1);
  const idleRpm = Math.max(1, cfg.idleRpm);
  const maxRpm = Math.max(idleRpm + 1, cfg.maxRpm);
  const peakTorqueRpm = Math.max(
    idleRpm + 1,
    Math.min(maxRpm - 1, cfg.peakTorqueRpm ?? (idleRpm + maxRpm) * 0.6)
  );
  const n = (rpm - idleRpm) / Math.max(1, maxRpm - idleRpm);
  const p = (peakTorqueRpm - idleRpm) / Math.max(1, maxRpm - idleRpm);
  const upP = Math.max(1e-4, p);
  const downP = Math.max(1e-4, 1 - p);
  const mult =
    n < p ? 0.3 + 0.7 * (n / upP) : 1.0 - 0.4 * ((n - p) / downP);
  return peakTorque * Math.max(0, mult);
}

function dedupeTorqueCurve(points: TorqueCurvePoint[]): TorqueCurvePoint[] {
  const sorted = [...points].sort((a, b) => a.rpm - b.rpm);
  const deduped: TorqueCurvePoint[] = [];
  for (const point of sorted) {
    const last = deduped[deduped.length - 1];
    if (last && Math.abs(last.rpm - point.rpm) < 1e-6) {
      last.torqueNm = point.torqueNm;
    } else {
      deduped.push({ rpm: point.rpm, torqueNm: point.torqueNm });
    }
  }
  return deduped;
}

/** Fallback torque curve built from peakTorque/peakTorqueRpm when no curve is given. */
function buildLegacyTorqueCurve(
  cfg: VehicleSpec['physics']['engine']
): TorqueCurvePoint[] {
  const idleRpm = Math.max(600, cfg.idleRpm);
  const maxRpm = Math.max(idleRpm + 400, cfg.maxRpm);
  const peakTorqueRpm = Math.max(
    idleRpm + 200,
    Math.min(maxRpm - 200, cfg.peakTorqueRpm ?? (idleRpm + maxRpm) * 0.6)
  );
  const curveRpms = [
    idleRpm,
    idleRpm + (peakTorqueRpm - idleRpm) * 0.35,
    idleRpm + (peakTorqueRpm - idleRpm) * 0.7,
    peakTorqueRpm,
    peakTorqueRpm + (maxRpm - peakTorqueRpm) * 0.35,
    peakTorqueRpm + (maxRpm - peakTorqueRpm) * 0.7,
    maxRpm,
  ];
  return dedupeTorqueCurve(
    curveRpms.map((rpm) => ({
      rpm: Math.round(rpm),
      torqueNm: sampleLegacyTorqueFormula(cfg, rpm),
    }))
  );
}

function normalizeTorqueCurve(
  cfg: VehicleSpec['physics']['engine']
): TorqueCurvePoint[] {
  const normalized = dedupeTorqueCurve(
    (cfg.torqueCurve ?? [])
      .filter(
        (point) =>
          Number.isFinite(point.rpm) &&
          Number.isFinite(point.torqueNm) &&
          point.rpm > 0 &&
          point.torqueNm >= 0
      )
      .map((point) => ({
        rpm: Number(point.rpm),
        torqueNm: Number(point.torqueNm),
      }))
  );
  if (normalized.length >= 2) {
    return normalized;
  }
  return buildLegacyTorqueCurve(cfg);
}

/** Linear interpolation over the torque curve, flat outside its ends. */
function sampleTorqueCurve(points: TorqueCurvePoint[], rpm: number): number {
  if (points.length === 0) return 0;
  if (rpm <= points[0].rpm) return points[0].torqueNm;
  for (let i = 1; i < points.length; i++) {
    const prev = points[i - 1];
    const next = points[i];
    if (rpm <= next.rpm) {
      const span = Math.max(1e-6, next.rpm - prev.rpm);
      const t = (rpm - prev.rpm) / span;
      return prev.torqueNm + (next.torqueNm - prev.torqueNm) * t;
    }
  }
  return points[points.length - 1].torqueNm;
}

class Engine {
  private rpm = 0;
  private running = false;
  private clutchEngagement = 0;
  /** Torque sent to the transmission at the engine shaft, N·m. Positive = drives wheels. */
  private clutchTorqueNm = 0;
  /** Clutch locked: RPM follows the axle, net torque goes to the axle. */
  private clutchLocked = false;
  /** Limiter fuel cut active (RPM ≥ redline, releases below redline - hyst). */
  private limiterCut = false;
  private readonly torqueCurve: TorqueCurvePoint[];
  private readonly peakTorqueNm: number;
  /** Full clutch capacity, N·m (config or default from peak torque). */
  private readonly clutchCapacityNm: number;
  /** Lock threshold on slip - derived from this vehicle's peak/capacity. */
  private readonly lockSlipRpm: number;
  private readonly limiterHystRpm: number;

  constructor(private cfg: EngineConfig) {
    this.torqueCurve = normalizeTorqueCurve(cfg);
    this.peakTorqueNm = Math.max(
      1,
      ...this.torqueCurve.map((point) => point.torqueNm)
    );
    this.clutchCapacityNm = Math.max(
      1,
      cfg.maxCouplingTorqueNm ?? this.peakTorqueNm * CLUTCH_CAPACITY_PEAK_MULT
    );
    // equilibrium slip of the smooth zone at peak torque: atanh(peak/cap)·ref
    const frac = Math.min(0.95, this.peakTorqueNm / this.clutchCapacityNm);
    const equilibriumSlipRpm =
      0.5 * Math.log((1 + frac) / (1 - frac)) * CLUTCH_SLIP_REF_RPM;
    this.lockSlipRpm = Math.max(
      CLUTCH_LOCK_SLIP_MIN_RPM,
      equilibriumSlipRpm * CLUTCH_LOCK_SLIP_MARGIN
    );
    this.limiterHystRpm = Math.max(
      LIMITER_HYST_MIN_RPM,
      this.getRedlineRpm() * LIMITER_HYST_FRAC
    );
  }

  start() {
    this.running = true;
    this.rpm = this.cfg.idleRpm;
    this.clutchLocked = false;
    this.limiterCut = false;
  }
  stop() {
    this.running = false;
    this.clutchLocked = false;
  }
  isRunning() {
    return this.running;
  }
  getRpm() {
    return this.rpm;
  }
  getIdleRpm() {
    return this.cfg.idleRpm;
  }
  getMaxRpm() {
    return this.cfg.maxRpm;
  }
  getRedlineRpm() {
    return Math.min(this.cfg.redlineRpm, this.cfg.maxRpm);
  }
  getLaunchSpreadRpm() {
    return this.cfg.clutchLaunchSpreadRpm ?? this.cfg.idleRpm * 0.55;
  }
  getClutch() {
    return this.clutchEngagement;
  }
  isClutchLocked() {
    return this.clutchLocked;
  }
  /** Torque to the transmission at the engine shaft, N·m (+ drives wheels, - engine braking). */
  getClutchTorqueNm() {
    return this.clutchTorqueNm;
  }
  /** Sequential shift: clutch kicked out instantly (ramp back = clutchReengageTime). */
  forceDeclutch() {
    this.clutchEngagement = 0;
    this.clutchLocked = false;
    this.clutchTorqueNm = 0;
  }
  getPeakTorque() {
    return this.peakTorqueNm;
  }
  getTorqueAtRpm(rpm: number) {
    const clampedRpm = Math.max(
      this.cfg.idleRpm,
      Math.min(this.cfg.maxRpm, rpm)
    );
    return sampleTorqueCurve(this.torqueCurve, clampedRpm);
  }
  getPowerAtRpm(rpm: number) {
    return this.getTorqueAtRpm(rpm) * Math.max(0, rpm) * RPM_TO_OMEGA;
  }

  setRpm(v: number) {
    this.rpm = Math.max(this.cfg.idleRpm, Math.min(this.cfg.maxRpm, v));
  }

  /**
   * drivelineRpm = wheelRpm · gearRatio · finalDrive (input shaft RPM when locked).
   *
   * Two modes:
   * - locked: RPM ≡ drivelineRpm, net torque (Te - drag) goes to the axle; the
   *   axle integrates it in the wheel model (engine inertia is reflected there,
   *   see drivelineInertiaAtWheelsKgM2). One DOF, zero slip by construction.
   * - slipping/open: own ω integration. Coulomb clutch T = cap·eng·tanh(slip/ref),
   *   semi-implicit in slip (explicit is unstable near sync: ∂T/∂ω·dt/I ≈ 4 > 2).
   */
  update(dt: number, throttle: number, drivelineRpm: number, clutchCommand: number) {
    if (!this.running) {
      this.rpm = Math.max(0, this.rpm - this.cfg.maxRpm * dt * 2);
      this.clutchEngagement = 0;
      this.clutchTorqueNm = 0;
      this.clutchLocked = false;
      return;
    }

    const { idleRpm, maxRpm } = this.cfg;
    const I = Math.max(0.02, this.cfg.flywheelInertia ?? 0.35);
    const rpmDrive = Math.max(0, drivelineRpm);
    const cmd = Math.max(0, Math.min(1, clutchCommand));
    const alpha = 1 - Math.exp(-dt / CLUTCH_BLEND_TAU_S);
    this.clutchEngagement += (cmd - this.clutchEngagement) * alpha;
    this.clutchEngagement = Math.max(0, Math.min(1, this.clutchEngagement));

    // rev limiter: fuel cut with hysteresis instead of a clamp - RPM bounces, not sticks
    const redline = this.getRedlineRpm();
    if (this.rpm >= redline) this.limiterCut = true;
    else if (this.rpm <= redline - this.limiterHystRpm) this.limiterCut = false;
    const thrEff = this.limiterCut ? 0 : Math.max(0, Math.min(1, throttle));

    const capacity = this.clutchCapacityNm * this.clutchEngagement;
    const slipRpm = this.rpm - rpmDrive;

    // state machine: unlock when the pedal releases, lock near sync
    if (this.clutchLocked && this.clutchEngagement < CLUTCH_LOCK_ENGAGE_MIN) {
      this.clutchLocked = false;
    }
    if (
      !this.clutchLocked &&
      this.clutchEngagement >= CLUTCH_LOCK_ENGAGE_MIN &&
      Math.abs(slipRpm) <= this.lockSlipRpm
    ) {
      this.clutchLocked = true;
    }

    if (this.clutchLocked) {
      // rigid link: tach = axle·ratio (lugging below idle allowed, as in a real car)
      this.rpm = Math.max(0, Math.min(maxRpm, rpmDrive));
      const Te = this.getTorqueAtRpm(this.rpm) * thrEff;
      this.clutchTorqueNm = Te - this.coastDragNm(thrEff);
      return;
    }

    // slipping/open: semi-implicit step on engine ω
    const Te = this.getTorqueAtRpm(this.rpm) * thrEff;
    const drag = this.coastDragNm(thrEff);
    const gov = this.governorTorqueNm();
    const refOmega = CLUTCH_SLIP_REF_RPM * RPM_TO_OMEGA;
    const f0 = Math.tanh((slipRpm * RPM_TO_OMEGA) / refOmega);
    // both stabilizing terms (∂T/∂ω ≥ 0) go into the denominator: clutch + governor.
    // Otherwise a config with low flywheel inertia / high peak torque blows up the explicit step.
    const govSlope =
      gov > 0
        ? (this.peakTorqueNm * IDLE_GOV_TORQUE_FRAC) /
          (IDLE_GOV_SPREAD_RPM * RPM_TO_OMEGA)
        : 0;
    const dTdOmega = (capacity * (1 - f0 * f0)) / refOmega + govSlope;
    const Tnet0 = Te + gov - drag - capacity * f0;
    const dOmega = (dt * Tnet0) / (I + dt * dTdOmega);
    this.clutchTorqueNm = Math.max(
      -capacity,
      Math.min(capacity, capacity * (f0 + ((1 - f0 * f0) / refOmega) * dOmega))
    );
    this.rpm += dOmega * OMEGA_TO_RPM;
    this.rpm = Math.max(
      idleRpm * MIN_RUNNING_RPM_FRAC_OF_IDLE,
      Math.min(maxRpm, this.rpm)
    );
  }

  /** Pumping/friction losses at closed throttle: engine braking + anti-run-up above idle. */
  private coastDragNm(thrEff: number): number {
    if (thrEff >= 0.02) return 0;
    const { idleRpm, maxRpm, engineBraking } = this.cfg;
    const braking = engineBraking * this.peakTorqueNm * Math.min(1, this.rpm / maxRpm);
    const antiRunUp =
      this.rpm > idleRpm
        ? this.peakTorqueNm * 0.03 * Math.min(1, (this.rpm - idleRpm) / 500)
        : 0;
    return braking + antiRunUp;
  }

  /** Idle P-controller: adds torque below idle+offset (free shaft only). */
  private governorTorqueNm(): number {
    const deficit =
      (this.cfg.idleRpm + IDLE_GOV_TARGET_OFFSET_RPM - this.rpm) / IDLE_GOV_SPREAD_RPM;
    return Math.max(0, Math.min(1, deficit)) * this.peakTorqueNm * IDLE_GOV_TORQUE_FRAC;
  }

  getTorque(): number {
    if (!this.running || this.rpm <= 0) return 0;
    return this.getTorqueAtRpm(this.rpm);
  }

}

// ── types ───────────────────────────────────────────────────────

export type EngineConfig = VehicleSpec['physics']['engine'];
export type ThrottleConfig = VehicleSpec['physics']['throttle'];
export type TransmissionConfig = VehicleSpec['physics']['transmission'];
export type BrakesConfig = VehicleSpec['physics']['brakes'];

export interface DrivetrainInput {
  throttlePressed: boolean;
  brakePressed: boolean;
  steer: number;
  throttleMode: ThrottleMode;
  throttleHold: boolean;
}

/** Static chassis data for the traction assist: lateral tire budget estimate. */
export interface DrivetrainChassisInfo {
  wheelBaseM: number;
  tireMu: number;
  /** 0.5·ρ·A·|Cl|, N·s²/m² - downforce = this × v². 0 = ignore. */
  downforcePerV2N?: number;
  /** Static weight fraction on the driven axle 0…1. Default 0.5. */
  drivenAxleWeightFrac?: number;
  /** Aero downforce fraction on the driven axle 0…1. Default 0.5. */
  drivenAxleAeroFrac?: number;
}

export interface DrivetrainFeedback {
  wheelRpm: number;
  speedMs: number;
  mass: number;
  wheelRadius: number;
  maxSteeringAngle: number;
  /** Body slip angle β, rad - for the yaw-catch assist layer. */
  bodySlipAngleRad?: number;
  /** Measured lateral acceleration, m/s² (steering kinematics lie during a slide). */
  latAccelMs2?: number;
}

export interface DrivetrainOutput {
  engineForce: number;
  /** Total brake force, N (m·g·G·pedal). Axle split (bias) lives in the wheel model. */
  brakeTotalForceN: number;
  steering: number;
  engineRunning: boolean;
  /**
   * Driveline inertia reflected to the driven axle: I_fly·(gear·final)²·clutch, kg·m².
   * Without it the axle is ~10× lighter than real and spins up into deep
   * wheelspin within tens of milliseconds.
   */
  drivelineInertiaAtWheelsKgM2: number;
}

export interface DrivetrainState {
  rpm: number;
  gear: number;
  /** Throttle pedal position 0…1 for UI. Source of truth is throttleVal inside Drivetrain. */
  pedal: number;
  brake: number;
  engineRunning: boolean;
  torque: number;
  /** Current throttle cap 0…1 from the assist (1 = no cut). For HUD/debug. */
  tractionCap: number;
  /** Smoothed auto-clutch engagement 0…1. */
  clutch: number;
  /** Clutch locked (RPM rigidly tied to the axle). */
  clutchLocked: boolean;
  /** Smoothed steering angle, rad (+ = right). For the HUD wheel. */
  steerAngleRad: number;
}

// ── traction assist resolve ─────────────────────────────────────

type TractionAssistLike = ThrottleConfig['tractionAssist'];

type ResolvedTractionAssist = {
  enabled: boolean;
  targetSlipRatio: number;
  cutGain: number;
  recoverRate: number;
  minThrottle: number;
  headroom: number;
  headroomHighSpeed: number;
  catchStartRad: number;
  catchFullRad: number;
  aggressiveHeadroomMult: number;
  aggressiveCatchShiftRad: number;
};

function resolveTractionAssist(ta?: TractionAssistLike): ResolvedTractionAssist {
  return {
    enabled: ta?.enabled ?? true,
    targetSlipRatio: clampFinite(ta?.targetSlipRatio ?? 0.11, 0.03, 0.5),
    cutGain: clampFinite(ta?.cutGain ?? 6, 0.5, 40),
    recoverRate: clampFinite(ta?.recoverRate ?? 1.2, 0.1, 10),
    minThrottle: clampFinite(ta?.minThrottle ?? 0.25, 0.05, 0.8),
    headroom: clampFinite(ta?.headroom ?? 1.2, 1.0, 2.0),
    headroomHighSpeed: clampFinite(ta?.headroomHighSpeed ?? ta?.headroom ?? 1.12, 1.0, 2.0),
    catchStartRad: clampFinite(ta?.catchStartRad ?? 0.35, 0.15, 1.0),
    catchFullRad: clampFinite(ta?.catchFullRad ?? 0.55, 0.25, 1.4),
    aggressiveHeadroomMult: clampFinite(ta?.aggressiveHeadroomMult ?? 1.4, 1.0, 2.0),
    aggressiveCatchShiftRad: clampFinite(ta?.aggressiveCatchShiftRad ?? 0.15, 0, 0.5),
  };
}

// ── constants ───────────────────────────────────────────────────

/**
 * Pedal-level simplification: brake force is a target deceleration m·g·G·pedal,
 * not line pressure. The wheel model turns it into per-wheel torque with bias;
 * lockup/ABS-lite emerge in the tire. G comes from physics.brakes.maxDecelG.
 */
const DEFAULT_BRAKE_MAX_DECEL_G = 1.05;

/** Speed range where headroom moves from `headroom` to `headroomHighSpeed`, m/s. */
const HEADROOM_TAPER_START_MS = 15;
const HEADROOM_TAPER_END_MS = 45;
/** Cap smoothing: fast down (cut), slower up (recovery feels like the tire). */
const CAP_FALL_TAU_S = 0.12;
const CAP_RISE_TAU_S = 0.3;
/** Lowpass on the steering kinematic demand - an arrow-key tap must not cut throttle instantly. */
const KIN_DEMAND_TAU_S = 0.15;
/** Lowpass on d|β|/dt (β is noisy on bumps). */
const BETA_RATE_TAU_S = 0.08;
/** Slide decay rate at which the yaw catcher fully releases throttle, rad/s. */
const BETA_DECAY_RELEASE_RADS = 0.8;

// Steering budget cap knobs live in configs/driver-assist.ts (STEER_ASSIST).
// Read directly (not snapshotted) so the debug panel can tune them live.

// ── transmission ────────────────────────────────────────────────

export class Drivetrain {
  private engine: Engine;
  private throttleCfg: ThrottleConfig;
  private transCfg: TransmissionConfig;
  /** The single throttle signal 0..1 used by all drivetrain physics. */
  private throttleVal = 0;
  private brakeVal = 0;
  private gear = 1;

  private shifting = false;
  private shiftTimer = 0;
  private shiftTarget = 1;
  private shiftIsDownshift = false;
  private rpmBefore = 0;
  private rpmAfter = 0;
  private blipPeakRpm = 0;
  private cachedWheelRpm = 0;
  private shiftCooldown = 0;
  private clutchCommand = 0;
  private shiftReengageTimer = 0;

  private steerAngleRad = 0;
  /**
   * Steering viscosity from traction: smoothed effect strength 0…1.
   * 0 = traction does not affect steering, 1 = driven axle at its budget.
   * Slows only turn-in rate; lock and counter-steer are untouched.
   */
  private steerSlowFrac = 0;
  private readonly handling: VehicleHandlingResolved;
  private readonly brakeMaxDecelG: number;
  private readonly flywheelInertia: number;

  /**
   * Traction assist: slip-aware throttle cap. Replaces an analog pedal for
   * keyboard play - a real driver holds 20-60% throttle out of a corner, a
   * keyboard jumps to 100% and hangs the driven axle in wheelspin.
   * Mutable so the debug panel can tune it live (applyTractionAssistConfig).
   */
  private tractionAssist: ResolvedTractionAssist;
  /** Reactive integrator (safety layer on measured slip). */
  private tractionCap = 1;
  /** Final throttle cap of the last tick (min of all layers, smoothed) - for HUD. */
  private lastAppliedCap = 1;
  /** Smoothed cap: removes min() steps - throttle cuts/returns with tire-like lag. */
  private smoothedCap = 1;
  /** Lowpassed steering kinematic demand. */
  private kinLatDemandLp = 0;
  /** Previous |β| and smoothed d|β|/dt - distinguishes growing vs decaying slides. */
  private prevBetaAbs = 0;
  private betaRateLp = 0;
  /** Per-tick cache of the lateral budget fraction (shared by two layers). */
  private latDemandFracCached = 0;

  constructor(
    engine: EngineConfig,
    throttle: ThrottleConfig,
    transmission: TransmissionConfig,
    handling?: Partial<VehicleHandlingConfig>,
    brakes?: BrakesConfig,
    private readonly chassis?: DrivetrainChassisInfo
  ) {
    this.engine = new Engine(engine);
    this.flywheelInertia = Math.max(0.02, engine.flywheelInertia ?? 0.35);
    this.throttleCfg = throttle;
    this.transCfg = transmission;
    this.handling = resolveVehicleHandling(handling);
    const g = brakes?.maxDecelG;
    this.brakeMaxDecelG =
      typeof g === 'number' && Number.isFinite(g) && g > 0 ? g : DEFAULT_BRAKE_MAX_DECEL_G;
    this.tractionAssist = resolveTractionAssist(throttle.tractionAssist);
  }

  /** Live re-tune of the traction assist (debug panel). Same clamps as the ctor. */
  public applyTractionAssistConfig(ta?: TractionAssistLike): void {
    this.tractionAssist = resolveTractionAssist(ta);
  }

  public update(dt: number, input: DrivetrainInput, fb: DrivetrainFeedback): DrivetrainOutput {
    this.integrateKeyboardPedal(dt, input.throttleMode, input.throttlePressed, input.throttleHold);

    const brUp = this.handling.brakeRampUpS;
    const brDn = this.handling.brakeRampDownS;
    if (input.brakePressed) {
      this.brakeVal = brUp <= 0 ? 1 : Math.min(1, this.brakeVal + dt / brUp);
    } else {
      this.brakeVal = brDn <= 0 ? 0 : Math.max(0, this.brakeVal - dt / brDn);
    }

    // Steering angle is physical (full lock at any speed). The tire itself dulls
    // response at speed (slip angle saturation); excess is caught by the budget cap.
    // On top: traction viscosity - full throttle slows turn-in only, so a tap is a
    // micro-correction and a long deliberate hold still reaches over-rotation.
    this.updateSteerViscosity(dt, fb);
    const steerCap = this.calcSteerBudgetCapRad(fb);
    const steerTargetRaw = input.steer * fb.maxSteeringAngle;
    const steerTarget = Math.max(-steerCap, Math.min(steerCap, steerTargetRaw));
    // Turn-in at speed is slower; return/counter-steer uses the base tau.
    const turnIn =
      Math.abs(steerTarget) > Math.abs(this.steerAngleRad) &&
      steerTarget * this.steerAngleRad >= 0;
    const steerTau = turnIn
      ? this.handling.steerAngleTauS * (1 + Math.max(0, fb.speedMs) / STEER_ASSIST.turnInTauSpeedRefMs)
      : this.handling.steerAngleTauS;
    let steerNext = stepSteeringAngle(this.steerAngleRad, steerTarget, dt, steerTau);
    if (turnIn) {
      // viscosity rate limiter, turn-in only. Progressive with angle: first degrees
      // stay quick (micro-corrections), deep turn-in under power gets slow.
      const c = THROTTLE_STEER_RATE;
      const maxLock = Math.max(0.05, fb.maxSteeringAngle);
      const prog = Math.pow(
        Math.min(1, Math.abs(this.steerAngleRad) / maxLock),
        Math.max(0.3, c.angleProgressPow)
      );
      const slow = this.steerSlowFrac * (c.baseSlowFrac + (1 - c.baseSlowFrac) * prog);
      const rate = c.maxTurnInRateRadS * Math.max(c.minRateFrac, 1 - slow);
      const maxDelta = rate * dt;
      const delta = steerNext - this.steerAngleRad;
      if (Math.abs(delta) > maxDelta) {
        steerNext = this.steerAngleRad + Math.sign(delta) * maxDelta;
      }
    }
    this.steerAngleRad = steerNext;

    // driven-axle ω is real state from the wheel model
    const wheelRpm = Number.isFinite(fb.wheelRpm) ? Math.max(0, fb.wheelRpm) : 0;
    this.cachedWheelRpm = wheelRpm;
    const aggressiveDriver = input.throttleMode === 'aggressive';
    this.updateAssistSignals(dt, fb);
    this.updateTractionAssist(dt, wheelRpm, fb, aggressiveDriver);

    if (this.shiftCooldown > 0) {
      this.shiftCooldown = Math.max(0, this.shiftCooldown - dt);
    }

    const currentGearRatio = this.transCfg.gearRatios[this.gear - 1] ?? 1;
    const currentTotalRatio = currentGearRatio * this.transCfg.finalDrive;

    if (this.shifting) {
      this.tickShift(dt);
    } else {
      const drivelineRpm = wheelRpm * currentTotalRatio;
      this.clutchCommand = this.resolveAutoClutchCommand(dt, fb.speedMs, drivelineRpm);
      // The assist cuts torque (effective throttle); clutch and shifts use pedal position.
      // Three layers: feedforward cap (no lag), yaw catcher (dead zone), reactive integrator.
      const ffCap = this.calcFeedforwardCap(fb, aggressiveDriver);
      const catchCap = this.calcYawCatchCap(fb, aggressiveDriver);
      const rawCap = Math.min(this.tractionCap, ffCap, catchCap);
      // First-order filter instead of a min() step: throttle leaves and returns
      // with lag, like through the tire, not like a script cut.
      const tau = rawCap < this.smoothedCap ? CAP_FALL_TAU_S : CAP_RISE_TAU_S;
      this.smoothedCap += (rawCap - this.smoothedCap) * (1 - Math.exp(-dt / tau));
      this.lastAppliedCap = this.tractionAssist.enabled ? this.smoothedCap : 1;
      const effectiveThrottle = Math.min(this.throttleVal, this.lastAppliedCap);
      this.engine.update(dt, effectiveThrottle, drivelineRpm, this.clutchCommand);
      if (this.transCfg.mode === 'auto') this.autoShift();
    }

    return {
      engineForce: this.calcEngineForce(fb.wheelRadius),
      brakeTotalForceN: this.calcBrakeTotalForceN(fb.mass),
      steering: this.steerAngleRad,
      engineRunning: this.engine.isRunning(),
      drivelineInertiaAtWheelsKgM2: this.calcDrivelineInertiaAtWheels(),
    };
  }

  /** Per-tick shared assist signals: smoothed d|β|/dt and lateral budget cache. */
  private updateAssistSignals(dt: number, fb: DrivetrainFeedback): void {
    const beta = Math.abs(fb.bodySlipAngleRad ?? 0);
    const betaRate = dt > 1e-4 ? (beta - this.prevBetaAbs) / dt : 0;
    this.prevBetaAbs = beta;
    this.betaRateLp += (betaRate - this.betaRateLp) * (1 - Math.exp(-dt / BETA_RATE_TAU_S));
    this.latDemandFracCached = this.calcLateralDemandFrac(dt, fb);
  }

  /**
   * Steering viscosity 0…1 from ACTUAL traction, not pedal: useFrac = drive Fx /
   * driven-axle budget (μ·N with weight and aero split). Full pedal in a tall gear
   * = light steering; 2nd gear under power = heavy. |Fx| - engine braking also
   * uses budget. Asymmetric smoothing: gets heavy fast, frees slowly (an instantly
   * fast wheel on lift = lift-off oversteer every time).
   */
  private updateSteerViscosity(dt: number, fb: DrivetrainFeedback): void {
    const c = THROTTLE_STEER_RATE;
    let target = 0;
    if (c.enabled && this.chassis && this.engine.isRunning()) {
      const v = Math.max(0, fb.speedMs);
      const down = (this.chassis.downforcePerV2N ?? 0) * v * v;
      const wFrac = clampFinite(this.chassis.drivenAxleWeightFrac ?? 0.5, 0.1, 1);
      const aFrac = clampFinite(this.chassis.drivenAxleAeroFrac ?? 0.5, 0, 1);
      const drivenAxleN = Math.max(1, fb.mass) * GRAVITY * wFrac + down * aFrac;
      const budgetN = Math.max(1, this.chassis.tireMu * drivenAxleN);
      const fxDriveN = Math.abs(this.outputTorque()) / Math.max(0.05, fb.wheelRadius);
      const useFrac = Math.min(1, fxDriveN / budgetN);
      const t = Math.max(0, (useFrac - c.startUseFrac) / Math.max(1e-3, 1 - c.startUseFrac));
      const gRaw = clampFinite(
        (v - c.minSpeedMs) / Math.max(1e-3, c.fullSpeedMs - c.minSpeedMs),
        0,
        1
      );
      const gate = gRaw * gRaw * (3 - 2 * gRaw);
      target = Math.min(1, t * t) * gate;
    }
    const tau = target > this.steerSlowFrac ? c.slowTauS : c.releaseTauS;
    this.steerSlowFrac +=
      (target - this.steerSlowFrac) * (1 - Math.exp(-dt / Math.max(1e-3, tau)));
  }

  /**
   * Steering angle cap from the tire's lateral budget at current speed.
   * Below 8 m/s or without chassis data: no cap (parking gets full lock).
   */
  private calcSteerBudgetCapRad(fb: DrivetrainFeedback): number {
    const max = Math.max(0.05, fb.maxSteeringAngle);
    const v = fb.speedMs;
    if (!this.chassis || !this.tractionAssist.enabled || v < 8) return max;
    const aLatMax =
      this.chassis.tireMu *
      (GRAVITY + ((this.chassis.downforcePerV2N ?? 0) * v * v) / Math.max(1, fb.mass));
    const kin = (aLatMax * Math.max(0.5, this.chassis.wheelBaseM)) / (v * v);
    return Math.min(max, STEER_ASSIST.kinHeadroom * kin + STEER_ASSIST.slipAllowanceRad);
  }

  /** Headroom vs speed and aggressive mode: loose at low speed, tighter at high speed. */
  private effectiveHeadroom(v: number, aggressive: boolean): number {
    const ta = this.tractionAssist;
    const t = Math.min(
      1,
      Math.max(0, (v - HEADROOM_TAPER_START_MS) / (HEADROOM_TAPER_END_MS - HEADROOM_TAPER_START_MS))
    );
    const base = ta.headroom + (ta.headroomHighSpeed - ta.headroom) * t;
    return base * (aggressive ? ta.aggressiveHeadroomMult : 1);
  }

  /**
   * Reactive integrator: cap drops proportional to slip excess, recovers at a
   * constant rate. Asymmetry damps the slip => couple => slip oscillation loop.
   */
  private updateTractionAssist(
    dt: number,
    wheelRpmMeasured: number,
    fb: DrivetrainFeedback,
    aggressive: boolean
  ): void {
    const ta = this.tractionAssist;
    if (!ta.enabled || !this.engine.isRunning()) {
      this.tractionCap = 1;
      return;
    }
    const wheelSpeedMs = wheelRpmMeasured * RPM_TO_OMEGA * Math.max(0.05, fb.wheelRadius);
    const v = Math.max(0, fb.speedMs);
    const slip = (wheelSpeedMs - v) / Math.max(v, 2);
    // Friction circle: lateral force eats the budget => target longitudinal slip drops.
    // Headroom > 1 deliberately allows going past the peak - power oversteer stays
    // available as a gradient; the safety layer catches only deep breakaway.
    const headroom = this.effectiveHeadroom(v, aggressive);
    let budgetFrac = headroom;
    if (this.chassis && v > 3) {
      const rest = 1 - this.latDemandFracCached ** 2;
      budgetFrac = Math.max(0.35, headroom * Math.sqrt(Math.max(0, rest)));
    }
    const excess = slip - ta.targetSlipRatio * budgetFrac;
    if (excess > 0) {
      this.tractionCap -= excess * ta.cutGain * dt;
    } else {
      this.tractionCap += ta.recoverRate * dt;
    }
    this.tractionCap = Math.min(1, Math.max(ta.minThrottle, this.tractionCap));
  }

  /**
   * Lateral budget fraction a_lat/a_max ∈ [0,1]. Steering kinematics (v²·δ/L)
   * go through a lowpass so a key tap does not cut throttle the same frame;
   * measured body acceleration adds what kinematics miss (slides, kerbs).
   */
  private calcLateralDemandFrac(dt: number, fb: DrivetrainFeedback): number {
    if (!this.chassis) return 0;
    const v = Math.max(0, fb.speedMs);
    const aLatMax =
      this.chassis.tireMu *
      (GRAVITY + ((this.chassis.downforcePerV2N ?? 0) * v * v) / Math.max(1, fb.mass));
    if (aLatMax <= 1e-6) return 0;
    const kinDemand = Math.min(
      aLatMax,
      (v * v * Math.abs(this.steerAngleRad)) / Math.max(0.5, this.chassis.wheelBaseM)
    );
    this.kinLatDemandLp += (kinDemand - this.kinLatDemandLp) * (1 - Math.exp(-dt / KIN_DEMAND_TAU_S));
    const measured = Math.min(aLatMax, Math.max(0, fb.latAccelMs2 ?? 0));
    return Math.max(this.kinLatDemandLp, measured) / aLatMax;
  }

  /**
   * Layer 1, feedforward cap: throttle ceiling from the driven axle's remaining
   * tire budget, without waiting for measured slip (the reactive layer misses
   * ~0.3 s snaps). fx_allowed = headroom·μ·N·√(1-(a_lat/a_max)²); headroom > 1
   * keeps progressive power oversteer available.
   */
  private calcFeedforwardCap(fb: DrivetrainFeedback, aggressive: boolean): number {
    const ta = this.tractionAssist;
    if (!ta.enabled || !this.chassis || !this.engine.isRunning() || this.shifting) return 1;
    const v = Math.max(0, fb.speedMs);
    if (v < 3) return 1; // launch: handled by the clutch and the reactive layer
    const down = (this.chassis.downforcePerV2N ?? 0) * v * v;
    const headroom = this.effectiveHeadroom(v, aggressive);
    const rest = 1 - this.latDemandFracCached ** 2;
    const budgetFrac = Math.max(0.25, Math.sqrt(Math.max(0, rest)));
    // driven axle load from static weight and aero split (load transfer ignored - conservative)
    const wFrac = clampFinite(this.chassis.drivenAxleWeightFrac ?? 0.5, 0.1, 1);
    const aFrac = clampFinite(this.chassis.drivenAxleAeroFrac ?? 0.5, 0, 1);
    const drivenAxleN = Math.max(1, fb.mass) * GRAVITY * wFrac + down * aFrac;
    const fxAllowed = headroom * this.chassis.tireMu * drivenAxleN * budgetFrac;
    const gr = this.transCfg.gearRatios[this.gear - 1] ?? 1;
    const fxFullThrottle =
      (this.engine.getTorque() * gr * this.transCfg.finalDrive) / Math.max(0.05, fb.wheelRadius);
    if (fxFullThrottle <= fxAllowed || fxFullThrottle <= 1e-6) return 1;
    return Math.max(0, fxAllowed / fxFullThrottle);
  }

  /**
   * Layer 2, yaw catcher: silent below catchStartRad - drifting is free. Then
   * cuts throttle linearly to zero at catchFullRad (angles a keyboard cannot
   * catch anymore). Applies no forces to the body. Gate on d|β|/dt: when the
   * slide decays (player caught it), throttle returns with the decay rate.
   */
  private calcYawCatchCap(fb: DrivetrainFeedback, aggressive: boolean): number {
    const ta = this.tractionAssist;
    if (!ta.enabled || fb.speedMs < 8) return 1; // β is noisy at parking speeds
    const beta = Math.abs(fb.bodySlipAngleRad ?? 0);
    const shift = aggressive ? ta.aggressiveCatchShiftRad : 0;
    const start = ta.catchStartRad + shift;
    const full = Math.max(start + 0.05, ta.catchFullRad + shift);
    if (beta <= start) return 1;
    const cap = Math.max(0, Math.min(1, 1 - (beta - start) / (full - start)));
    const decaying = Math.min(1, Math.max(0, -this.betaRateLp / BETA_DECAY_RELEASE_RADS));
    return cap + (1 - cap) * decaying;
  }

  /**
   * I_fly·(gear·final)²: locked - full (one DOF); slipping - ×engagement
   * (half-coupled approximation, keeps launch tuning); shifting - 0.
   */
  private calcDrivelineInertiaAtWheels(): number {
    if (!this.engine.isRunning() || this.shifting) return 0;
    const gr = this.transCfg.gearRatios[this.gear - 1] ?? 1;
    const total = gr * this.transCfg.finalDrive;
    const couple = this.engine.isClutchLocked() ? 1 : this.engine.getClutch();
    return this.flywheelInertia * total * total * couple;
  }

  public getState(): DrivetrainState {
    return {
      rpm: this.engine.getRpm(),
      gear: this.gear,
      pedal: this.throttleVal,
      brake: this.brakeVal,
      engineRunning: this.engine.isRunning(),
      torque: this.outputTorque(),
      tractionCap: this.lastAppliedCap,
      clutch: this.engine.getClutch(),
      clutchLocked: this.engine.isClutchLocked(),
      steerAngleRad: this.steerAngleRad,
    };
  }

  public toggleEngine() {
    this.engine.isRunning() ? this.engine.stop() : this.engine.start();
  }
  /** Idempotent start (auto-start on spawn) - never stops a running engine. */
  public startEngine() {
    if (!this.engine.isRunning()) this.engine.start();
  }
  public resetSteering() {
    this.steerAngleRad = 0;
  }

  private integrateKeyboardPedal(
    dt: number,
    throttleMode: ThrottleMode,
    pressed: boolean,
    hold: boolean
  ): void {
    const m = this.throttleCfg[throttleMode];
    if (pressed) {
      const minRamp = Math.max(0.06, this.throttleCfg.keyboardPedalMinRampUpS ?? 0.45);
      const rampUp = Math.max(minRamp, m.rampUp);
      const tipFrac = Math.max(
        0,
        Math.min(0.2, this.throttleCfg.keyboardPedalTipInFraction ?? 0)
      );
      const tipTime = this.throttleCfg.keyboardPedalTipInAttackS ?? 0;
      const useTip = tipFrac > 1e-4 && tipTime > 1e-4;
      if (!useTip) {
        this.throttleVal = Math.min(1, this.throttleVal + dt / rampUp);
        return;
      }
      if (this.throttleVal < tipFrac - 1e-5) {
        this.throttleVal = Math.min(tipFrac, this.throttleVal + dt / Math.max(0.03, tipTime));
        return;
      }
      const mainAttack = Math.max(0.06, rampUp - tipTime);
      this.throttleVal = Math.min(
        1,
        this.throttleVal + (dt * (1 - tipFrac)) / mainAttack
      );
      return;
    }
    if (!hold) {
      this.throttleVal = Math.max(0, this.throttleVal - dt / Math.max(0.04, m.rampDown));
    }
  }

  private outputTorque(): number {
    // torque is cut during a shift - otherwise it feels like a CVT
    if (!this.engine.isRunning() || this.shifting) return 0;
    const gr = this.transCfg.gearRatios[this.gear - 1] ?? 1;
    // getClutchTorqueNm is already "to transmission": + drives, - engine braking
    return this.engine.getClutchTorqueNm() * gr * this.transCfg.finalDrive;
  }

  private calcEngineForce(wheelR: number) {
    return this.outputTorque() / Math.max(0.05, wheelR);
  }

  private calcBrakeTotalForceN(mass: number): number {
    const m = Math.max(1, mass);
    return m * GRAVITY * this.brakeMaxDecelG * this.brakeVal;
  }

  private autoShift() {
    if (this.shiftCooldown > 0 || this.shifting) return;
    const rpm = this.engine.getRpm();
    const maxGear = this.transCfg.gearRatios.length;
    const upThreshold = this.resolveAutoUpshiftRpm(this.gear);
    const downThreshold = this.resolveAutoDownshiftRpm(this.gear);

    if (rpm >= upThreshold && this.gear < maxGear) {
      this.startShift(this.gear + 1);
      return;
    }
    if (rpm <= downThreshold && this.gear > 1) {
      this.startShift(this.gear - 1);
    }
  }

  private resolveAutoUpshiftRpm(gear: number): number {
    const override = this.transCfg.shiftUpRpm;
    if (Number.isFinite(override)) {
      return this.clampShiftRpm(Number(override));
    }
    const powerUpshift = this.derivePowerCurveUpshiftRpm(gear);
    const idleRpm = this.engine.getIdleRpm();
    const cruiseUpshift = this.clampShiftRpm(
      idleRpm + (powerUpshift - idleRpm) * 0.58
    );
    const aggression = this.getShiftAggression();
    return this.clampShiftRpm(
      cruiseUpshift + (powerUpshift - cruiseUpshift) * aggression
    );
  }

  private resolveAutoDownshiftRpm(gear: number): number {
    const override = this.transCfg.shiftDownRpm;
    if (Number.isFinite(override)) {
      const hyst = this.transCfg.shiftHysteresisRpm ?? 350;
      return this.clampShiftRpm(Number(override) - hyst);
    }
    const coastDownshift = this.deriveCoastDownshiftRpm(gear);
    const powerDownshift = this.derivePowerCurveDownshiftRpm(gear);
    const aggression = this.getShiftAggression();
    return this.clampShiftRpm(
      coastDownshift + (powerDownshift - coastDownshift) * aggression
    );
  }

  /** Lowest RPM where shifting up keeps ≥98.5% of current power. */
  private derivePowerCurveUpshiftRpm(gear: number): number {
    const ratios = this.transCfg.gearRatios;
    if (gear < 1 || gear >= ratios.length) {
      return this.clampShiftRpm(this.engine.getRedlineRpm());
    }

    const currentRatio = ratios[gear - 1] ?? 1;
    const nextRatio = ratios[gear] ?? currentRatio;
    const idleRpm = this.engine.getIdleRpm();
    const redlineRpm = this.engine.getRedlineRpm();
    const searchStart = this.roundShiftRpm(Math.max(idleRpm + 300, redlineRpm * 0.55));

    for (let rpm = searchStart; rpm <= redlineRpm; rpm += AUTO_SHIFT_RPM_STEP) {
      const rpmAfterShift = rpm * (nextRatio / currentRatio);
      if (rpmAfterShift < idleRpm * 0.9) {
        continue;
      }
      const currentPower = this.engine.getPowerAtRpm(rpm);
      const nextPower = this.engine.getPowerAtRpm(rpmAfterShift);
      if (nextPower >= currentPower * AUTO_UPSHIFT_POWER_MATCH) {
        return this.clampShiftRpm(rpm);
      }
    }

    return this.clampShiftRpm(redlineRpm);
  }

  /** Highest RPM where the lower gear gives ≥110% of current power. */
  private derivePowerCurveDownshiftRpm(gear: number): number {
    if (gear <= 1) {
      return 0;
    }

    const ratios = this.transCfg.gearRatios;
    const currentRatio = ratios[gear - 1] ?? 1;
    const lowerRatio = ratios[gear - 2] ?? currentRatio;
    const idleRpm = this.engine.getIdleRpm();
    const redlineRpm = this.engine.getRedlineRpm();
    const minCurrentRpm = idleRpm + Math.max(150, idleRpm * 0.15);
    const maxCurrentForSafeDownshift = redlineRpm * (currentRatio / lowerRatio);
    const searchMax = Math.min(redlineRpm * 0.92, maxCurrentForSafeDownshift);
    if (searchMax <= minCurrentRpm) {
      return this.clampShiftRpm(minCurrentRpm);
    }

    let threshold = minCurrentRpm;
    for (
      let rpm = this.roundShiftRpm(minCurrentRpm);
      rpm <= searchMax;
      rpm += AUTO_SHIFT_RPM_STEP
    ) {
      const rpmAfterDownshift = rpm * (lowerRatio / currentRatio);
      if (rpmAfterDownshift > this.engine.getMaxRpm()) {
        break;
      }
      const currentPower = this.engine.getPowerAtRpm(rpm);
      const lowerPower = this.engine.getPowerAtRpm(rpmAfterDownshift);
      if (lowerPower >= currentPower * AUTO_DOWNSHIFT_POWER_GAIN) {
        threshold = rpm;
      }
    }

    const hysteresisTrim = (this.transCfg.shiftHysteresisRpm ?? 350) * 0.35;
    return this.clampShiftRpm(threshold - hysteresisTrim);
  }

  /** Downshift RPM that lands the lower gear slightly above idle (coasting). */
  private deriveCoastDownshiftRpm(gear: number): number {
    if (gear <= 1) {
      return 0;
    }

    const ratios = this.transCfg.gearRatios;
    const currentRatio = ratios[gear - 1] ?? 1;
    const lowerRatio = ratios[gear - 2] ?? currentRatio;
    const idleRpm = this.engine.getIdleRpm();
    const targetLowerGearRpm =
      idleRpm + Math.max(220, this.engine.getLaunchSpreadRpm() * 0.35);
    const currentGearRpmAtShift =
      targetLowerGearRpm * (currentRatio / Math.max(0.01, lowerRatio));
    const hysteresisTrim = (this.transCfg.shiftHysteresisRpm ?? 350) * 0.4;
    return this.clampShiftRpm(currentGearRpmAtShift - hysteresisTrim);
  }

  /** 0…1 from pedal position; blends shift points between cruise and power. */
  private getShiftAggression(): number {
    const t = Math.max(0, Math.min(1, (this.throttleVal - 0.08) / 0.75));
    return t * t * (3 - 2 * t);
  }

  private clampShiftRpm(rpm: number): number {
    const minRpm = this.engine.getIdleRpm() + 120;
    const maxRpm = Math.min(this.engine.getMaxRpm() - 50, this.engine.getRedlineRpm());
    return Math.max(minRpm, Math.min(maxRpm, rpm));
  }

  private roundShiftRpm(rpm: number): number {
    return Math.ceil(rpm / AUTO_SHIFT_RPM_STEP) * AUTO_SHIFT_RPM_STEP;
  }

  private startShift(newGear: number) {
    if (this.shifting) return;
    const oldR = this.transCfg.gearRatios[this.gear - 1] ?? 1;
    const newR = this.transCfg.gearRatios[newGear - 1] ?? 1;
    const fd = this.transCfg.finalDrive;

    this.shifting = true;
    this.clutchCommand = 0;
    this.engine.forceDeclutch(); // instant declutch, otherwise stale lock after the shift
    this.shiftReengageTimer = 0;
    this.shiftTimer = 0;
    this.shiftTarget = newGear;
    this.shiftIsDownshift = newGear < this.gear;
    this.rpmBefore = this.engine.getRpm();
    const idle = this.engine.getIdleRpm();
    const maxR = this.engine.getMaxRpm();
    const wh = Math.max(0, this.cachedWheelRpm);
    // sync target from wheels - correct when wheelRpm is trustworthy
    const syncFromWheels = wh * newR * fd;
    // sync target from the ratio change at constant wheel speed (real sequential)
    const syncFromRatio = this.rpmBefore * (newR / oldR);
    // wheelRpm ≈ 0 or near-idle sync at high rpmBefore = telemetry glitch
    // (first frame without prev). Trusting it would setRpm(idle) then spike up.
    const wheelRpmUnreliable = wh < 12 || syncFromWheels < idle + 70;
    let rpmAfter = wheelRpmUnreliable ? syncFromRatio : syncFromWheels;
    if (!wheelRpmUnreliable && Math.abs(syncFromWheels - syncFromRatio) > 0.28 * Math.max(syncFromWheels, syncFromRatio, 1)) {
      rpmAfter = 0.55 * syncFromWheels + 0.45 * syncFromRatio;
    }
    this.rpmAfter = Math.max(idle, Math.min(maxR, rpmAfter));

    const overshoot = this.transCfg.downshiftBlipOvershoot ?? 0.04;
    if (this.shiftIsDownshift) {
      this.blipPeakRpm = Math.min(
        this.engine.getMaxRpm(),
        this.rpmAfter * (1 + overshoot)
      );
    } else {
      this.blipPeakRpm = this.rpmAfter;
    }
  }

  private tickShift(dt: number) {
    const shiftTime = Math.max(0.08, this.transCfg.shiftTimeSec ?? 0.18);
    this.shiftTimer += dt;
    const p = Math.min(1, this.shiftTimer / shiftTime);

    if (this.shiftIsDownshift) {
      this.applyDownshiftCurve(p);
    } else {
      this.applyUpshiftCurve(p);
    }

    if (p >= 1) {
      this.shifting = false;
      this.gear = this.shiftTarget;
      this.engine.setRpm(this.rpmAfter);
      this.shiftCooldown = Math.max(0, this.transCfg.shiftCooldownSec ?? 0.3);
      this.shiftReengageTimer = Math.max(0.04, this.transCfg.clutchReengageTimeSec ?? 0.09);
    }
  }

  private resolveAutoClutchCommand(dt: number, speedMs: number, drivelineRpm: number): number {
    if (!this.engine.isRunning() || !this.transCfg.autoClutch) {
      return 0;
    }
    if (this.shifting) {
      return 0;
    }

    if (this.shiftReengageTimer > 0) {
      const total = Math.max(0.04, this.transCfg.clutchReengageTimeSec ?? 0.09);
      this.shiftReengageTimer = Math.max(0, this.shiftReengageTimer - dt);
      return 1 - this.shiftReengageTimer / total;
    }

    const idle = this.engine.getIdleRpm();
    const launchSpread = this.engine.getLaunchSpreadRpm();
    if (this.throttleVal <= 0.02) {
      const coastReleaseRpm = idle * 0.9;
      const coastLockRpm = idle + Math.max(40, launchSpread * 0.2);
      if (drivelineRpm <= coastReleaseRpm) {
        return 0;
      }
      if (drivelineRpm >= coastLockRpm) {
        return 1;
      }
      const t =
        (drivelineRpm - coastReleaseRpm) /
        Math.max(1, coastLockRpm - coastReleaseRpm);
      const smooth = t * t * (3 - 2 * t);
      return Math.max(0, Math.min(1, smooth));
    }

    const launchMode = speedMs < 2.8 || drivelineRpm < idle * 0.45;
    if (!launchMode) {
      return 1;
    }

    const rpmNow = this.engine.getRpm();
    const byRpm = Math.max(0, Math.min(1, (rpmNow - idle) / Math.max(1, launchSpread)));
    const byPedal = Math.max(0.12, Math.min(1, this.throttleVal * 1.15));
    return Math.min(byRpm, byPedal);
  }

  /** Downshift: short dip => blip above sync => settle to rpmAfter. */
  private applyDownshiftCurve(p: number) {
    const idle = this.engine.getIdleRpm();
    const maxR = this.engine.getMaxRpm();
    const dipEnd = 0.22;
    const blipEnd = 0.62;

    if (p < dipEnd) {
      const u = p / dipEnd;
      const dip = this.rpmBefore * (1 - 0.12 * u);
      this.engine.setRpm(Math.max(idle, dip));
    } else if (p < blipEnd) {
      const u = (p - dipEnd) / (blipEnd - dipEnd);
      const smooth = u * u * (3 - 2 * u);
      const low = this.rpmBefore * 0.88;
      const r = low + (this.blipPeakRpm - low) * smooth;
      this.engine.setRpm(Math.min(maxR, Math.max(idle, r)));
    } else {
      const u = (p - blipEnd) / (1 - blipEnd);
      const smooth = 1 - (1 - u) * (1 - u);
      const r = this.blipPeakRpm + (this.rpmAfter - this.blipPeakRpm) * smooth;
      this.engine.setRpm(Math.min(maxR, Math.max(idle, r)));
    }
  }

  /** Upshift: RPM falls with the throttle cut => settles into the new sync. */
  private applyUpshiftCurve(p: number) {
    const idle = this.engine.getIdleRpm();
    const maxR = this.engine.getMaxRpm();
    const cutEnd = 0.48;

    if (p < cutEnd) {
      const u = p / cutEnd;
      const smooth = u * u;
      const r = this.rpmBefore + (this.rpmAfter - this.rpmBefore) * smooth * 0.92;
      this.engine.setRpm(Math.min(maxR, Math.max(idle, r)));
    } else {
      const u = (p - cutEnd) / (1 - cutEnd);
      const smooth = 1 - Math.pow(1 - u, 2);
      const mid = this.rpmBefore + (this.rpmAfter - this.rpmBefore) * 0.92;
      const r = mid + (this.rpmAfter - mid) * smooth;
      this.engine.setRpm(Math.min(maxR, Math.max(idle, r)));
    }
  }
}
