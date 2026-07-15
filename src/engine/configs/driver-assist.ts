export type TractionAssistConfig = {
  /** false - assist fully off (binary pedal, physics unchanged). */
  enabled?: boolean;

  // ── Layer 3: reactive integrator ─────────────────────────────────
  /** Target slip ratio of the driven axle on a straight. Tire peak ~0.085; higher = angrier launch. */
  targetSlipRatio?: number;
  /** Cap cut rate per unit of slip excess, 1/s. Higher = catches wheelspin harder. */
  cutGain?: number;
  /** Cap recovery rate, 1/s. Higher = throttle returns faster after a cut. */
  recoverRate?: number;
  /** Cap floor 0…1 - throttle left even at full cut (so the car can launch). */
  minThrottle?: number;

  // ── Layer 1: feedforward cap ─────────────────────────────────────
  /**
   * Margin above the tire budget, 1…2. The main character knob:
   * 1.0 - on rails (throttle never exceeds budget, no power oversteer);
   * 1.2 - rear drifts progressively under power (default);
   * 1.5+ - angry, snap is close.
   */
  headroom?: number;
  /**
   * Headroom at high speed (≥ ~45 m/s), 1…2. Between 15 and 45 m/s headroom
   * moves linearly from `headroom` to this value: alive at low speed,
   * tighter safety at 250+ km/h.
   */
  headroomHighSpeed?: number;

  // ── Layer 2: yaw catcher ─────────────────────────────────────────
  /** Body β (rad) where the catcher starts cutting throttle. 0.35 ≈ 20°. Higher = freer drift. */
  catchStartRad?: number;
  /** β of full throttle cut (rad). 0.55 ≈ 31°. Past this a keyboard cannot catch the slide. */
  catchFullRad?: number;

  // ── Aggressive throttle mode ("I know what I'm doing") ───────────
  /** Headroom multiplier in aggressive mode. Default 1.4. */
  aggressiveHeadroomMult?: number;
  /** Catcher threshold shift in aggressive mode, rad. Default 0.15 (≈ +8.6°). */
  aggressiveCatchShiftRad?: number;
};

/**
 * Live preset. Tune here.
 * Loosened to work with throttle-based steering viscosity: a hard throttle on a
 * turned wheel breaks the rear loose honestly; the assist saves later.
 */
export const DRIVER_ASSIST: TractionAssistConfig = {
  enabled: true,

  // reactive safety: target above tire peak (0.085) - wheelspin lives, deep breakaway is caught
  targetSlipRatio: 0.13,
  cutGain: 5.5,
  recoverRate: 1.5,
  minThrottle: 0.25,

  // feedforward: power oversteer stays available; only full wheelspin is guarded
  headroom: 1.6,
  headroomHighSpeed: 1.35,

  // catcher: slides up to ~29° are the player's job, throttle zero at ~46°
  catchStartRad: 0.5,
  catchFullRad: 0.8,

  // aggressive mode: more freedom, spins are genuinely possible
  aggressiveHeadroomMult: 1.4,
  aggressiveCatchShiftRad: 0.15,
};

// ─────────────────────────────────────────────────────────────────
// Steering assist (drivetrain.ts): angle cap from tire budget + slower turn-in.
// ─────────────────────────────────────────────────────────────────

export type SteerAssistConfig = {
  /**
   * Margin on the kinematic part of the cap: δ_cap = k·(a_max·L/v²) + slipAllowanceRad.
   * Higher = freer steering at speed.
   */
  kinHeadroom: number;
  /**
   * Slip angle allowance up to peak Fy, rad. The main provocation knob:
   * below tire peak (0.10) - the front cannot be oversaturated, "on rails";
   * above - a sharp input can unsettle the car.
   */
  slipAllowanceRad: number;
  /** Turn-in slows with speed: tau_eff = tau·(1 + v/REF). Counter-steer stays fast. */
  turnInTauSpeedRefMs: number;
};

export const STEER_ASSIST: SteerAssistConfig = {
  kinHeadroom: 1.35,
  slipAllowanceRad: 0.11,
  turnInTauSpeedRefMs: 40,
};

// ─────────────────────────────────────────────────────────────────
// Steering viscosity from traction (drivetrain.ts): explicit friction circle
// for keyboard. Lock is NOT limited - only turn-in RATE slows, driven by actual
// traction (useFrac = drive Fx / driven-axle μ·N), not pedal position.
// Full traction = viscous wheel (tap = micro-shift, deep turn-in is slow but
// possible - deliberate over-rotation earns an honest push).
// Return to center and counter-steer never slow down.
// ─────────────────────────────────────────────────────────────────

export type ThrottleSteerRateConfig = {
  /** false - viscosity off, steering rate independent of traction. */
  enabled: boolean;
  /**
   * Driven-axle budget fraction where viscosity starts. Below it the wheel is
   * free (cruise, part throttle in a tall gear). Curve is quadratic from here.
   */
  startUseFrac: number;
  /**
   * Base slow fraction at full traction NEAR CENTER, 0…1. The main
   * "micro-shift" knob: 0.85 = a tap at full traction gives ~a quarter of lock.
   */
  baseSlowFrac: number;
  /** Viscosity progression with angle (exponent of |δ|/lock): deeper = more viscous. */
  angleProgressPow: number;
  /**
   * Turn-in rate floor at full traction and deep angle, fraction of base.
   * Deliberately NOT zero: crawling to over-rotation is possible - slow,
   * deliberate, rewarded with an honest push. 0.05 ≈ 1.3 s center to full lock.
   */
  minRateFrac: number;
  /** Base turn-in rate without traction, rad/s. */
  maxTurnInRateRadS: number;
  /** Below this speed viscosity is off (launch/pit: floor it and steer), m/s. */
  minSpeedMs: number;
  /** Speed of full effect, m/s (smoothstep between min and full). */
  fullSpeedMs: number;
  /** Viscosity build-up smoothing, s (the wheel gets heavy fast). */
  slowTauS: number;
  /**
   * Release smoothing when traction drops, s. Slower than slowTauS on purpose:
   * an instantly fast wheel on lift = lift-off oversteer every time.
   */
  releaseTauS: number;
};

export const THROTTLE_STEER_RATE: ThrottleSteerRateConfig = {
  enabled: true,
  startUseFrac: 0.35,
  // 0.9 near center + progression pow 0.6: a tap at full traction ≈ 20% of lock,
  // full lock takes ~1.2 s of continuous hold (deliberate over-rotation)
  baseSlowFrac: 0.9,
  angleProgressPow: 0.6,
  minRateFrac: 0.035,
  maxTurnInRateRadS: 3.5,
  minSpeedMs: 8,
  fullSpeedMs: 18,
  slowTauS: 0.08,
  releaseTauS: 0.25,
};

// ─────────────────────────────────────────────────────────────────
// Auto counter-steer / caster (custom-vehicle-physics.ts): emulates
// self-aligning torque - keyboard cannot feel the wheel, so the wheels
// steer themselves into the slide.
// ─────────────────────────────────────────────────────────────────

export type CasterAssistConfig = {
  /** β fraction added to steering: δ_eff = δ + gain·(β - deadzone). 0 = off. */
  gain: number;
  /** Dead zone on β, rad. Smaller = catches earlier (nanny), bigger = slides live. */
  deadzoneRad: number;
  /** Below this speed β is noisy, caster off, m/s. */
  minSpeedMs: number;
  /** Correction lowpass, s. */
  tauS: number;
};

// weakened on purpose: the push must be felt, the car does not steer itself
export const CASTER_ASSIST: CasterAssistConfig = {
  gain: 0.3,
  deadzoneRad: 0.1, // ~5.7°
  minSpeedMs: 5,
  tauS: 0.06,
};

// ─────────────────────────────────────────────────────────────────
// ABS-lite (custom-vehicle-physics.ts): a binary brake key must not lock wheels solid.
// ─────────────────────────────────────────────────────────────────

export type AbsAssistConfig = {
  /** |slip| where brake torque starts backing off (tire peak ~0.085). */
  slipStart: number;
  /** |slip| of full backoff. */
  slipFull: number;
  /** Brake torque floor 0…1 at full backoff. */
  minBrakeScale: number;
};

export const ABS_ASSIST: AbsAssistConfig = {
  slipStart: 0.12,
  slipFull: 0.3,
  minBrakeScale: 0.2,
};
