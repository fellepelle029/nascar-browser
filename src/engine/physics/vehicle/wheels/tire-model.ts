import { clamp } from '../../../utils/math';

/**
 * Tire model - pure math, no Rapier. SI units.
 *
 * Core: combined slip through a friction circle. Slip ratio and slip angle are
 * normalized by their peak values and joined into one vector ρ; force comes from
 * a Pacejka-lite curve f(ρ) = sin(C·atan(B·ρ)) and splits back per axis. So
 * longitudinal and lateral grip compete for one μN budget automatically - trail
 * braking, throttle-steer and breakaway follow from the formula, no manual clamps.
 */

export type TireModelParams = {
  /** Slip ratio at peak longitudinal force (NASCAR slick ~0.06-0.10). */
  peakSlipRatio: number;
  /** Slip angle at peak lateral force, rad (~0.09-0.12 ≈ 5-7°). */
  peakSlipAngleRad: number;
  /**
   * Shape factor C of the Pacejka-lite curve. Peak f=1 at ρ=1, deep-slide
   * asymptote = sin(C·π/2): C=1.35 => ~0.85 (slick loses ~15% past the peak).
   */
  shapeC: number;
  /**
   * Load sensitivity: μ loss per unit of relative overload,
   * μ_eff = μ·(1 - k·(N/N₀ - 1)). Real tires ~0.1-0.2.
   */
  loadSensitivity: number;
  /** Lateral force relaxation length, m (NASCAR ~0.15-0.30). */
  relaxationLengthM: number;
  /** Slip velocity that gives full friction in the low-speed model, m/s. */
  lowSpeedFullSlipMs: number;
  /**
   * Contact speed where the LONGITUDINAL low-speed model hands over to the slip
   * model, m/s. The lateral axis blends separately and wider (FY_BLEND_* in
   * custom-vehicle-physics).
   */
  lowSpeedBlendMs: number;
};

export const DEFAULT_TIRE_MODEL: TireModelParams = {
  peakSlipRatio: 0.085,
  peakSlipAngleRad: 0.1,
  shapeC: 1.35,
  loadSensitivity: 0.1,
  relaxationLengthM: 0.22,
  lowSpeedFullSlipMs: 0.6,
  lowSpeedBlendMs: 1.2,
};

export function resolveTireModel(p?: Partial<TireModelParams>): TireModelParams {
  const d = DEFAULT_TIRE_MODEL;
  return {
    peakSlipRatio: clamp(p?.peakSlipRatio ?? d.peakSlipRatio, 0.02, 0.3),
    peakSlipAngleRad: clamp(p?.peakSlipAngleRad ?? d.peakSlipAngleRad, 0.03, 0.35),
    shapeC: clamp(p?.shapeC ?? d.shapeC, 1.05, 1.8),
    loadSensitivity: clamp(p?.loadSensitivity ?? d.loadSensitivity, 0, 0.5),
    relaxationLengthM: clamp(p?.relaxationLengthM ?? d.relaxationLengthM, 0.02, 1.0),
    lowSpeedFullSlipMs: clamp(p?.lowSpeedFullSlipMs ?? d.lowSpeedFullSlipMs, 0.1, 3),
    lowSpeedBlendMs: clamp(p?.lowSpeedBlendMs ?? d.lowSpeedBlendMs, 0.3, 5),
  };
}

/** f(ρ): 0 at zero, peak 1 at ρ=1, falls to sin(C·π/2) in deep slide. */
export function tireCurve(rhoNorm: number, shapeC: number): number {
  // B chosen so the maximum of sin(C·atan(B·ρ)) lands exactly at ρ=1
  const B = Math.tan(Math.PI / (2 * shapeC));
  return Math.sin(shapeC * Math.atan(B * rhoNorm));
}

/** μ_eff(N): sublinear - an overloaded tire grips less per unit load. */
export function effectiveMu(
  mu: number,
  normalN: number,
  staticLoadN: number,
  loadSensitivity: number
): number {
  if (staticLoadN <= 1e-6 || loadSensitivity <= 0) {
    return mu;
  }
  const overload = normalN / staticLoadN - 1;
  return mu * clamp(1 - loadSensitivity * overload, 0.6, 1.15);
}

export type TireForceResult = {
  /** Longitudinal contact force, N (+ = wheel forward). */
  fxN: number;
  /** Lateral contact force, N (wheel axes: + = right; sign already opposes slip). */
  fyN: number;
  /** Full budget μ_eff·N, N. */
  gripN: number;
  /** ρ - normalized combined slip (1 = peak). */
  rho: number;
};

/**
 * Combined slip: both axes share one μ_eff·N budget.
 * slipRatio and slipAngle are signed; force signs follow from them
 * (Fy opposes the lateral slip).
 */
export function combinedSlipForces(
  slipRatio: number,
  slipAngleRad: number,
  normalN: number,
  mu: number,
  staticLoadN: number,
  p: TireModelParams
): TireForceResult {
  if (normalN <= 0) {
    return { fxN: 0, fyN: 0, gripN: 0, rho: 0 };
  }
  const muEff = effectiveMu(mu, normalN, staticLoadN, p.loadSensitivity);
  const gripN = muEff * normalN;

  const sN = slipRatio / p.peakSlipRatio;
  const aN = slipAngleRad / p.peakSlipAngleRad;
  const rho = Math.hypot(sN, aN);
  if (rho < 1e-9) {
    return { fxN: 0, fyN: 0, gripN, rho: 0 };
  }
  const f = tireCurve(rho, p.shapeC);
  const totalN = gripN * f;
  return {
    fxN: totalN * (sN / rho),
    fyN: -totalN * (aN / rho),
    gripN,
    rho,
  };
}

/**
 * Local longitudinal tire stiffness: ∂(Fx/gripN)/∂slipRatio ≥ 0 at the current
 * combined-slip point. Needed by the semi-implicit wheel ω integration: the
 * explicit step is unstable on the linear part of the curve (λ·h > 2 with real
 * axle inertias) and saws the traction.
 *
 * Fx/gripN = f(ρ)·sN/ρ, ρ=√(sN²+aN²) =>
 * ∂/∂sN = f'(ρ)·sN²/ρ² + f(ρ)·aN²/ρ³, then divide by peakSlipRatio.
 * Past the peak f'(ρ) < 0 - return 0 (underestimating stiffness only reduces
 * damping; a negative value would break the implicit step).
 */
export function longitudinalSlipStiffness(
  slipRatio: number,
  slipAngleRad: number,
  p: TireModelParams
): number {
  const B = Math.tan(Math.PI / (2 * p.shapeC));
  const sN = slipRatio / p.peakSlipRatio;
  const aN = slipAngleRad / p.peakSlipAngleRad;
  const rho = Math.hypot(sN, aN);
  if (rho < 1e-9) {
    // f'(0) = C·B - stiffness at zero
    return (p.shapeC * B) / p.peakSlipRatio;
  }
  const fPrime = (p.shapeC * B * Math.cos(p.shapeC * Math.atan(B * rho))) / (1 + B * B * rho * rho);
  const f = Math.sin(p.shapeC * Math.atan(B * rho));
  const d = (fPrime * sN * sN) / (rho * rho) + (f * aN * aN) / (rho * rho * rho);
  return Math.max(0, d) / p.peakSlipRatio;
}
