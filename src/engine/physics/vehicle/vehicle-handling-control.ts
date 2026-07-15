import { clampFinite } from '../../utils/math';

export type VehicleHandlingConfig = {
  steerAngleTauS: number;
  yawRateDamping: number;
  brakeRampUpS: number;
  brakeRampDownS: number;
};

const DEFAULT_HANDLING: VehicleHandlingConfig = {
  steerAngleTauS: 0.055,
  yawRateDamping: 0,
  brakeRampUpS: 0.2,
  brakeRampDownS: 0.3,
};

export type VehicleHandlingResolved = Readonly<VehicleHandlingConfig>;

export function resolveVehicleHandling(
  partial?: Partial<VehicleHandlingConfig>
): VehicleHandlingResolved {
  const pick = (v: number | undefined, fallback: number) =>
    typeof v === 'number' && Number.isFinite(v) ? Math.max(0, v) : fallback;
  return {
    steerAngleTauS: pick(partial?.steerAngleTauS, DEFAULT_HANDLING.steerAngleTauS),
    yawRateDamping: pick(partial?.yawRateDamping, DEFAULT_HANDLING.yawRateDamping),
    brakeRampUpS: pick(partial?.brakeRampUpS, DEFAULT_HANDLING.brakeRampUpS),
    brakeRampDownS: pick(partial?.brakeRampDownS, DEFAULT_HANDLING.brakeRampDownS),
  };
}

/** Exponential approach of the steering angle toward the target. */
export function stepSteeringAngle(
  currentRad: number,
  targetRad: number,
  dt: number,
  tauS: number
): number {
  if (tauS <= 0) {
    return targetRad;
  }
  const alpha = 1 - Math.exp(-dt / tauS);
  return currentRad + (targetRad - currentRad) * alpha;
}
