import type { VehicleHandlingConfig } from '../physics/vehicle/vehicle-handling-control';
import type { TireModelParams } from '../physics/vehicle/wheels/tire-model';
import type { TractionAssistConfig } from '../configs/driver-assist';

/** Which wheels are driven. */
export type DriveLayout = 'rwd' | 'fwd' | 'awd';
/** Which axle steers. */
export type SteeredAxle = 'front' | 'rear' | 'all';

/** Suspension in real units (N/m, N·s/m). Missing values => defaults in resolveSuspension. */
export type RawSuspensionSettings = {
  /** Suspension length at rest, m. */
  restLength: number;
  springRate?: number; // N/m at the spring; wheel rate = ·motionRatio²
  compressionDamping?: number; // N·s/m
  reboundDamping?: number; // N·s/m
  bumpTravel?: number; // m
  reboundTravel?: number; // m
  motionRatio?: number; // wheel travel / spring travel
  /** Front anti-roll bar, N/m of wheel travel difference. */
  arbFrontNM?: number;
  /** Rear anti-roll bar, N/m. */
  arbRearNM?: number;
  /** Bump stop stack stiffness relative to the wheel rate. */
  bumpStopRateMult?: number;
  /** Depth over which the bump stop keeps adding force past bumpTravel, m. */
  bumpStopRangeM?: number;
};

export interface VehicleSpec {
  id: string;
  name: string;
  manufacturer: string;
  model: string;
  generation: number;

  // model
  modelPath: string;
  modelName: string;

  // physics
  physics: {
    mass: number; // kg
    wheelBase: number; // m
    trackWidth: number; // m
    centerOfMass: { x: number; y: number; z: number };
    wheelRadius: number; // m
    maxSteeringAngle: number; // rad
    /** Which axle steers. Default 'front'. */
    steeredAxle?: SteeredAxle;
    brakes?: {
      /** Target full-pedal deceleration, g. Default 1.05. μN still limits the real value. */
      maxDecelG?: number;
      /** Front axle brake share 0.2…0.8. Default 0.58. */
      bias?: number;
    };
    aero: {
      dragCoefficient: number;  // Cd
      liftCoefficient: number;  // Cl (negative = downforce)
      frontalArea: number;      // m²
      airDensity?: number;      // kg/m³, default 1.225
      /** Front axle downforce share 0…1. Default 0.4 (stock car: splitter weaker than spoiler). */
      balanceFront?: number;
    };
    engine: {
      idleRpm: number;
      maxRpm: number;
      redlineRpm: number;
      peakTorque?: number;
      peakTorqueRpm?: number;
      torqueCurve?: Array<{
        rpm: number;
        torqueNm: number;
      }>;
      engineBraking: number;
      flywheelInertia?: number;
      /** Clutch capacity, N·m (Coulomb cap on transferred torque). Default 1.4·peakTorque. */
      maxCouplingTorqueNm?: number;
      clutchLaunchSpreadRpm?: number;
    };
    transmission: {
      mode: 'auto' | 'manual';
      autoClutch: boolean;
      /** Which wheels are driven. Default 'rwd'. */
      layout?: DriveLayout;
      gearRatios: number[];
      finalDrive: number;
      shiftUpRpm?: number;
      shiftDownRpm?: number;
      shiftTimeSec?: number;
      shiftCooldownSec?: number;
      shiftHysteresisRpm?: number;
      downshiftBlipOvershoot?: number;
      clutchReengageTimeSec?: number;
    };
    throttle: {
      normal: { rampUp: number; rampDown: number };
      aggressive: { rampUp: number; rampDown: number };
      precise: { rampUp: number; rampDown: number };
      /**
       * Keyboard tip-in: pedal jumps to this fraction over keyboardPedalTipInAttackS,
       * then grows to 1 over (rampUp - tipInAttack).
       */
      keyboardPedalTipInFraction?: number;
      /** s, time 0 => tip-in fraction while W is held */
      keyboardPedalTipInAttackS?: number;
      /** s, minimum keyboard pedal 0 => 1 time */
      keyboardPedalMinRampUpS?: number;
      /**
       * Keyboard traction assist: slip-aware throttle cap.
       * Knobs and docs: configs/driver-assist.ts.
       */
      tractionAssist?: TractionAssistConfig;
    };
    suspension: RawSuspensionSettings;
    tires: {
      friction: number;
      rollingResistance: number;
      wheelMass: number; // kg
      /** Tire model params. Missing values => NASCAR slick defaults. */
      model?: Partial<TireModelParams>;
    };
    handling?: Partial<VehicleHandlingConfig>;
  };

  // visuals (optional)
  visual?: {
    wheelSpinSpeed: number;
    exhaustParticles?: boolean;
    damageZones?: string[];
    /** Cockpit camera eye point in carRoot local space, m. Missing => derived from model bounds. */
    cockpitCamera?: { x: number; y: number; z: number };
  };
}
