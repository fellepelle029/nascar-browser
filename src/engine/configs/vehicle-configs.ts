import { VehicleSpec } from '../types/vehicle-spec';
import { DRIVER_ASSIST } from './driver-assist';

// realistic preset (close to NASCAR Next Gen)
export const TEST_VEHICLE_SPEC: VehicleSpec = {
  id: 'test',
  name: 'NASCAR Next Gen (Realistic)',
  manufacturer: 'Chevrolet / Ford / Toyota',
  model: 'Cup Car',
  generation: 6,
  modelPath: '/assets/models/vehicles/',
  modelName: 'test2.glb',
  physics: {
    mass: 1535,
    wheelBase: 2.794,
    trackWidth: 1.90,
    centerOfMass: { x: 0, y: 0.23, z: 0.08 },
    wheelRadius: 0.355,
    maxSteeringAngle: 0.22,
    steeredAxle: 'front',
    brakes: { maxDecelG: 1.05, bias: 0.58 },
    aero: {
      // Next Gen: Cd ~0.5 (stock car), ClA ≈ 2.9 => ~8 kN of downforce at 250 km/h
      dragCoefficient: 0.52,
      liftCoefficient: -1.35,
      frontalArea: 2.18,
      // splitter/spoiler: ~40% of downforce on the front
      balanceFront: 0.4,
    },
    engine: {
      idleRpm: 800,
      maxRpm: 9200,
      redlineRpm: 9000,
      peakTorque: 655,
      peakTorqueRpm: 5800,
      // 670 hp (500 kW) @ 7800 - NASCAR Next Gen package
      torqueCurve: [
        { rpm: 800, torqueNm: 200 },
        { rpm: 1500, torqueNm: 320 },
        { rpm: 3000, torqueNm: 480 },
        { rpm: 4500, torqueNm: 590 },
        { rpm: 5800, torqueNm: 655 },
        { rpm: 7000, torqueNm: 645 },
        { rpm: 7800, torqueNm: 612 },
        { rpm: 8400, torqueNm: 560 },
        { rpm: 9000, torqueNm: 510 },
        { rpm: 9200, torqueNm: 490 },
      ],
      engineBraking: 0.25,
      flywheelInertia: 0.3,
      // clutch capacity (Coulomb cap): ~1.45× peak torque, race disc
      maxCouplingTorqueNm: 950,
      clutchLaunchSpreadRpm: 420,
    },
    transmission: {
      mode: 'auto',
      autoClutch: true,
      layout: 'rwd',
      gearRatios: [2.90, 2.10, 1.60, 1.30, 1.00],
      finalDrive: 3.55,
      shiftTimeSec: 0.22,
      shiftCooldownSec: 0.35,
      shiftHysteresisRpm: 380,
      downshiftBlipOvershoot: 0.045,
      clutchReengageTimeSec: 0.1,
    },
    throttle: {
      normal: { rampUp: 0.4, rampDown: 0.6 },
      aggressive: { rampUp: 0.1, rampDown: 0.15 },
      precise: { rampUp: 1.0, rampDown: 1.0 },
      keyboardPedalTipInFraction: 0.12,
      keyboardPedalTipInAttackS: 0.11,
      keyboardPedalMinRampUpS: 0.5,
      // traction assist tuning and off-switch: configs/driver-assist.ts
      tractionAssist: DRIVER_ASSIST,
    },
    suspension: {
      restLength: 0.12,
      springRate: 125000,
      // high-speed bump ~0.40 of critical (crit ≈ 13850 N·s/m)
      compressionDamping: 5500,
      reboundDamping: 13500,
      bumpTravel: 0.055,
      reboundTravel: 0.075,
      motionRatio: 1.0,
      // stiff front bar, soft rear (NASCAR) - balance toward understeer
      arbFrontNM: 38000,
      arbRearNM: 9000,
    },
    tires: {
      friction: 1.45,
      rollingResistance: 0.011,
      wheelMass: 28,
      // NASCAR slick: peak slip ~8.5%, peak slip angle ~5.7°, relaxation 0.22 m
      model: {
        peakSlipRatio: 0.085,
        peakSlipAngleRad: 0.1,
        shapeC: 1.35,
        loadSensitivity: 0.1,
        relaxationLengthM: 0.22,
      },
    },
    handling: {
      steerAngleTauS: 0.06,
      yawRateDamping: 0,
    },
  },
};

export const ALL_VEHICLE_CONFIGS: Record<string, VehicleSpec> = {
  'test': TEST_VEHICLE_SPEC,
};

export function getVehicleConfig(id: string): VehicleSpec | undefined {
  const base = ALL_VEHICLE_CONFIGS[id];
  return base ? structuredClone(base) : undefined;
}
