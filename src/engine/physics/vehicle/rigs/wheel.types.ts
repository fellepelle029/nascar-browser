export type WheelKey = 'FL' | 'FR' | 'RL' | 'RR';

/** Single source of wheel order - do not redefine locally. */
export const WHEEL_KEYS: WheelKey[] = ['FL', 'FR', 'RL', 'RR'];
export const FRONT_KEYS: WheelKey[] = ['FL', 'FR'];
export const REAR_KEYS: WheelKey[] = ['RL', 'RR'];

export type XYZ = { x: number; y: number; z: number };

export type Bounds = {
  size: { x: number; y: number; z: number };
  center: { x: number; y: number; z: number };
};

export type WheelOffsets = Record<WheelKey, { x: number; y: number; z: number }>;
export type WheelSizes = Record<WheelKey, { x: number; y: number; z: number }>;
