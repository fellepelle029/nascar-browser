export type PrePhysicsStepContext = {
  substepIndex: number;
  substepCount: number;
};

export interface IPhysicsEntity {
  prePhysicsStep?(dt: number, ctx?: PrePhysicsStepContext): void;
  /** Capture the pose after a fixed physics step (for render interpolation). */
  capturePhysicsPose?(): void;
  /**
   * Sync visuals with physics. alpha ∈ [0,1] - progress between the previous
   * and current physics tick (accumulator/step): render stays smooth on any
   * monitor rate while physics runs at fixed 60 Hz.
   */
  syncPhysics(alpha?: number): void;
}
