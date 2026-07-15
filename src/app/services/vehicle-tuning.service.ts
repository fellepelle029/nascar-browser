import { Injectable, signal } from '@angular/core';
import { Vehicle } from '../../engine/entities/vehicle/vehicle';
import { GameEngine } from '../../engine/game-engine';
import { CameraViewMode } from '../../engine/components/renderer/camera-controller';
import { RawSuspensionSettings } from '../../engine/types/vehicle-spec';
import type { TractionAssistConfig } from '../../engine/configs/driver-assist';

type SuspensionSettings = RawSuspensionSettings;

@Injectable({ providedIn: 'root' })
export class VehicleTuningService {
  private readonly vehicleSignal = signal<Vehicle | null>(null);
  public readonly vehicle = this.vehicleSignal.asReadonly();
  private gameEngine?: GameEngine;

  public setVehicle(vehicle: Vehicle): void {
    this.vehicleSignal.set(vehicle);
  }

  public setGameEngine(engine: GameEngine): void {
    this.gameEngine = engine;
  }

  public clearGameEngine(): void {
    this.gameEngine = undefined;
  }

  public setRapierPhysicsDebug(enabled: boolean): void {
    this.gameEngine?.setRapierPhysicsDebug(enabled);
  }

  public getRapierPhysicsDebug(): boolean {
    return this.gameEngine?.getRapierPhysicsDebug() ?? false;
  }

  public setCameraViewMode(mode: CameraViewMode): void {
    this.gameEngine?.setCameraViewMode(mode);
  }

  public getCameraViewMode(): CameraViewMode {
    return this.gameEngine?.getCameraViewMode() ?? 'free';
  }

  public clearVehicle(): void {
    this.vehicleSignal.set(null);
  }

  /**
   * Merges over the current suspension config (does not wipe unset params).
   * Non-finite input (empty field, NaN) is dropped - clamp() in the physics
   * passes NaN through and a NaN force silently teleports the body away.
   */
  public updateSuspension(settings: Partial<SuspensionSettings>): void {
    const vehicle = this.vehicleSignal();
    if (!vehicle) {
      return;
    }

    const sanitized: Partial<SuspensionSettings> = {};
    for (const [key, value] of Object.entries(settings)) {
      if (typeof value === 'number' && Number.isFinite(value)) {
        (sanitized as Record<string, number>)[key] = value;
      }
    }

    const spec = vehicle.getSpec();
    const merged: SuspensionSettings = { ...spec.physics.suspension, ...sanitized };
    merged.restLength = this.safeNumber(merged.restLength, 0.12);
    spec.physics.suspension = merged;

    vehicle.getPhysics()?.updateSuspension(merged);
  }

  public updateMass(mass: number): void {
    const vehicle = this.vehicleSignal();
    if (!vehicle) {
      return;
    }

    const normalizedMass = this.safeNumber(mass, 1500);
    const spec = vehicle.getSpec();
    spec.physics.mass = normalizedMass;

    vehicle.getPhysics()?.updateMass(normalizedMass);
  }

  public updateCenterOfMass(com: { x: number; y: number; z: number }): void {
    const vehicle = this.vehicleSignal();
    if (!vehicle) return;
    const safe = {
      x: this.safeNumber(com.x, 0),
      y: this.safeNumber(com.y, 0.25),
      z: this.safeNumber(com.z, 0),
    };
    vehicle.getPhysics()?.updateCenterOfMass(safe);
  }

  public setDebugCenterMass(on: boolean): void {
    this.vehicleSignal()?.getPhysics()?.debugCenterMass(on);
  }

  /**
   * Live traction assist update: merged into the vehicle spec (survives respawn)
   * and applied to the running Drivetrain right away.
   */
  public updateTractionAssist(partial: Partial<TractionAssistConfig>): void {
    const vehicle = this.vehicleSignal();
    if (!vehicle) return;
    const throttle = vehicle.getSpec().physics.throttle;
    throttle.tractionAssist = { ...throttle.tractionAssist, ...partial };
    this.gameEngine?.getDrivetrain()?.applyTractionAssistConfig(throttle.tractionAssist);
  }

  /** Current traction assist config from the vehicle spec (for the debug panel). */
  public getTractionAssist(): TractionAssistConfig | undefined {
    return this.vehicleSignal()?.getSpec().physics.throttle.tractionAssist;
  }

  public async respawnVehicle(): Promise<void> {
    const vehicle = this.vehicleSignal();
    if (!vehicle || !this.gameEngine) {
      return;
    }

    const spec = vehicle.getSpec();
    const newVehicle = await this.gameEngine.respawnVehicle(spec);
    this.vehicleSignal.set(newVehicle);
  }

  private safeNumber(value: number | undefined, fallback: number): number {
    return Number.isFinite(value) ? Number(value) : fallback;
  }
}
