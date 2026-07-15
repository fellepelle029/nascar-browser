import { Component, effect } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { Hint } from '../hint/hint';

@Component({
  selector: 'app-car-debug',
  imports: [CommonModule, FormsModule, Hint],
  templateUrl: './car-debug.html',
  styleUrl: './car-debug.scss',
  standalone: true,
})
export class CarDebug {
  /** Live suspension knobs (real units: N/m, N·s/m, m). */
  public suspension = {
    springRate: 0,
    compressionDamping: 0,
    reboundDamping: 0,
    restLength: 0,
    bumpTravel: 0,
    reboundTravel: 0,
    motionRatio: 1,
    arbFrontNM: 0,
    arbRearNM: 0,
    bumpStopRateMult: 10,
    bumpStopRangeM: 0.15,
  };
  public mass = 0;
  public hasVehicle = false;

  constructor(private tuningService: VehicleTuningService) {
    effect(() => {
      const vehicle = this.tuningService.vehicle();
      if (!vehicle) {
        this.hasVehicle = false;
        return;
      }

      // effective values from the physics - what the sim actually runs,
      // including per-corner auto damper defaults (averaged for display)
      const resolved = vehicle.getPhysics()?.getSuspensionResolved();
      if (resolved) {
        this.suspension = {
          springRate: Math.round(resolved.springRate),
          compressionDamping: Math.round(resolved.compressionDamping),
          reboundDamping: Math.round(resolved.reboundDamping),
          restLength: resolved.restLength,
          bumpTravel: resolved.bumpTravel,
          reboundTravel: resolved.reboundTravel,
          motionRatio: resolved.motionRatio,
          arbFrontNM: Math.round(resolved.arbFrontNM),
          arbRearNM: Math.round(resolved.arbRearNM),
          bumpStopRateMult: Math.round(resolved.bumpStopRateMult * 10) / 10,
          bumpStopRangeM: resolved.bumpStopRangeM,
        };
      }
      this.mass = vehicle.getSpec().physics.mass;
      this.hasVehicle = true;
    });
  }

  public applySuspension(): void {
    if (!this.hasVehicle) {
      return;
    }
    this.tuningService.updateSuspension(this.suspension);
  }

  public applyMass(): void {
    if (!this.hasVehicle) {
      return;
    }
    this.tuningService.updateMass(this.mass);
  }
}
