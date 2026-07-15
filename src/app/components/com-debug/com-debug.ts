import { Component, effect } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { Hint } from '../hint/hint';

@Component({
  selector: 'app-com-debug',
  imports: [CommonModule, FormsModule, Hint],
  templateUrl: './com-debug.html',
  styleUrl: './com-debug.scss',
  standalone: true,
})
export class ComDebug {
  public show = true;
  public comX = 0;
  public comY = 0.25;
  public comZ = 0.25;
  public hasVehicle = false;
  private defaultCom = { x: 0, y: 0.25, z: 0.25 };

  constructor(private tuning: VehicleTuningService) {
    effect(() => {
      const v = this.tuning.vehicle();
      if (!v) { this.hasVehicle = false; return; }
      this.hasVehicle = true;
      const com = v.getSpec().physics.centerOfMass;
      this.comX = com.x;
      this.comY = com.y;
      this.comZ = com.z;
      this.defaultCom = { ...com };
      // the component is recreated by the accordion - sync the gizmo with the checkbox
      this.tuning.setDebugCenterMass(this.show);
    });
  }

  public onShowToggle(on: boolean): void {
    this.show = on;
    this.tuning.setDebugCenterMass(on);
  }

  public onComChange(): void {
    if (!this.hasVehicle) return;
    this.tuning.updateCenterOfMass({ x: this.comX, y: this.comY, z: this.comZ });
  }

  public resetCom(): void {
    this.comX = this.defaultCom.x;
    this.comY = this.defaultCom.y;
    this.comZ = this.defaultCom.z;
    this.onComChange();
  }
}
