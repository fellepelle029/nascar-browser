import { Component } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { Hint } from '../hint/hint';

@Component({
  selector: 'app-rapier-debug',
  imports: [CommonModule, FormsModule, Hint],
  templateUrl: './rapier-debug.html',
  styleUrl: './rapier-debug.scss',
  standalone: true,
})
export class RapierDebug {
  public wireframe = false;

  constructor(private tuningService: VehicleTuningService) {
    this.wireframe = this.tuningService.getRapierPhysicsDebug();
  }

  public onWireframeChange(enabled: boolean): void {
    this.wireframe = enabled;
    this.tuningService.setRapierPhysicsDebug(enabled);
  }
}
