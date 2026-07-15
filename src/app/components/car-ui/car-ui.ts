import { Component, Input } from '@angular/core';
import { CommonModule } from '@angular/common';

/** Shift light LED count. Zones: 5 green, 4 yellow, 3 red. */
const LED_COUNT = 12;
/** First LED lights at this fraction of redline (top range, like iRacing). */
const LED_START_FRAC = 0.55;

@Component({
  selector: 'app-car-ui',
  imports: [CommonModule],
  templateUrl: './car-ui.html',
  styleUrl: './car-ui.scss',
  standalone: true,
})
export class CarUi {
  /** Throttle pedal 0…1 from the engine: DrivetrainState.pedal. */
  @Input() throttle = 0;
  @Input() brake = 0;
  @Input() rpm = 0;
  @Input() maxRpm = 9000;
  @Input() redlineRpm = 8500;
  @Input() gear = 1;
  @Input() speedKmh = 0;
  /** Smoothed steering angle, rad (+ = right): DrivetrainState.steerAngleRad. */
  @Input() steerAngleRad = 0;

  readonly leds = Array.from({ length: LED_COUNT }, (_, i) => i);

  get isRedline(): boolean {
    return this.rpm >= this.redlineRpm;
  }

  get gearDisplay(): string {
    if (this.gear === 0) return 'N';
    if (this.gear === -1) return 'R';
    return this.gear.toString();
  }

  get speedDisplay(): string {
    return Math.round(this.speedKmh).toString();
  }

  get rpmDisplay(): string {
    return Math.round(this.rpm).toString();
  }

  get wheelRotationDeg(): number {
    return (this.steerAngleRad * 180) / Math.PI;
  }

  ledOn(i: number): boolean {
    const start = this.redlineRpm * LED_START_FRAC;
    const step = (this.redlineRpm - start) / LED_COUNT;
    return this.rpm >= start + step * (i + 0.5);
  }

  ledZone(i: number): 'g' | 'y' | 'r' {
    if (i < 5) return 'g';
    if (i < 9) return 'y';
    return 'r';
  }
}
