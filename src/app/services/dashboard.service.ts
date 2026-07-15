import { Injectable, signal } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class DashboardService {
  /** Mirror of the drivetrain pedal 0…1 - display only, computed in Drivetrain. */
  public throttle = signal(0);
  public brake = signal(0);
  public rpm = signal(800);
  public maxRpm = signal(9000);
  public redlineRpm = signal(8500);
  public gear = signal(1);
  public speedKmh = signal(0);
  /** Smoothed steering angle, rad (+ = right). Mirrors DrivetrainState.steerAngleRad. */
  public steerAngleRad = signal(0);

  public updateThrottle(value: number): void {
    this.throttle.set(Math.max(0, Math.min(1, value)));
  }

  public updateBrake(value: number): void {
    this.brake.set(Math.max(0, Math.min(1, value)));
  }

  public updateRpm(value: number): void {
    this.rpm.set(Math.max(0, value));
  }

  public updateGear(value: number): void {
    this.gear.set(value);
  }

  public updateSpeed(kmh: number): void {
    this.speedKmh.set(Math.max(0, kmh));
  }

  public updateSteerAngle(rad: number): void {
    this.steerAngleRad.set(Number.isFinite(rad) ? rad : 0);
  }

  public setEngineConfig(config: { maxRpm: number; redlineRpm: number }): void {
    this.maxRpm.set(config.maxRpm);
    this.redlineRpm.set(config.redlineRpm);
  }
}
