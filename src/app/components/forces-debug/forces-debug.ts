import { Component, ElementRef, ViewChild, effect, OnDestroy, NgZone } from '@angular/core';
import { CommonModule } from '@angular/common';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { Hint } from '../hint/hint';
import { Vehicle } from '../../../engine/entities/vehicle/vehicle';
import type { VehicleTractionSnapshot } from '../../../engine/physics/vehicle/vehicle-physics-shared';

@Component({
  selector: 'app-forces-debug',
  imports: [CommonModule, Hint],
  templateUrl: './forces-debug.html',
  styleUrl: './forces-debug.scss',
  standalone: true,
})
export class ForcesDebug implements OnDestroy {
  @ViewChild('comCanvas', { static: false }) canvasRef?: ElementRef<HTMLCanvasElement>;

  public hasVehicle = false;
  public gravity = 0;
  public drag = 0;
  public downforce = 0;
  public centrifugal = 0;
  public centrifugalDir = 0;
  public speedKmh = 0;

  private vehicle?: Vehicle;
  private rafId = 0;
  private latG = 0;
  private lonG = 0;
  private loadLat = 0;
  private loadLon = 0;
  public traction: VehicleTractionSnapshot | null = null;

  constructor(private tuning: VehicleTuningService, private zone: NgZone) {
    effect(() => {
      const v = this.tuning.vehicle();
      if (!v) { this.hasVehicle = false; this.vehicle = undefined; return; }
      this.hasVehicle = true;
      this.vehicle = v;
      this.startLoop();
    });
  }

  ngOnDestroy(): void {
    if (this.rafId) cancelAnimationFrame(this.rafId);
  }

  private startLoop(): void {
    if (this.rafId) return;
    this.zone.runOutsideAngular(() => this.loop());
  }

  private loop = (): void => {
    this.tick();
    this.rafId = requestAnimationFrame(this.loop);
  };

  private tick(): void {
    const forces = this.vehicle?.getForces();
    if (forces) {
      this.gravity = forces.gravity;
      this.drag = forces.drag;
      this.downforce = forces.downforce;
      this.centrifugal = forces.centrifugal;
      this.centrifugalDir = forces.centrifugalDir;
      this.speedKmh = forces.speedMs * 3.6;
      this.latG = forces.lateralG;
      this.lonG = forces.longitudinalG;
      this.traction = forces.traction;
      this.updateLoadTransferDot(forces.traction);
    }

    this.drawGSquare();
  }

  private updateLoadTransferDot(traction: VehicleTractionSnapshot | null): void {
    if (!traction?.wheels?.length) {
      this.loadLat += (0 - this.loadLat) * 0.18;
      this.loadLon += (0 - this.loadLon) * 0.18;
      return;
    }

    const loadByKey = new Map(
      traction.wheels.map((wheel) => [wheel.key, Math.max(0, wheel.suspensionForceN)] as const)
    );

    const fl = loadByKey.get('FL') ?? 0;
    const fr = loadByKey.get('FR') ?? 0;
    const rl = loadByKey.get('RL') ?? 0;
    const rr = loadByKey.get('RR') ?? 0;
    const total = fl + fr + rl + rr;

    if (total < 1) {
      this.loadLat += (0 - this.loadLat) * 0.18;
      this.loadLon += (0 - this.loadLon) * 0.18;
      return;
    }

    // The dot shows wheel load transfer, not the acceleration vector.
    const rawLat = ((fr + rr) - (fl + rl)) / total;
    const rawLon = ((fl + fr) - (rl + rr)) / total;
    const alpha = 0.18;

    this.loadLat += (rawLat - this.loadLat) * alpha;
    this.loadLon += (rawLon - this.loadLon) * alpha;
  }

  private drawGSquare(): void {
    const canvas = this.canvasRef?.nativeElement;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // background
    ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
    ctx.fillRect(0, 0, w, h);

    // axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
    ctx.moveTo(0, cy); ctx.lineTo(w, cy);
    ctx.stroke();

    // axis labels
    ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('перед', cx, 10);
    ctx.fillText('зад', cx, h - 3);
    ctx.textAlign = 'left';
    ctx.fillText('Л', 3, cy - 4);
    ctx.textAlign = 'right';
    ctx.fillText('П', w - 3, cy - 4);

    // dot = load transfer: right = more load on right wheels, up = front axle
    const normX = Math.max(-1, Math.min(1, this.loadLat));
    const normY = Math.max(-1, Math.min(1, -this.loadLon));

    const dotX = cx + normX * (cx - 6);
    const dotY = cy + normY * (cy - 6);

    // center marker (rest position)
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.fill();

    // dot
    ctx.beginPath();
    ctx.arc(dotX, dotY, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#44ff44';
    ctx.fill();
    ctx.strokeStyle = '#22aa22';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // g readout
    ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
    ctx.font = '9px monospace';
    ctx.textAlign = 'right';
    ctx.fillText(`${this.latG.toFixed(2)}g lat`, w - 4, h - 14);
    ctx.fillText(`${this.lonG.toFixed(2)}g lon`, w - 4, h - 3);
  }
}
