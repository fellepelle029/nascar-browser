import { Component, AfterViewInit, OnDestroy, ViewChild, ElementRef, HostListener } from '@angular/core';
import { from, Subject, EMPTY, defer, throwError } from 'rxjs';
import { catchError, mergeMap, takeUntil, tap } from 'rxjs/operators';
import { GameEngine } from '../../../engine/game-engine';
import { AssistDebug } from '../../components/assist-debug/assist-debug';
import { CarDebug } from '../../components/car-debug/car-debug';
import { CarUi } from '../../components/car-ui/car-ui';
import { ComDebug } from '../../components/com-debug/com-debug';
import { ForcesDebug } from '../../components/forces-debug/forces-debug';
import { RapierDebug } from '../../components/rapier-debug/rapier-debug';
import { VehicleTuningService } from '../../services/vehicle-tuning.service';
import { DashboardService } from '../../services/dashboard.service';
import { version } from '../../../../package.json';

type DebugSectionId = 'car' | 'assist' | 'com' | 'forces' | 'rapier';

@Component({
  selector: 'app-race',
  imports: [AssistDebug, CarDebug, CarUi, ComDebug, ForcesDebug, RapierDebug],
  providers: [DashboardService],
  templateUrl: './race.html',
  styleUrl: './race.scss',
  standalone: true,
})
export class Race implements AfterViewInit, OnDestroy {
  @ViewChild('canvas', { static: false }) canvasRef!: ElementRef<HTMLCanvasElement>;

  private gameEngine?: GameEngine;
  private destroy$ = new Subject<void>();

  /** App version from package.json for the corner badge. */
  readonly appVersion = version;
  /** Debug hub - hidden by default, F1 toggles. */
  debugVisible = false;
  /** Expanded accordion sections (multiple can stay open). */
  private readonly openSections = new Set<DebugSectionId>(['car']);

  toggleSection(id: DebugSectionId): void {
    this.openSections.has(id) ? this.openSections.delete(id) : this.openSections.add(id);
  }

  isOpen(id: DebugSectionId): boolean {
    return this.openSections.has(id);
  }

  @HostListener('window:keydown', ['$event'])
  onWindowKeyDown(e: KeyboardEvent): void {
    if (e.key === 'F1') {
      e.preventDefault(); // otherwise the browser opens its help
      this.debugVisible = !this.debugVisible;
    }
  }

  constructor(
    private tuningService: VehicleTuningService,
    public dashboard: DashboardService
  ) {}

  ngAfterViewInit(): void {
    const canvas = this.canvasRef?.nativeElement;
    
    defer(() => {
      if (!canvas) {
        return throwError(() => new Error('Canvas element not found'));
      }
      
      this.gameEngine = new GameEngine(canvas);
      this.tuningService.setGameEngine(this.gameEngine);
      // R-key respawn lives in the engine - tuning service must track the new vehicle
      this.gameEngine.setVehicleRespawnCallback((vehicle) => this.tuningService.setVehicle(vehicle));

      this.gameEngine.setDrivetrainUpdateCallback((state) => {
        this.dashboard.updateThrottle(state.pedal);
        this.dashboard.updateBrake(state.brake);
        this.dashboard.updateRpm(state.rpm);
        this.dashboard.updateGear(state.gear);
        this.dashboard.updateSpeed((this.gameEngine?.getVehicle()?.getSpeedMs() ?? 0) * 3.6);
        this.dashboard.updateSteerAngle(state.steerAngleRad);
      });
      
      return from(this.gameEngine.loadStaticTrack('/assets/models/tracks/', 'test_oval.glb')).pipe(
        mergeMap(() => from(this.gameEngine!.loadVehicleById('test'))),
        tap((vehicle) => {
          this.tuningService.setVehicle(vehicle);
          const eng = vehicle.getSpec().physics.engine;
          this.dashboard.setEngineConfig({ maxRpm: eng.maxRpm, redlineRpm: eng.redlineRpm });
        })
      );
    })
      .pipe(
        catchError((error) => {
          console.error('Error initializing GameEngine or loading vehicle model:', error);
          return EMPTY;
        }),
        takeUntil(this.destroy$)
      )
      .subscribe();
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    
    if (this.gameEngine) {
      this.gameEngine.dispose();
    }
    this.tuningService.clearVehicle();
    this.tuningService.clearGameEngine();
  }
}
