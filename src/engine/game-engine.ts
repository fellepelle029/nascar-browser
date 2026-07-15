import { HemisphericLight, Scene, Vector3 } from '@babylonjs/core';
import { EngineCore } from './core/engine-core';
import { CameraController, CameraViewMode } from './components/renderer/camera-controller';
import { Vehicle } from './entities/vehicle/vehicle';
import { VehicleSpec } from './types/vehicle-spec';
import { PhysicsWorld } from './physics/world/physics-world';
import { IPhysicsEntity } from './types/entity.interface';
import { InputSystem } from './systems/input-system';
import { AIR_DENSITY } from './constants';
import { Drivetrain, DrivetrainState, DrivetrainFeedback, DrivetrainInput } from './physics/vehicle/systems/drivetrain';
import { RapierDebugDrawer } from './physics/debug/rapier-debug-drawer';

export class GameEngine {
  private core: EngineCore;
  private camera: CameraController;
  private physicsWorld: PhysicsWorld;
  private inputSystem: InputSystem;
  private vehicle?: Vehicle;
  private drivetrain?: Drivetrain;
  private entities: IPhysicsEntity[] = [];
  private rapierDebug?: RapierDebugDrawer;
  private rapierDebugOn = false;
  private onDrivetrainUpdate?: (state: DrivetrainState) => void;
  private onVehicleRespawned?: (vehicle: Vehicle) => void;
  private respawning = false;
  private lastTime = 0;
  private accumulator = 0;
  private readonly step = 1 / 60;
  private readonly physicsReady: Promise<void>;

  constructor(canvas: HTMLCanvasElement) {
    this.core = new EngineCore(canvas);
    this.camera = new CameraController(this.core.scene, canvas);
    new HemisphericLight('light', new Vector3(0, 1, 0), this.core.scene).intensity = 0.7;
    this.physicsWorld = new PhysicsWorld();
    this.inputSystem = new InputSystem();
    const cameraModes: Record<1 | 2 | 3, CameraViewMode> = { 1: 'free', 2: 'chase', 3: 'cockpit' };
    this.inputSystem.setCameraSelectCallback(i => this.camera.setMode(cameraModes[i]));
    this.core.startRenderLoop(() => this.tick());
    this.physicsReady = this.physicsWorld.initialize().then(() =>
      this.physicsWorld.setScene(this.core.scene)
    );
  }

  public async loadStaticTrack(modelPath: string, modelName: string): Promise<void> {
    await this.physicsReady;
    const { loadStaticTrackGlb } = await import('./physics/world/track/ground');
    await loadStaticTrackGlb(this.core.scene, modelPath, modelName);
    this.physicsWorld.createGround();
  }

  public async loadVehicle(spec: VehicleSpec): Promise<Vehicle> {
    await this.physicsReady;
    const instanceSpec = structuredClone(spec);
    if (!this.vehicle) {
      this.vehicle = new Vehicle(this.core.scene, instanceSpec, this.physicsWorld);
      await this.vehicle.load();
      this.entities.push(this.vehicle);
      const aero = instanceSpec.physics.aero;
      const layout = instanceSpec.physics.transmission.layout ?? 'rwd';
      const balanceFront = aero.balanceFront ?? 0.4;
      this.drivetrain = new Drivetrain(
        instanceSpec.physics.engine,
        instanceSpec.physics.throttle,
        instanceSpec.physics.transmission,
        instanceSpec.physics.handling,
        instanceSpec.physics.brakes,
        {
          wheelBaseM: instanceSpec.physics.wheelBase,
          tireMu: instanceSpec.physics.tires.friction,
          downforcePerV2N:
            0.5 * (aero.airDensity ?? AIR_DENSITY) * aero.frontalArea * Math.max(0, -aero.liftCoefficient),
          // driven axle weight split from real car geometry (GLB + CoM)
          drivenAxleWeightFrac: this.vehicle.getDrivenAxleStaticWeightFrac(),
          drivenAxleAeroFrac:
            layout === 'awd' ? 1 : layout === 'fwd' ? balanceFront : 1 - balanceFront,
        }
      );
      this.inputSystem.setEngineToggleCallback(() => this.drivetrain?.toggleEngine());
      this.inputSystem.setResetCallback(() => void this.respawnCurrentVehicle());
      // auto-start so W drives right away; I stays the engine toggle
      this.drivetrain.startEngine();
    }
    return this.vehicle;
  }

  public async loadVehicleById(id: string): Promise<Vehicle> {
    const { getVehicleConfig } = await import('./configs/vehicle-configs');
    const spec = getVehicleConfig(id);
    if (!spec) throw new Error(`Vehicle config not found: ${id}`);
    return this.loadVehicle(spec);
  }

  public async respawnVehicle(spec: VehicleSpec): Promise<Vehicle> {
    if (this.vehicle) {
      const old = this.vehicle;
      this.vehicle.dispose();
      this.vehicle = undefined;
      this.entities = this.entities.filter(e => e !== old);
    }
    this.drivetrain?.resetSteering();
    return this.loadVehicle(spec);
  }

  /** Respawn the current vehicle with its spec (R key). UI learns via callback. */
  public async respawnCurrentVehicle(): Promise<void> {
    if (!this.vehicle || this.respawning) return;
    this.respawning = true;
    try {
      const vehicle = await this.respawnVehicle(this.vehicle.getSpec());
      this.onVehicleRespawned?.(vehicle);
    } finally {
      this.respawning = false;
    }
  }

  public setVehicleRespawnCallback(cb: (vehicle: Vehicle) => void) { this.onVehicleRespawned = cb; }

  // ── accessors ──────────────────────────────────────────────

  public getVehicle()       { return this.vehicle; }
  public getScene(): Scene  { return this.core.scene; }
  public getEngine()        { return this.core.engine; }
  public getPhysicsWorld()  { return this.physicsWorld; }
  public getDrivetrain()    { return this.drivetrain; }
  public getCameraViewMode()            { return this.camera.getMode(); }
  public setCameraViewMode(m: CameraViewMode) { this.camera.setMode(m); }
  public getRapierPhysicsDebug()        { return this.rapierDebugOn; }

  public setRapierPhysicsDebug(on: boolean): void {
    this.rapierDebugOn = on;
    if (on && !this.rapierDebug) {
      this.rapierDebug = new RapierDebugDrawer(this.core.scene, () => this.physicsWorld.getWorld());
    }
    this.rapierDebug?.setEnabled(on);
  }

  public setDrivetrainUpdateCallback(cb: (s: DrivetrainState) => void) { this.onDrivetrainUpdate = cb; }

  // ── tick ───────────────────────────────────────────────────

  private tick(): void {
    if (!this.physicsWorld?.isReady()) return;

    const now = performance.now();
    const dt = this.lastTime ? Math.min(0.1, (now - this.lastTime) / 1000) : this.step;
    this.lastTime = now;
    this.accumulator += dt;

    let latest: DrivetrainState | undefined;

    while (this.accumulator >= this.step) {
      if (this.drivetrain && this.vehicle) {
        const inp = this.inputSystem.getState();
        const output = this.drivetrain.update(this.step, {
          throttlePressed: inp.throttle, brakePressed: inp.brake,
          steer: inp.steer, throttleMode: inp.throttleMode, throttleHold: inp.throttleHold,
        } satisfies DrivetrainInput, {
          wheelRpm: this.vehicle.getEstimatedDrivenWheelRpm(),
          speedMs: this.vehicle.getSpeedMs(),
          mass: this.vehicle.getMass(),
          wheelRadius: this.vehicle.getDrivenWheelRadiusM(),
          maxSteeringAngle: this.vehicle.getMaxSteeringAngle(),
          bodySlipAngleRad: this.vehicle.getBodySlipAngleRad(),
          latAccelMs2: this.vehicle.getLateralAccelMs2(),
        } satisfies DrivetrainFeedback);
        this.vehicle.setDriveControls(output);
        latest = this.drivetrain.getState();
      }

      const sub = Math.min(3, Math.max(1, this.vehicle?.getPhysicsSubstepCount() ?? 1));
      const subDt = this.step / sub;
      for (let s = 0; s < sub; s++) {
        const ctx = { substepIndex: s, substepCount: sub };
        for (const e of this.entities) e.prePhysicsStep?.(subDt, ctx);
        this.physicsWorld.step(subDt);
      }
      this.accumulator -= this.step;
      for (const e of this.entities) e.capturePhysicsPose?.();
    }

    if (latest) this.onDrivetrainUpdate?.(latest);
    // accumulator remainder = render position inside the physics tick;
    // interpolation removes jitter on monitors that are not 60 Hz
    const alpha = Math.min(1, Math.max(0, this.accumulator / this.step));
    for (const e of this.entities) e.syncPhysics(alpha);
    this.rapierDebug?.update();
    this.camera.update(this.vehicle, dt);
  }

  public dispose(): void {
    this.inputSystem.dispose();
    this.rapierDebug?.dispose();
    this.vehicle?.dispose();
    this.physicsWorld?.dispose();
    this.camera?.dispose();
    this.core.dispose();
  }
}
