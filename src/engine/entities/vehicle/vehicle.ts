import { Scene, Quaternion } from '@babylonjs/core';
import { VehicleRenderer } from './vehicle-renderer';
import { IPhysicsEntity } from '../../types/entity.interface';
import { VehicleSpec } from '../../types/vehicle-spec';
import { PhysicsWorld } from '../../physics/world/physics-world';
import { CustomVehiclePhysics } from '../../physics/vehicle/custom-vehicle-physics';
import { DrivetrainOutput } from '../../physics/vehicle/systems/drivetrain';
import { WheelKey, WHEEL_KEYS, XYZ } from '../../physics/vehicle/rigs/wheel.types';

export type AnyVehiclePhysics = CustomVehiclePhysics;

/** How far below the model root the body spawns (tuned for the scene). */
const SPAWN_DROP_M = 2;

/** Spawn yaw, degrees. Positive = nose right (clockwise from above). */
const SPAWN_YAW_DEG = 90;

type Quat = { x: number; y: number; z: number; w: number };
type Pose = { pos: XYZ; rot: Quat };
type WheelPoses = Record<WheelKey, { position: XYZ; rotation: Quat }>;
type PoseSnapshot = { body: Pose; wheels: WheelPoses | null };

export class Vehicle implements IPhysicsEntity {
  private readonly renderer: VehicleRenderer;
  private physics?: AnyVehiclePhysics;
  private readonly spec: VehicleSpec;
  /** Poses of the last two physics ticks - render interpolates between them. */
  private prevPose?: PoseSnapshot;
  private currPose?: PoseSnapshot;

  constructor(scene: Scene, spec: VehicleSpec, physicsWorld?: PhysicsWorld) {
    this.spec = spec;
    this.renderer = new VehicleRenderer(scene);
    if (physicsWorld?.isReady()) {
      this.physics = new CustomVehiclePhysics(physicsWorld.getWorld(), scene, spec.physics);
    }
  }

  public async load(): Promise<void> {
    await this.renderer.loadModel(this.spec.modelPath, this.spec.modelName);
    if (!this.physics) return;

    const rootPos = this.renderer.getRoot().position;
    const bounds = this.renderer.getBounds();
    const wd = this.renderer.getWheelLocalData();
    const wheelOffsets = wd ? mapWheels(wd, (w) => toXYZ(w.position)) : undefined;
    const wheelSizes = wd ? mapWheels(wd, (w) => toXYZ(w.size)) : undefined;
    const chassisMesh = this.renderer.getChassisColliderMeshData() ?? undefined;

    // spawn clearance: full suspension length + largest wheel radius
    const s = this.spec.physics.suspension;
    const suspFullTravel = s.restLength + Math.max(s.bumpTravel ?? 0.055, s.reboundTravel ?? 0.075);
    let wheelR = Math.max(0.05, this.spec.physics.wheelRadius);
    if (wd) {
      for (const k of WHEEL_KEYS) wheelR = Math.max(wheelR, wd[k].size.y / 2);
    }
    const spawnY = rootPos.y + suspFullTravel + wheelR - SPAWN_DROP_M;

    this.physics.create(
      { x: rootPos.x, y: spawnY, z: rootPos.z },
      bounds, wheelOffsets, wheelSizes, chassisMesh,
      (SPAWN_YAW_DEG * Math.PI) / 180
    );
  }

  // ── Public API ──────────────────────────────────────────────

  public getRenderer()  { return this.renderer; }
  public getPhysics()   { return this.physics; }
  public getSpec()      { return this.spec; }
  public getSpeedMs()   { return this.physics?.getSpeedMs() ?? 0; }
  public getBodySlipAngleRad() { return this.physics?.getBodySlipAngleRad() ?? 0; }
  public getLateralAccelMs2() { return this.physics?.getLateralAccelMs2() ?? 0; }
  public getForces()    { return this.physics?.getForces(); }
  public getMass()      { return this.spec.physics.mass; }
  public getMaxSteeringAngle() { return this.spec.physics.maxSteeringAngle; }
  public getDrivenWheelRadiusM() { return this.physics?.getDrivenWheelRadiusM() ?? Math.max(0.05, this.spec.physics.wheelRadius); }
  public getEstimatedDrivenWheelRpm() { return this.physics?.getEstimatedDrivenWheelRpm() ?? 0; }
  public getDrivenAxleStaticWeightFrac() { return this.physics?.getDrivenAxleStaticWeightFrac() ?? 0.5; }
  public getPhysicsSubstepCount() { return this.physics?.getPhysicsSubstepCountForStep() ?? 1; }
  public setDriveControls(c: DrivetrainOutput) { this.physics?.setDriveControls(c); }

  // ── IPhysicsEntity ─────────────────────────────────────────

  public capturePhysicsPose(): void {
    if (!this.physics) return;
    this.prevPose = this.currPose;
    this.currPose = this.snapshotPose();
  }

  public syncPhysics(alpha = 1): void {
    if (!this.physics) return;
    const curr = this.currPose ?? this.snapshotPose();
    const prev = this.prevPose ?? curr;
    const a = Math.min(1, Math.max(0, alpha));

    const pos = lerp3(prev.body.pos, curr.body.pos, a);
    const rot = nlerpQuat(prev.body.rot, curr.body.rot, a);
    const root = this.renderer.getRoot();
    root.position.set(pos.x, pos.y, pos.z);
    root.rotationQuaternion ??= new Quaternion();
    root.rotationQuaternion.set(rot.x, rot.y, rot.z, rot.w);

    const cw = curr.wheels;
    if (cw) {
      const pw = prev.wheels;
      const wp = pw
        ? mapWheels(pw, (p, k) => ({
            position: lerp3(p.position, cw[k].position, a),
            rotation: nlerpQuat(p.rotation, cw[k].rotation, a),
          }))
        : cw;
      this.renderer.syncWheelPhysics(wp);
    }
    this.physics.syncComDebug();
  }

  private snapshotPose(): PoseSnapshot {
    return {
      body: { pos: this.physics!.getPosition(), rot: this.physics!.getRotation() },
      wheels: this.physics!.getWheelPoses(),
    };
  }

  public prePhysicsStep(dt: number): void {
    this.physics?.beforeStep(dt);
  }

  public dispose(): void {
    this.physics?.dispose();
    this.renderer.dispose();
  }
}

// ── utils ───────────────────────────────────────────────────

function toXYZ(v: XYZ): XYZ {
  return { x: v.x, y: v.y, z: v.z };
}

function mapWheels<T, R>(w: Record<WheelKey, T>, f: (x: T, k: WheelKey) => R): Record<WheelKey, R> {
  const out = {} as Record<WheelKey, R>;
  for (const k of WHEEL_KEYS) out[k] = f(w[k], k);
  return out;
}

function lerp3(a: XYZ, b: XYZ, t: number): XYZ {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t };
}

/** nlerp: angles between adjacent physics ticks are tiny - slerp not needed. */
function nlerpQuat(a: Quat, b: Quat, t: number): Quat {
  const dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  const s = dot < 0 ? -1 : 1;
  let x = a.x + (b.x * s - a.x) * t;
  let y = a.y + (b.y * s - a.y) * t;
  let z = a.z + (b.z * s - a.z) * t;
  let w = a.w + (b.w * s - a.w) * t;
  const len = Math.sqrt(x * x + y * y + z * z + w * w);
  if (len > 1e-9) {
    x /= len; y /= len; z /= len; w /= len;
  } else {
    x = b.x; y = b.y; z = b.z; w = b.w;
  }
  return { x, y, z, w };
}
