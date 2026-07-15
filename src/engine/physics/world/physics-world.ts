import RAPIER from '@dimforge/rapier3d-compat';
import { Scene } from '@babylonjs/core';
import { TRACK_COLLIDER_FRICTION } from '../../constants';
import { TrackGround } from './track/ground';

export class PhysicsWorld {
  private world?: RAPIER.World;
  private gravity: { x: number; y: number; z: number };
  private isInitialized = false;
  private ground?: TrackGround;
  private scene?: Scene;

  constructor(gravity: { x: number; y: number; z: number } = { x: 0, y: -9.81, z: 0 }) {
    this.gravity = gravity;
  }

  public async initialize(): Promise<void> {
    if (this.isInitialized) {
      return;
    }

    await RAPIER.init();
    this.world = new RAPIER.World(new RAPIER.Vector3(this.gravity.x, this.gravity.y, this.gravity.z));

    this.world.numSolverIterations = 16;
    this.world.numInternalPgsIterations = 2;
    // less penetration - no falling through the floor
    this.world.integrationParameters.normalizedAllowedLinearError = 0.0002;
    // stiffer contacts - less bounce on the surface
    this.world.integrationParameters.contact_natural_frequency = 30;
    // extra CCD substeps for sharp trajectories
    this.world.maxCcdSubsteps = 2;

    this.isInitialized = true;
  }

  public setScene(scene: Scene): void {
    this.scene = scene;
  }

  public createGround(colliderFriction = TRACK_COLLIDER_FRICTION): void {
    if (!this.world) {
      throw new Error('PhysicsWorld is not initialized. Call initialize() first.');
    }
    if (!this.scene) {
      throw new Error('Scene is not set. Call setScene() first.');
    }

    if (this.ground) {
      this.ground.dispose(this.world);
      this.ground = undefined;
    }

    this.ground = new TrackGround(colliderFriction);
    this.ground.create(this.world, this.scene);
  }

  public getWorld(): RAPIER.World {
    if (!this.world) {
      throw new Error('PhysicsWorld is not initialized. Call initialize() first.');
    }
    return this.world;
  }

  public step(dt: number = 1 / 60): void {
    if (!this.world) {
      return;
    }
    this.world.timestep = dt;
    this.world.step();
  }

  public dispose(): void {
    if (this.ground && this.world) {
      this.ground.dispose(this.world);
    }
    if (this.world) {
      this.world.free();
      this.world = undefined;
      this.isInitialized = false;
    }
  }

  public isReady(): boolean {
    return this.isInitialized && this.world !== undefined;
  }
}
