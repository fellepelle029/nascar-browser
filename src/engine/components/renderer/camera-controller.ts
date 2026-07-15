import { ArcRotateCamera, Scene, UniversalCamera, Vector3 } from '@babylonjs/core';
import { Vehicle } from '../../entities/vehicle/vehicle';

export type CameraViewMode = 'free' | 'chase' | 'cockpit';

// chase camera smoothing rate
const CHASE_SMOOTH_LAMBDA = 5.2;
// how fast the LMB orbit offset returns to rear view after release
const ORBIT_RETURN_LAMBDA = 5.0;
// LMB orbit sensitivity, rad per px
const ORBIT_YAW_SENS = 0.006;
const ORBIT_PITCH_SENS = 0.004;

const CHASE_DISTANCE = 9.1;
const CHASE_BASE_PITCH = 0.13;
const CHASE_LOOK_HEIGHT = 1.0;
// total pitch limits (base + orbit offset)
const PITCH_MIN = -0.2;
const PITCH_MAX = 1.35;

export class CameraController {
  private readonly freeCamera: ArcRotateCamera;
  private readonly chaseCamera: UniversalCamera;
  private readonly cockpitCamera: UniversalCamera;
  private mode: CameraViewMode = 'free';
  private readonly canvas: HTMLCanvasElement;
  // yaw/height are smoothed instead of world positions: a lerped world target
  // lags behind at speed (~v/lambda) and the orbit axis drifts off the car
  private chaseYawSmoothed = 0;
  private chaseLookYSmoothed = 0;
  private chaseSmoothReady = false;

  // LMB orbit around the car in chase mode
  private orbiting = false;
  private orbitYaw = 0;
  private orbitPitch = 0;
  private lastPointerX = 0;
  private lastPointerY = 0;

  constructor(scene: Scene, canvas: HTMLCanvasElement) {
    this.canvas = canvas;

    this.freeCamera = new ArcRotateCamera(
      'cameraFree',
      -Math.PI / 2,
      Math.PI / 2.5,
      10,
      Vector3.Zero(),
      scene
    );
    this.freeCamera.attachControl(canvas, true);

    this.chaseCamera = new UniversalCamera('cameraChase', new Vector3(0, 4, -12), scene);
    this.chaseCamera.minZ = 0.15;
    this.chaseCamera.fov = 0.85;
    this.chaseCamera.inputs.clear();

    this.cockpitCamera = new UniversalCamera('cameraCockpit', new Vector3(0, 2, 0), scene);
    this.cockpitCamera.minZ = 0.05;
    this.cockpitCamera.fov = 1.0;
    this.cockpitCamera.inputs.clear();

    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);

    scene.activeCamera = this.freeCamera;
  }

  public getMode(): CameraViewMode {
    return this.mode;
  }

  public setMode(mode: CameraViewMode): void {
    if (mode === this.mode) {
      return;
    }
    const scene = this.freeCamera.getScene();
    this.mode = mode;
    this.orbiting = false;
    this.orbitYaw = 0;
    this.orbitPitch = 0;

    if (mode === 'free') {
      scene.activeCamera = this.freeCamera;
      this.freeCamera.attachControl(this.canvas, true);
      return;
    }

    this.freeCamera.detachControl();
    if (mode === 'chase') {
      scene.activeCamera = this.chaseCamera;
      this.chaseSmoothReady = false;
    } else {
      scene.activeCamera = this.cockpitCamera;
    }
  }

  public update(vehicle: Vehicle | undefined, dt: number): void {
    if (!vehicle) {
      return;
    }
    if (this.mode === 'chase') {
      this.updateChase(vehicle, dt);
    } else if (this.mode === 'cockpit') {
      this.updateCockpit(vehicle);
    }
  }

  // ── chase ──────────────────────────────────────────────────────

  private updateChase(vehicle: Vehicle, dt: number): void {
    const root = vehicle.getRenderer().getRoot();
    root.computeWorldMatrix(true);

    const pos = root.getAbsolutePosition();
    const forward = root.getDirection(Vector3.Forward());
    forward.normalize();

    const clampedDt = Math.min(0.1, Math.max(0, dt));

    // released LMB: orbit offset decays back to the rear view
    if (!this.orbiting) {
      const decay = Math.exp(-ORBIT_RETURN_LAMBDA * clampedDt);
      this.orbitYaw *= decay;
      this.orbitPitch *= decay;
    }

    const baseYaw = Math.atan2(forward.x, forward.z);
    const lookYIdeal = pos.y + CHASE_LOOK_HEIGHT;

    if (!this.chaseSmoothReady) {
      this.chaseYawSmoothed = baseYaw;
      this.chaseLookYSmoothed = lookYIdeal;
      this.chaseSmoothReady = true;
    } else {
      const k = 1 - Math.exp(-CHASE_SMOOTH_LAMBDA * clampedDt);
      // shortest-arc delta so the smoothed yaw never unwinds through ±π
      const dyaw = Math.atan2(
        Math.sin(baseYaw - this.chaseYawSmoothed),
        Math.cos(baseYaw - this.chaseYawSmoothed)
      );
      this.chaseYawSmoothed += dyaw * k;
      this.chaseLookYSmoothed += (lookYIdeal - this.chaseLookYSmoothed) * k;
    }

    const yaw = this.chaseYawSmoothed + this.orbitYaw;
    const pitch = Math.min(PITCH_MAX, Math.max(PITCH_MIN, CHASE_BASE_PITCH + this.orbitPitch));

    // orbit pivot rides the car: x/z are hard-locked, only height is smoothed
    const look = new Vector3(pos.x, this.chaseLookYSmoothed, pos.z);
    const horiz = Math.cos(pitch) * CHASE_DISTANCE;

    this.chaseCamera.position.set(
      look.x - Math.sin(yaw) * horiz,
      look.y + Math.sin(pitch) * CHASE_DISTANCE,
      look.z - Math.cos(yaw) * horiz
    );
    this.chaseCamera.setTarget(look);
  }

  // ── cockpit ────────────────────────────────────────────────────

  private updateCockpit(vehicle: Vehicle): void {
    const root = vehicle.getRenderer().getRoot();
    root.computeWorldMatrix(true);

    const offset = this.getCockpitLocalOffset(vehicle);
    const worldPos = Vector3.TransformCoordinates(offset, root.getWorldMatrix());
    const forward = root.getDirection(Vector3.Forward());
    const up = root.getDirection(Vector3.Up());

    this.cockpitCamera.position.copyFrom(worldPos);
    this.cockpitCamera.upVector.copyFrom(up);
    this.cockpitCamera.setTarget(worldPos.add(forward.scale(10)));
  }

  /** Camera seat point in carRoot local space: spec override or a bounds-derived default. */
  private getCockpitLocalOffset(vehicle: Vehicle): Vector3 {
    const cfg = vehicle.getSpec().visual?.cockpitCamera;
    if (cfg) {
      return new Vector3(cfg.x, cfg.y, cfg.z);
    }
    // generic fallback: eye height near the roof line, slightly ahead of the body center
    const { size, center } = vehicle.getRenderer().getBounds();
    return new Vector3(center.x, center.y + size.y * 0.27, center.z + size.z * 0.14);
  }

  // ── LMB orbit input ────────────────────────────────────────────

  private onPointerDown = (e: PointerEvent): void => {
    if (e.button !== 0 || this.mode !== 'chase') {
      return;
    }
    this.orbiting = true;
    this.lastPointerX = e.clientX;
    this.lastPointerY = e.clientY;
    this.canvas.setPointerCapture(e.pointerId);
  };

  private onPointerMove = (e: PointerEvent): void => {
    if (!this.orbiting) {
      return;
    }
    const dx = e.clientX - this.lastPointerX;
    const dy = e.clientY - this.lastPointerY;
    this.lastPointerX = e.clientX;
    this.lastPointerY = e.clientY;

    this.orbitYaw -= dx * ORBIT_YAW_SENS;
    this.orbitPitch = Math.min(
      PITCH_MAX - CHASE_BASE_PITCH,
      Math.max(PITCH_MIN - CHASE_BASE_PITCH, this.orbitPitch + dy * ORBIT_PITCH_SENS)
    );
  };

  private onPointerUp = (e: PointerEvent): void => {
    if (e.button !== 0 && e.type !== 'pointercancel') {
      return;
    }
    this.orbiting = false;
  };

  public dispose(): void {
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointermove', this.onPointerMove);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);
    this.freeCamera.detachControl();
    this.freeCamera.dispose();
    this.chaseCamera.dispose();
    this.cockpitCamera.dispose();
  }
}
