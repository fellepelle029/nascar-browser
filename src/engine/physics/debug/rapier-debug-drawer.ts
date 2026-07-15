import RAPIER from '@dimforge/rapier3d-compat';
import { Color3, LinesMesh, MeshBuilder, Scene, Vector3 } from '@babylonjs/core';

// debug rendering of Rapier colliders as lines
export class RapierDebugDrawer {
  private mesh?: LinesMesh;
  private enabled = false;
  private lastSegmentCount = -1;
  private readonly segmentPool: Vector3[][] = [];

  constructor(
    private readonly scene: Scene,
    private readonly getWorld: () => RAPIER.World
  ) {}

  public setEnabled(on: boolean): void {
    this.enabled = on;
    if (!on) {
      this.disposeMesh();
      this.lastSegmentCount = -1;
    }
  }

  public update(): void {
    if (!this.enabled) {
      return;
    }

    const { vertices } = this.getWorld().debugRender();
    const n = vertices.length;
    const nSeg = Math.floor(n / 6);
    if (nSeg === 0) {
      this.disposeMesh();
      this.lastSegmentCount = -1;
      return;
    }

    while (this.segmentPool.length < nSeg) {
      this.segmentPool.push([new Vector3(), new Vector3()]);
    }

    const lines = this.segmentPool.slice(0, nSeg);
    for (let s = 0; s < nSeg; s++) {
      const b = s * 6;
      lines[s][0].set(vertices[b], vertices[b + 1], vertices[b + 2]);
      lines[s][1].set(vertices[b + 3], vertices[b + 4], vertices[b + 5]);
    }

    if (!this.mesh || nSeg !== this.lastSegmentCount) {
      this.disposeMesh();
      this.mesh = MeshBuilder.CreateLineSystem(
        'rapierPhysicsDebug',
        { lines, updatable: true, useVertexAlpha: false },
        this.scene
      );
      this.mesh.isPickable = false;
      this.mesh.color = new Color3(0.15, 0.95, 0.25);
      this.mesh.renderingGroupId = 1;
      this.lastSegmentCount = nSeg;
    } else {
      MeshBuilder.CreateLineSystem(
        'rapierPhysicsDebug',
        { lines, updatable: true, useVertexAlpha: false, instance: this.mesh },
        this.scene
      );
    }
  }

  public dispose(): void {
    this.disposeMesh();
    this.lastSegmentCount = -1;
    this.enabled = false;
  }

  private disposeMesh(): void {
    if (this.mesh) {
      this.mesh.dispose();
      this.mesh = undefined;
    }
  }
}
