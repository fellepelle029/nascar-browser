import RAPIER from '@dimforge/rapier3d-compat';
import { Quaternion } from '@babylonjs/core';

type Bounds = { size: { x: number; y: number; z: number }; center: { x: number; y: number; z: number } };
type ChassisMesh = { positions: number[]; indices: number[] };

export class ChassisRig {
  private collider?: RAPIER.Collider;

  constructor(private world: RAPIER.World) {}

  public create(options: {
    rigidBody: RAPIER.RigidBody;
    bounds: Bounds;
    chassisMesh?: ChassisMesh;
    hullVertexMinLocalY?: number;
  }): void {
    const { rigidBody, bounds, chassisMesh, hullVertexMinLocalY } = options;

    let desc: RAPIER.ColliderDesc;

    if (chassisMesh?.positions.length && chassisMesh.indices.length) {
      const verts = new Float32Array(chassisMesh.positions);
      const idx = new Uint32Array(chassisMesh.indices);
      const hullVerts = hullVertexMinLocalY !== undefined
        ? filterByMinY(chassisMesh.positions, hullVertexMinLocalY)
        : verts;

      desc = (hullVerts.length >= 12 ? RAPIER.ColliderDesc.convexHull(hullVerts) : null)
        ?? RAPIER.ColliderDesc.convexHull(verts)
        ?? RAPIER.ColliderDesc.trimesh(verts, idx, RAPIER.TriMeshFlags.MERGE_DUPLICATE_VERTICES);
    } else {
      const { size, center } = bounds;
      desc = RAPIER.ColliderDesc.cuboid(size.x / 2, size.y / 2, size.z / 2)
        .setTranslation(center.x, center.y, center.z);
    }

    this.collider = this.world.createCollider(
      desc.setMass(0).setFriction(1.0).setRestitution(0.0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Max),
      rigidBody
    );
  }

  public dispose(): void {
    if (this.collider) {
      this.world.removeCollider(this.collider, true);
      this.collider = undefined;
    }
  }
}

function filterByMinY(positions: number[], minY: number): Float32Array {
  const out: number[] = [];
  for (let i = 0; i < positions.length; i += 3) {
    if (positions[i + 1] >= minY) out.push(positions[i], positions[i + 1], positions[i + 2]);
  }
  return new Float32Array(out);
}
