import RAPIER from '@dimforge/rapier3d-compat';
import { Mesh, InstancedMesh, Node, Scene, TransformNode, Vector3, VertexBuffer } from '@babylonjs/core';
import { SceneLoader } from '@babylonjs/core/Loading/sceneLoader';
import '@babylonjs/loaders/glTF';
import { TRACK_COLLIDER_FRICTION } from '../../../constants';

const TRACK_ROOT = 'trackStaticRoot';
const TRACK_Y_OFFSET = -2.5;

// ── track GLB loading ────────────────────────────────────────────

export async function loadStaticTrackGlb(scene: Scene, modelPath: string, modelName: string): Promise<void> {
  scene.getTransformNodeByName(TRACK_ROOT)?.dispose();
  const result = await SceneLoader.ImportMeshAsync('', modelPath, modelName, scene);

  const root = new TransformNode(TRACK_ROOT, scene);
  root.position.set(0, TRACK_Y_OFFSET, 0);
  root.rotationQuaternion = null;
  root.rotation.set(0, 0, 0);
  root.scaling.set(1.25, 1.25, 1.25);

  const transformNodes = (result as { transformNodes?: TransformNode[] }).transformNodes;
  const all: Node[] = [...result.meshes, ...(transformNodes ?? [])];
  const imported = new Set<Node>(all);
  for (const node of all) {
    if (!node.parent || !imported.has(node.parent)) node.parent = root;
  }

  root.computeWorldMatrix(true);
  root.freezeWorldMatrix();
}

// ── track colliders (Rapier trimesh) ─────────────────────────────

export class TrackGround {
  private body?: RAPIER.RigidBody;
  private colliders: RAPIER.Collider[] = [];

  constructor(private friction = TRACK_COLLIDER_FRICTION, private restitution = 0.0) {}

  public create(world: RAPIER.World, scene: Scene): void {
    const trackRoot = scene.getTransformNodeByName(TRACK_ROOT);
    if (!trackRoot) { console.warn('TrackGround: trackStaticRoot not found'); return; }

    const meshes = this.collectMeshes(trackRoot);
    if (!meshes.length) { console.warn('TrackGround: no triangle meshes under track root'); return; }

    this.body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    for (const mesh of meshes) this.addTrimesh(world, mesh);
  }

  public dispose(world: RAPIER.World): void {
    for (const c of this.colliders) world.removeCollider(c, true);
    this.colliders = [];
    if (this.body) world.removeRigidBody(this.body);
    this.body = undefined;
  }

  private collectMeshes(node: Node, out: Mesh[] = [], visited = new Set<number>()): Mesh[] {
    if (visited.has(node.uniqueId)) return out;
    visited.add(node.uniqueId);
    if (node instanceof Mesh && !(node instanceof InstancedMesh)) {
      const pos = node.getVerticesData(VertexBuffer.PositionKind);
      const idx = node.getIndices();
      if (pos && pos.length >= 9 && idx && idx.length >= 3) out.push(node);
    }
    for (const child of node.getChildren()) this.collectMeshes(child, out, visited);
    return out;
  }

  private addTrimesh(world: RAPIER.World, mesh: Mesh): void {
    mesh.computeWorldMatrix(true);
    const wm = mesh.getWorldMatrix();
    const positions = mesh.getVerticesData(VertexBuffer.PositionKind)!;
    const indices = mesh.getIndices()!;

    const transformed = new Float32Array(positions.length);
    const v = new Vector3();
    for (let i = 0; i < positions.length; i += 3) {
      v.set(positions[i], positions[i + 1], positions[i + 2]);
      const t = Vector3.TransformCoordinates(v, wm);
      transformed[i] = t.x; transformed[i + 1] = t.y; transformed[i + 2] = t.z;
    }

    const desc = RAPIER.ColliderDesc.trimesh(transformed, new Uint32Array(Array.from(indices)))
      .setFriction(this.friction)
      .setRestitution(this.restitution)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Max);
    this.colliders.push(world.createCollider(desc, this.body!));
  }
}
