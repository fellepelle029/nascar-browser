import { Scene, AbstractMesh, TransformNode, Vector3, Quaternion, VertexBuffer } from '@babylonjs/core';
import { SceneLoader } from '@babylonjs/core/Loading/sceneLoader';
import '@babylonjs/loaders/glTF';
import { WheelKey, WHEEL_KEYS } from '../../physics/vehicle/rigs/wheel.types';

const WHEEL_NAMES: Record<WheelKey, string> = { FL: 'WheelFL', FR: 'WheelFR', RL: 'WheelRL', RR: 'WheelRR' };

export class VehicleRenderer {
  public carRoot: TransformNode;
  private wheelRoots: Record<WheelKey, TransformNode> | null = null;
  private wheelAnchors: Record<WheelKey, TransformNode> | null = null;
  private wheelMeshIds = new Set<number>();
  private boundsCache?: { size: Vector3; center: Vector3 };
  private chassisGeomCache?: { positions: number[]; indices: number[] } | null;

  constructor(private scene: Scene) {
    this.carRoot = new TransformNode('carRoot', scene);
  }

  public async loadModel(modelPath: string, modelName: string): Promise<void> {
    const result = await SceneLoader.ImportMeshAsync('', modelPath, modelName, this.scene);
    if (result.meshes[0]) result.meshes[0].parent = this.carRoot;

    this.wheelMeshIds.clear();
    this.boundsCache = undefined;
    this.chassisGeomCache = undefined;
    this.wheelRoots = null;
    this.wheelAnchors = null;

    const roots = {} as Record<WheelKey, TransformNode>;
    for (const key of WHEEL_KEYS) {
      const node = this.scene.getMeshByName(WHEEL_NAMES[key]) as TransformNode
        ?? this.scene.getTransformNodeByName(WHEEL_NAMES[key]);
      if (!node) {
        console.warn('[VehicleRenderer] Missing wheel:', WHEEL_NAMES[key]);
        return;
      }
      roots[key] = node;
      for (const m of this.getMeshesOf(node)) this.wheelMeshIds.add(m.uniqueId);
    }

    const anchors = {} as Record<WheelKey, TransformNode>;
    for (const key of WHEEL_KEYS) anchors[key] = this.createWheelAnchor(key, roots[key]);

    this.wheelRoots = roots;
    this.wheelAnchors = anchors;
  }

  public getBounds(): { size: Vector3; center: Vector3 } {
    if (this.boundsCache) return this.boundsCache;

    const geom = this.getChassisGeometry();
    if (geom) {
      this.boundsCache = aabbFromPositions(geom.positions);
    } else {
      const meshes = this.getChassisMeshes();
      this.boundsCache = meshes.length
        ? this.meshAABB(meshes)
        : { size: new Vector3(1, 1, 1), center: Vector3.Zero() };
    }
    return this.boundsCache;
  }

  public getChassisColliderMeshData(): { positions: number[]; indices: number[] } | null {
    const g = this.getChassisGeometry();
    return g ? { positions: g.positions.slice(), indices: g.indices.slice() } : null;
  }

  public syncWheelPhysics(
    poses: Record<WheelKey, { position: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } }>
  ): void {
    if (!this.wheelAnchors) return;

    this.carRoot.computeWorldMatrix(true);
    const invWorld = this.carRoot.getWorldMatrix().clone();
    invWorld.invert();
    const invRot = Quaternion.Inverse(this.carRoot.rotationQuaternion ?? Quaternion.Identity());

    for (const key of WHEEL_KEYS) {
      const anchor = this.wheelAnchors[key];
      const { position: p, rotation: r } = poses[key];
      anchor.position.copyFrom(Vector3.TransformCoordinates(new Vector3(p.x, p.y, p.z), invWorld));
      anchor.rotationQuaternion = invRot.multiply(new Quaternion(r.x, r.y, r.z, r.w));
    }
  }

  public getWheelLocalData(): Record<WheelKey, { position: Vector3; size: Vector3 }> | null {
    if (!this.wheelRoots || !this.wheelAnchors) return null;

    const data = {} as Record<WheelKey, { position: Vector3; size: Vector3 }>;
    for (const key of WHEEL_KEYS) {
      const meshes = this.getMeshesOf(this.wheelRoots[key]);
      const size = meshes.length ? this.meshAABB(meshes).size : new Vector3(0.3, 0.34, 0.34);
      data[key] = { position: this.wheelAnchors[key].position.clone(), size };
    }
    return data;
  }

  public getRoot(): TransformNode { return this.carRoot; }

  public dispose(): void { this.carRoot?.dispose(); }

  // ── internal ───────────────────────────────────────────────────

  private getMeshesOf(node: TransformNode): AbstractMesh[] {
    const m = node as AbstractMesh;
    return m.getTotalVertices !== undefined && m.getTotalVertices() > 0
      ? [m]
      : node.getChildMeshes(false);
  }

  private getChassisMeshes(): AbstractMesh[] {
    return this.carRoot.getChildMeshes(false).filter(
      m => !this.wheelMeshIds.has(m.uniqueId) && !m.name.toLowerCase().includes('wheel')
    );
  }

  // mesh AABB in carRoot local space
  private meshAABB(meshes: AbstractMesh[]): { size: Vector3; center: Vector3 } {
    this.carRoot.computeWorldMatrix(true);
    meshes.forEach(m => m.computeWorldMatrix(true));

    let minW = new Vector3(Infinity, Infinity, Infinity);
    let maxW = new Vector3(-Infinity, -Infinity, -Infinity);
    for (const m of meshes) {
      const bb = m.getBoundingInfo().boundingBox;
      minW = Vector3.Minimize(minW, bb.minimumWorld);
      maxW = Vector3.Maximize(maxW, bb.maximumWorld);
    }

    const inv = this.carRoot.getWorldMatrix().clone();
    inv.invert();
    const minL = Vector3.TransformCoordinates(minW, inv);
    const maxL = Vector3.TransformCoordinates(maxW, inv);
    const size = maxL.subtract(minL);
    return { size, center: minL.add(size.scale(0.5)) };
  }

  private createWheelAnchor(key: WheelKey, node: TransformNode): TransformNode {
    this.carRoot.computeWorldMatrix(true);
    node.computeWorldMatrix(true);

    const meshes = this.getMeshesOf(node);
    const localCenter = meshes.length
      ? this.meshAABB(meshes).center
      : Vector3.TransformCoordinates(
          node.absolutePosition,
          (() => { const m = this.carRoot.getWorldMatrix().clone(); m.invert(); return m; })()
        );

    const anchor = new TransformNode(`wheelAnchor_${key}`, this.scene);
    anchor.parent = this.carRoot;
    anchor.position.copyFrom(localCenter);
    anchor.rotationQuaternion = Quaternion.Identity();
    anchor.computeWorldMatrix(true);

    const savedWorld = node.getWorldMatrix().clone();
    node.parent = anchor;

    const anchorInv = anchor.getWorldMatrix().clone();
    anchorInv.invert();
    const localMatrix = savedWorld.multiply(anchorInv);
    const p = new Vector3(), r = new Quaternion(), s = new Vector3();
    localMatrix.decompose(s, r, p);
    node.position.copyFrom(p);
    node.rotationQuaternion = r;
    node.scaling.copyFrom(s);

    return anchor;
  }

  // chassis vertices/indices in carRoot local space (cached)
  private getChassisGeometry(): { positions: number[]; indices: number[] } | null {
    if (this.chassisGeomCache !== undefined) return this.chassisGeomCache;

    const meshes = this.getChassisMeshes();
    if (!meshes.length) { this.chassisGeomCache = null; return null; }

    this.carRoot.computeWorldMatrix(true);
    const invRoot = this.carRoot.getWorldMatrix().clone();
    invRoot.invert();

    const positions: number[] = [];
    const indices: number[] = [];
    let vertexOffset = 0;

    for (const mesh of meshes) {
      mesh.computeWorldMatrix(true);
      const verts = mesh.getVerticesData(VertexBuffer.PositionKind);
      const idx = mesh.getIndices();
      if (!verts?.length || !idx?.length) continue;

      const toLocal = mesh.getWorldMatrix().multiply(invRoot);
      const v = new Vector3();
      for (let i = 0; i < verts.length; i += 3) {
        v.set(verts[i], verts[i + 1], verts[i + 2]);
        const lp = Vector3.TransformCoordinates(v, toLocal);
        positions.push(lp.x, lp.y, lp.z);
      }
      for (const i of idx) indices.push(i + vertexOffset);
      vertexOffset += verts.length / 3;
    }

    this.chassisGeomCache = positions.length ? { positions, indices } : null;
    return this.chassisGeomCache;
  }
}

function aabbFromPositions(positions: number[]): { size: Vector3; center: Vector3 } {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  const min = new Vector3(minX, minY, minZ);
  const max = new Vector3(maxX, maxY, maxZ);
  const size = max.subtract(min);
  return { size, center: min.add(size.scale(0.5)) };
}
