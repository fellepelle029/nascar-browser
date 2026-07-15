import { Engine, Scene, Color4 } from '@babylonjs/core';

export class EngineCore {
  public readonly engine: Engine;
  public readonly scene: Scene;
  private resizeHandler: () => void;

  constructor(canvas: HTMLCanvasElement) {
    this.engine = new Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true, antialias: true });
    this.engine.resize();

    this.scene = new Scene(this.engine);
    this.scene.clearColor = new Color4(0.2, 0.3, 0.4, 1);

    this.resizeHandler = () => this.engine.resize();
    window.addEventListener('resize', this.resizeHandler);
  }

  public startRenderLoop(onFrame: () => void): void {
    this.engine.runRenderLoop(() => { onFrame(); this.scene.render(); });
  }

  public dispose(): void {
    window.removeEventListener('resize', this.resizeHandler);
    this.engine.stopRenderLoop();
    this.scene.dispose();
    this.engine.dispose();
  }
}
