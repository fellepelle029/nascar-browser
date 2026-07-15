export type ThrottleMode = 'normal' | 'aggressive' | 'precise';

export interface InputState {
  throttle: boolean;
  brake: boolean;
  steer: number;
  throttleMode: ThrottleMode;
  throttleHold: boolean;
}

export class InputSystem {
  private keys = new Set<string>();
  private engineTogglePressed = false;
  private resetPressed = false;
  private onEngineToggle?: () => void;
  private onReset?: () => void;
  private onCameraSelect?: (index: 1 | 2 | 3) => void;

  constructor() {
    this.setupListeners();
  }

  private setupListeners(): void {
    window.addEventListener('keydown', this.onKeyDown);
    window.addEventListener('keyup', this.onKeyUp);
    window.addEventListener('blur', this.onBlur);
  }

  /** Focus loss (Alt+Tab): keyup never arrives - throttle/steer would stick. */
  private onBlur = (): void => {
    this.keys.clear();
    this.engineTogglePressed = false;
    this.resetPressed = false;
  };

  private onKeyDown = (e: KeyboardEvent): void => {
    const key = e.key.toLowerCase();

    // block page scroll
    if (key === ' ' || key === 'arrowup' || key === 'arrowdown' || key === 'arrowleft' || key === 'arrowright') {
      e.preventDefault();
    }

    this.keys.add(key);

    if (key === ' ') this.keys.add('space');

    if (key === 'i' && !this.engineTogglePressed) {
      this.engineTogglePressed = true;
      this.onEngineToggle?.();
    }

    if (key === 'r' && !this.resetPressed) {
      this.resetPressed = true;
      this.onReset?.();
    }

    if (key === '1' || key === '2' || key === '3') {
      this.onCameraSelect?.(Number(key) as 1 | 2 | 3);
    }
  };

  private onKeyUp = (e: KeyboardEvent): void => {
    const key = e.key.toLowerCase();
    this.keys.delete(key);

    if (key === ' ') this.keys.delete('space');
    if (key === 'i') this.engineTogglePressed = false;
    if (key === 'r') this.resetPressed = false;
  };

  public getState(): InputState {
    const throttle = this.keys.has('w') || this.keys.has('arrowup');
    const brake = this.keys.has('s') || this.keys.has('arrowdown');
    const left = this.keys.has('a') || this.keys.has('arrowleft');
    const right = this.keys.has('d') || this.keys.has('arrowright');
    const space = this.keys.has('space');
    const steer = left === right ? 0 : left ? -1 : 1;

    return {
      throttle,
      brake,
      steer,
      throttleMode: 'normal',
      throttleHold: space
    };
  }

  public setEngineToggleCallback(cb: () => void): void {
    this.onEngineToggle = cb;
  }

  public setResetCallback(cb: () => void): void {
    this.onReset = cb;
  }

  public setCameraSelectCallback(cb: (index: 1 | 2 | 3) => void): void {
    this.onCameraSelect = cb;
  }

  public dispose(): void {
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    window.removeEventListener('blur', this.onBlur);
  }
}
