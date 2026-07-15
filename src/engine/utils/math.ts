/** Clamp v to [min, max]. */
export function clamp(v: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, v));
}

/** Clamp v to [min, max]; non-finite input falls back to min. */
export function clampFinite(v: number, min: number, max: number): number {
  return Number.isFinite(v) ? clamp(v, min, max) : min;
}
