// Similarity transforms (uniform scale + translation, no rotation) in f64.
// All deep-zoom camera math uses these instead of Pixi matrices so precision
// is fully under our control.

export interface Sim {
  s: number
  tx: number
  ty: number
}

export const IDENTITY: Sim = { s: 1, tx: 0, ty: 0 }

/** apply a after b: compose(a, b)(p) === a(b(p)) */
export function compose(a: Sim, b: Sim): Sim {
  return { s: a.s * b.s, tx: a.s * b.tx + a.tx, ty: a.s * b.ty + a.ty }
}

export function invert(t: Sim): Sim {
  return { s: 1 / t.s, tx: -t.tx / t.s, ty: -t.ty / t.s }
}

export function applyToPoint(t: Sim, x: number, y: number): [number, number] {
  return [t.s * x + t.tx, t.s * y + t.ty]
}

export function translation(tx: number, ty: number): Sim {
  return { s: 1, tx, ty }
}

export function scaleAround(scale: number, cx: number, cy: number): Sim {
  // fixed point (cx, cy): p -> scale * (p - c) + c
  return { s: scale, tx: cx * (1 - scale), ty: cy * (1 - scale) }
}
