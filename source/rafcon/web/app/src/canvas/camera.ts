// Deep-zoom camera with anchor re-rooting.
//
// The camera never stores one absolute zoom factor. Its pose is
//   { anchor: RenderState, local: Sim }
// where `local` maps the anchor state's local space onto the screen. While zooming
// in, the anchor migrates down the tree (push) so `local` stays bounded; while
// zooming out it migrates up (pop). All numbers that reach the GPU therefore stay
// in f32-safe ranges regardless of absolute depth — absolute zoom exists only as
// (anchor path, bounded local transform).

import type { RenderState } from './model'
import { compose, invert, scaleAround, translation, type Sim } from './transform'

export interface Camera {
  anchor: RenderState
  local: Sim
}

export interface Viewport {
  width: number
  height: number
}

/** margin factor: a child must contain the viewport this comfortably before it becomes the anchor */
const PUSH_MARGIN = 0.0

export function screenRect(camera: Sim, node: { width: number; height: number }): {
  x: number
  y: number
  w: number
  h: number
} {
  return { x: camera.tx, y: camera.ty, w: node.width * camera.s, h: node.height * camera.s }
}

function rectContainsViewport(
  rect: { x: number; y: number; w: number; h: number },
  viewport: Viewport,
  margin: number,
): boolean {
  const mx = viewport.width * margin
  const my = viewport.height * margin
  return (
    rect.x <= -mx && rect.y <= -my && rect.x + rect.w >= viewport.width + mx && rect.y + rect.h >= viewport.height + my
  )
}

/**
 * Re-root the camera: descend while a child fully contains the viewport, ascend while
 * the anchor no longer does. Returns true if the anchor changed.
 *
 * Invariant (unit-tested): the screen position of any fixed world point is unchanged —
 * each rebase composes `local` with exactly one child step (or its inverse), which is
 * a lossless algebraic re-expression up to one f64 rounding.
 */
export function rebase(camera: Camera, viewport: Viewport): boolean {
  let changed = false

  // pop: anchor must contain the viewport (unless it is the root)
  while (camera.anchor.parent) {
    const rect = screenRect(camera.local, camera.anchor)
    if (rectContainsViewport(rect, viewport, PUSH_MARGIN)) break
    camera.local = compose(camera.local, invert(camera.anchor.step))
    camera.anchor = camera.anchor.parent
    changed = true
  }

  // push: descend into the deepest child that fully contains the viewport
  let descending = true
  while (descending) {
    descending = false
    for (const child of camera.anchor.children) {
      const childLocal = compose(camera.local, child.step)
      if (rectContainsViewport(screenRect(childLocal, child), viewport, PUSH_MARGIN)) {
        camera.local = childLocal
        camera.anchor = child
        changed = true
        descending = true
        break
      }
    }
  }
  return changed
}

// With canonical units (model.ts) a rebase push multiplies local.s by ~0.1, so the
// scale only exceeds this bound where no deeper structure exists (inside a leaf or a
// gap between children). Clamping there caps every number the renderer ever sees —
// zoom depth is unlimited exactly where there is something to zoom into.
const MAX_LOCAL_SCALE = 1e5
const MIN_LOCAL_SCALE = 1e-4

/** zoom around a screen point (wheel): factor > 1 zooms in */
export function zoomAt(camera: Camera, factor: number, screenX: number, screenY: number): void {
  let clamped = factor
  if (factor > 1 && camera.local.s * factor > MAX_LOCAL_SCALE) {
    clamped = Math.max(1, MAX_LOCAL_SCALE / camera.local.s)
  } else if (factor < 1 && !camera.anchor.parent && camera.local.s * factor < MIN_LOCAL_SCALE) {
    clamped = Math.min(1, MIN_LOCAL_SCALE / camera.local.s)
  }
  camera.local = compose(scaleAround(clamped, screenX, screenY), camera.local)
}

export function panBy(camera: Camera, dx: number, dy: number): void {
  camera.local = compose(translation(dx, dy), camera.local)
}

/** the transform from `node`'s local space to `ancestor`'s local space (node must be a descendant) */
export function stepChain(node: RenderState, ancestor: RenderState): Sim | null {
  let sim: Sim = { s: 1, tx: 0, ty: 0 }
  let current: RenderState | null = node
  while (current && current !== ancestor) {
    sim = compose(current.step, sim)
    current = current.parent
  }
  return current === ancestor ? sim : null
}

/** camera pose that fits `node` into the viewport with a margin, expressed with `node` as anchor */
export function fitCamera(node: RenderState, viewport: Viewport, fill = 0.85): Camera {
  const scale = Math.min(
    (viewport.width * fill) / node.width,
    (viewport.height * fill) / node.height,
  )
  return {
    anchor: node,
    local: {
      s: scale,
      tx: (viewport.width - node.width * scale) / 2,
      ty: (viewport.height - node.height * scale) / 2,
    },
  }
}
