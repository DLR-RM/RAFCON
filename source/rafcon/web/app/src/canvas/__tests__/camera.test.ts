import { describe, expect, it } from 'vitest'
import type { StateMachineJson } from '@/services/protocol'
import { buildRenderTree, walkRenderTree, type RenderState } from '../model'
import { fitCamera, panBy, rebase, stepChain, zoomAt, type Camera } from '../camera'
import { applyToPoint, compose } from '../transform'
import deepLibrariesFixture from '@/__fixtures__/deep_libraries.json'

const deepLibraries = deepLibrariesFixture as unknown as StateMachineJson
const VIEWPORT = { width: 1200, height: 800 }

/** screen position of a world point given as (node, local coords), robust for any anchor */
function screenPos(camera: Camera, node: RenderState, x: number, y: number): [number, number] {
  const chainDown = stepChain(node, camera.anchor)
  if (chainDown) return applyToPoint(compose(camera.local, chainDown), x, y)
  // node above the anchor: express the anchor in node space instead
  const chainUp = stepChain(camera.anchor, node)!
  const [ax, ay] = applyToPoint(chainUp, 0, 0)
  // point in anchor space = (p - anchorOrigin) / anchorScale
  return applyToPoint(camera.local, (x - ax) / chainUp.s, (y - ay) / chainUp.s)
}

function deepestNode(root: RenderState): RenderState {
  let best = root
  walkRenderTree(root, (node) => {
    if (node.depth > best.depth) best = node
  })
  return best
}

describe('camera rebase', () => {
  it('keeps fixed points stationary across rebases while zooming in (invariant)', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    const target = deepestNode(root)
    const camera = fitCamera(root, VIEWPORT)
    // marker: center of the deepest node, tracked in that node's local coords
    const marker: [number, number] = [target.width / 2, target.height / 2]

    // zoom toward the marker's screen position for many doublings
    for (let i = 0; i < 220; i++) {
      const before = screenPos(camera, target, marker[0], marker[1])
      zoomAt(camera, 1.25, before[0], before[1])
      const anchorBefore = camera.anchor.path
      rebase(camera, VIEWPORT)
      const after = screenPos(camera, target, marker[0], marker[1])
      // the zoom fixed point must not move, regardless of rebase
      expect(after[0]).toBeCloseTo(before[0], 3)
      expect(after[1]).toBeCloseTo(before[1], 3)
      // camera scale stays bounded thanks to re-rooting
      if (camera.anchor.path !== anchorBefore) {
        expect(Math.abs(camera.local.tx)).toBeLessThan(1e7)
      }
    }
    // the anchor must have migrated well below the root
    expect(camera.anchor.depth).toBeGreaterThan(0)
    // local scale is bounded even though absolute zoom is astronomic
    expect(camera.local.s).toBeLessThan(2e5)
  })

  it('pops back to the root when zooming out', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    const target = deepestNode(root)
    const camera = fitCamera(target, VIEWPORT)
    rebase(camera, VIEWPORT)
    for (let i = 0; i < 400; i++) {
      zoomAt(camera, 0.8, VIEWPORT.width / 2, VIEWPORT.height / 2)
      rebase(camera, VIEWPORT)
    }
    expect(camera.anchor.path).toBe(root.path)
  })

  it('pan keeps the anchor containing the viewport or pops', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    const camera = fitCamera(root, VIEWPORT)
    zoomAt(camera, 50, VIEWPORT.width / 2, VIEWPORT.height / 2)
    rebase(camera, VIEWPORT)
    const before = screenPos(camera, root, 10, 10)
    panBy(camera, 137, -55)
    rebase(camera, VIEWPORT)
    const after = screenPos(camera, root, 10, 10)
    expect(after[0]).toBeCloseTo(before[0] + 137, 6)
    expect(after[1]).toBeCloseTo(before[1] - 55, 6)
  })

  it('f32-unsafe absolute zoom stays representable: 150 doublings', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    const target = deepestNode(root)
    const camera = fitCamera(root, VIEWPORT)
    for (let i = 0; i < 150; i++) {
      const pos = screenPos(camera, target, target.width / 2, target.height / 2)
      zoomAt(camera, 2, pos[0], pos[1])
      rebase(camera, VIEWPORT)
    }
    // all numbers handed to the renderer stay bounded
    expect(isFinite(camera.local.s)).toBe(true)
    expect(isFinite(camera.local.tx)).toBe(true)
    expect(camera.local.s).toBeLessThanOrEqual(1.001e5)
    expect(Math.abs(camera.local.tx)).toBeLessThan(1e8)
  })
})
