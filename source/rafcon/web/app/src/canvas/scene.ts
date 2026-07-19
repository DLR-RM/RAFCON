// SceneManager: owns the Pixi application, the render/view trees and the deep-zoom
// camera. Drives rendering on demand (dirty flag + rAF), applies LOD culling and
// re-roots the camera anchor while zooming.

import { Application, Container, Graphics, Rectangle, TilingSprite } from 'pixi.js'
import type { StateExecutionStatus, StateJson } from '@/services/protocol'
import { buildRenderTree, findByPath, type RenderState } from './model'
import { buildViews, NodeView } from './nodes'
import {
  fitCamera,
  panBy,
  rebase,
  stepChain,
  zoomAt,
  type Camera,
  type Viewport,
} from './camera'
import { applyToPoint, compose, invert, type Sim } from './transform'

// LOD thresholds in on-screen pixels (mirrors the gaphas MINIMUM_*_FOR_DISPLAY constants)
const MIN_STATE_PX = 4
const MIN_DETAIL_PX = 40
// nodes larger than this on screen get their own graphics culled — only their
// (much smaller) descendants are drawn, keeping GPU-bound numbers f32-safe
const HUGE_CULL_PX = 2e5
const MIN_HIT_PX = 8
// screen-space background dot grid
const GRID_SPACING = 36
const BACKGROUND_COLOR = 0x1a1f29
const GRID_DOT_COLOR = 0x323b4c

export interface SceneCallbacks {
  onSelect(path: string | null): void
  onHover(path: string | null): void
}

export class SceneManager {
  private app: Application | null = null
  private root: RenderState | null = null
  private rootView: NodeView | null = null
  private viewsByPath = new Map<string, NodeView>()
  private cameraLayer = new Container()
  private camera: Camera | null = null
  private flyTarget: RenderState | null = null
  private needsRender = true
  private rafHandle = 0
  private destroyed = false
  private canvasEl: HTMLCanvasElement | null = null
  private hoveredView: NodeView | null = null
  private selectedView: NodeView | null = null
  private anchorParentRestore: { view: NodeView; parent: Container; index: number } | null = null
  private dragging = false
  private dragMoved = false
  private lastPointer: [number, number] = [0, 0]
  private lastFrameTime = 0
  private grid: TilingSprite | null = null
  private activeViews = new Set<NodeView>()

  constructor(private callbacks: SceneCallbacks) {}

  async init(canvas: HTMLCanvasElement): Promise<void> {
    this.canvasEl = canvas
    const app = new Application()
    await app.init({
      canvas,
      antialias: true,
      resolution: window.devicePixelRatio || 1,
      autoDensity: true,
      background: BACKGROUND_COLOR,
      resizeTo: canvas.parentElement ?? undefined,
    })
    if (this.destroyed) {
      app.destroy()
      return
    }
    this.app = app
    app.ticker.stop()

    // subtle screen-space dot grid: fixed dot size, translates with the camera
    const dot = new Graphics()
      .rect(0, 0, GRID_SPACING, GRID_SPACING)
      .fill({ color: BACKGROUND_COLOR, alpha: 0 })
      .circle(GRID_SPACING / 2, GRID_SPACING / 2, 1.1)
      .fill({ color: GRID_DOT_COLOR, alpha: 0.55 })
    const gridTexture = app.renderer.generateTexture({
      target: dot,
      frame: new Rectangle(0, 0, GRID_SPACING, GRID_SPACING),
    })
    this.grid = new TilingSprite({ texture: gridTexture, width: 8192, height: 8192 })
    app.stage.addChild(this.grid)

    app.stage.addChild(this.cameraLayer)
    app.stage.eventMode = 'none'
    this.bindInput(canvas)
    this.lastFrameTime = performance.now()
    const loop = (time: number) => {
      if (this.destroyed) return
      this.frame(time)
      this.rafHandle = requestAnimationFrame(loop)
    }
    this.rafHandle = requestAnimationFrame(loop)
  }

  destroy(): void {
    this.destroyed = true
    cancelAnimationFrame(this.rafHandle)
    this.app?.destroy()
    this.app = null
  }

  setStateMachine(rootState: StateJson | null): void {
    this.restoreAnchorView()
    this.cameraLayer.removeChildren()
    this.viewsByPath.clear()
    this.activeViews.clear()
    this.root = null
    this.rootView = null
    this.camera = null
    this.selectedView = null
    this.hoveredView = null
    if (rootState) {
      this.root = buildRenderTree(rootState)
      const { rootView, byPath } = buildViews(this.root)
      this.rootView = rootView
      this.viewsByPath = byPath
      // the root's own step offset is irrelevant on the camera layer
      this.camera = fitCamera(this.root, this.viewportSize())
      this.mountAnchor(this.root)
    }
    this.invalidate()
  }

  setStatuses(statuses: Record<string, StateExecutionStatus>): void {
    this.activeViews.clear()
    for (const [path, view] of this.viewsByPath) {
      view.setStatus(statuses[path] ?? null)
      if (view.isActive) this.activeViews.add(view)
    }
    this.invalidate()
  }

  setSelection(path: string | null): void {
    this.selectedView?.setSelected(false)
    this.selectedView = path ? (this.viewsByPath.get(path) ?? null) : null
    this.selectedView?.setSelected(true)
    this.invalidate()
  }

  zoomToPath(path: string): void {
    if (!this.root) return
    const node = findByPath(this.root, path)
    if (node) {
      this.flyTarget = node
      this.invalidate()
    }
  }

  fitView(): void {
    if (this.root) {
      this.flyTarget = this.root
      this.invalidate()
    }
  }

  invalidate(): void {
    this.needsRender = true
  }

  private viewportSize(): Viewport {
    return {
      width: this.canvasEl?.clientWidth || 800,
      height: this.canvasEl?.clientHeight || 600,
    }
  }

  // ---------------------------------------------------------------- anchor mounting

  /** put the anchor's container onto the camera layer (undoing any previous mount) */
  private mountAnchor(anchor: RenderState): void {
    const view = this.viewsByPath.get(anchor.path)
    if (!view) return
    this.restoreAnchorView()
    if (view.container.parent && view.container.parent !== this.cameraLayer) {
      const parent = view.container.parent
      this.anchorParentRestore = {
        view,
        parent,
        index: parent.getChildIndex(view.container),
      }
      parent.removeChild(view.container)
    }
    if (view.container.parent !== this.cameraLayer) {
      this.cameraLayer.addChild(view.container)
    }
  }

  private restoreAnchorView(): void {
    const restore = this.anchorParentRestore
    if (restore) {
      this.cameraLayer.removeChild(restore.view.container)
      restore.parent.addChildAt(
        restore.view.container,
        Math.min(restore.index, restore.parent.children.length),
      )
      this.anchorParentRestore = null
    } else if (this.cameraLayer.children.length) {
      this.cameraLayer.removeChildren()
    }
  }

  // ---------------------------------------------------------------- frame loop

  private frame(time: number): void {
    const dt = Math.min((time - this.lastFrameTime) / 1000, 0.1)
    this.lastFrameTime = time
    if (!this.app || !this.camera || !this.root) return

    let animating = false
    if (this.flyTarget) {
      animating = this.stepFlight(dt)
      if (!animating) this.flyTarget = null
    }

    const pulsing = this.activeViews.size > 0
    if (!this.needsRender && !animating && !pulsing) return
    this.needsRender = false

    const viewport = this.viewportSize()
    if (rebase(this.camera, viewport)) {
      this.mountAnchor(this.camera.anchor)
    }

    // the anchor container has its own step baked in — neutralize it so the
    // container's effective transform equals camera.local
    const layerSim = compose(this.camera.local, invert(this.camera.anchor.step))
    this.cameraLayer.position.set(layerSim.tx, layerSim.ty)
    this.cameraLayer.scale.set(layerSim.s)

    if (this.grid) {
      this.grid.tilePosition.set(layerSim.tx % GRID_SPACING, layerSim.ty % GRID_SPACING)
    }

    this.applyLod(this.camera.anchor, this.camera.local, viewport)

    if (pulsing) {
      // gentle breathing glow on running states; alpha-only, no re-tessellation
      const pulse = 0.85 + 0.15 * Math.sin(time * 0.006)
      for (const view of this.activeViews) view.setRingAlpha(pulse)
    }

    this.app.render()
  }

  /** walk visible subtree, set LOD tiers, update dirty views; sims stay f64 */
  private applyLod(node: RenderState, sim: Sim, viewport: Viewport): void {
    const view = this.viewsByPath.get(node.path)
    if (!view) return
    const x = sim.tx
    const y = sim.ty
    const w = node.width * sim.s
    const h = node.height * sim.s
    if (x > viewport.width || y > viewport.height || x + w < 0 || y + h < 0) {
      view.setTier('hidden')
      return
    }
    const px = Math.max(w, h)
    if (px < MIN_STATE_PX) {
      view.setTier('dot')
      view.update()
      return
    }
    if (px > HUGE_CULL_PX) {
      view.setTier('chrome')
    } else {
      view.setTier(px < MIN_DETAIL_PX ? 'frame' : 'full')
      // fade nested content in as it grows past the detail threshold
      view.container.alpha = px < MIN_DETAIL_PX ? 0.4 + (0.6 * px) / MIN_DETAIL_PX : 1
      view.update()
    }
    if (px >= MIN_DETAIL_PX / 2) {
      for (const child of node.children) {
        this.applyLod(child, compose(sim, child.step), viewport)
      }
    }
  }

  // ---------------------------------------------------------------- flight animation

  /** exponential approach toward a camera pose that fits the fly target; rebases en route */
  private stepFlight(dt: number): boolean {
    if (!this.camera || !this.flyTarget) return false
    const viewport = this.viewportSize()
    const desiredFit = fitCamera(this.flyTarget, viewport)
    // express the desired pose relative to the current anchor
    const anchorToTarget = this.simBetween(this.camera.anchor, this.flyTarget)
    if (!anchorToTarget) return false
    const desired: Sim = compose(desiredFit.local, anchorToTarget)

    const current = this.camera.local
    const logRatio = Math.log(desired.s / current.s)
    // fixed point: viewport center in anchor space under the desired pose
    const [cx, cy] = applyToPoint(invert(desired), viewport.width / 2, viewport.height / 2)
    const [qx, qy] = applyToPoint(current, cx, cy)
    const dxCenter = viewport.width / 2 - qx
    const dyCenter = viewport.height / 2 - qy

    if (Math.abs(logRatio) < 0.002 && Math.hypot(dxCenter, dyCenter) < 0.5) {
      this.camera.local = desired
      return false
    }

    const k = 1 - Math.exp(-dt * 6)
    const newS = current.s * Math.exp(logRatio * k)
    const newQx = qx + dxCenter * k
    const newQy = qy + dyCenter * k
    this.camera.local = { s: newS, tx: newQx - newS * cx, ty: newQy - newS * cy }
    return true
  }

  /** transform from `from`'s local space to screen given camera at `from`'s ancestor — here:
   * sim mapping `to`-local coordinates into `from`-local coordinates (any two tree nodes) */
  private simBetween(from: RenderState, to: RenderState): Sim | null {
    const down = stepChain(to, from)
    if (down) return down
    const up = stepChain(from, to)
    if (up) return invert(up)
    // general case: via the root
    if (!this.root) return null
    const fromRoot = stepChain(from, this.root)
    const toRoot = stepChain(to, this.root)
    if (!fromRoot || !toRoot) return null
    return compose(invert(fromRoot), toRoot)
  }

  // ---------------------------------------------------------------- input

  private bindInput(canvas: HTMLCanvasElement): void {
    canvas.addEventListener('wheel', (event) => {
      event.preventDefault()
      if (!this.camera) return
      this.flyTarget = null
      const factor = Math.exp(-event.deltaY * 0.0015)
      const rect = canvas.getBoundingClientRect()
      zoomAt(this.camera, factor, event.clientX - rect.left, event.clientY - rect.top)
      this.invalidate()
    })

    canvas.addEventListener('pointerdown', (event) => {
      this.dragging = true
      this.dragMoved = false
      this.lastPointer = [event.clientX, event.clientY]
      canvas.setPointerCapture(event.pointerId)
    })

    canvas.addEventListener('pointermove', (event) => {
      if (this.dragging && this.camera) {
        const dx = event.clientX - this.lastPointer[0]
        const dy = event.clientY - this.lastPointer[1]
        if (Math.abs(dx) + Math.abs(dy) > 0) {
          if (Math.hypot(dx, dy) > 2) this.dragMoved = true
          this.flyTarget = null
          panBy(this.camera, dx, dy)
          this.lastPointer = [event.clientX, event.clientY]
          this.invalidate()
        }
      } else {
        this.updateHover(event)
      }
    })

    canvas.addEventListener('pointerup', (event) => {
      canvas.releasePointerCapture(event.pointerId)
      const wasDrag = this.dragMoved
      this.dragging = false
      if (!wasDrag) {
        const hit = this.hitTest(event)
        this.callbacks.onSelect(hit?.path ?? null)
      }
    })

    canvas.addEventListener('dblclick', (event) => {
      const hit = this.hitTest(event)
      if (hit) this.zoomToPath(hit.path)
    })

    canvas.addEventListener('pointerleave', () => {
      this.hoveredView?.setHovered(false)
      this.hoveredView = null
      this.invalidate()
    })
  }

  private pointerToScreen(event: MouseEvent): [number, number] {
    const rect = this.canvasEl!.getBoundingClientRect()
    return [event.clientX - rect.left, event.clientY - rect.top]
  }

  /** deepest state under the pointer with a minimum on-screen size */
  private hitTest(event: MouseEvent): RenderState | null {
    if (!this.camera) return null
    const [px, py] = this.pointerToScreen(event)

    let best: RenderState | null = null
    const descend = (node: RenderState, sim: Sim): void => {
      const x = sim.tx
      const y = sim.ty
      const w = node.width * sim.s
      const h = node.height * sim.s
      if (px < x || py < y || px > x + w || py > y + h) return
      if (Math.max(w, h) < MIN_HIT_PX) return
      best = node
      for (const child of node.children) descend(child, compose(sim, child.step))
    }
    descend(this.camera.anchor, this.camera.local)
    return best
  }

  private updateHover(event: MouseEvent): void {
    const hit = this.hitTest(event)
    const view = hit ? (this.viewsByPath.get(hit.path) ?? null) : null
    if (view !== this.hoveredView) {
      this.hoveredView?.setHovered(false)
      view?.setHovered(true)
      this.hoveredView = view
      this.callbacks.onHover(hit?.path ?? null)
      this.invalidate()
    }
  }
}
