// Pixi view objects for render states. One NodeView per RenderState; drawing is
// dirty-flag based and level-of-detail aware. All coordinates are the state's
// local space — the parent chain (and the camera layer) provides the transforms.
//
// Visual language: "clean flat modern" node-editor look on the RAFCON dark palette —
// rounded cards with a fake soft drop shadow, per-depth surface tinting, a name
// header band with a type-colored chip, smooth curved connections and glow rings
// for selection / execution status.

import { Container, Graphics, Text } from 'pixi.js'
import type { StateExecutionStatus } from '@/services/protocol'
import type { PortAnchor, RenderState } from './model'

// 'chrome' = the node itself is far larger than the viewport: its own graphics are
// culled (keeps GPU numbers small) while its children remain visible
export type LodTier = 'hidden' | 'dot' | 'frame' | 'full' | 'chrome'

const CARD_BASE = 0x232833
const BORDER_DEFAULT = 0x3a4150
const BORDER_HOVERED = 0x747981
const RING_SELECTED = 0x2896ff
const RING_ACTIVE = 0x94e480
const RING_WAITING = 0xdfe480
const OUTCOME_COLOR = 0xababa9
const OUTCOME_ABORTED = 0xf21000
const OUTCOME_PREEMPTED = 0x359dff
const INCOME_COLOR = 0xababa9
const DATA_PORT_COLOR = 0xffd24d
const SCOPED_COLOR = 0xd4b13e
const TRANSITION_COLOR = 0x7d838d
const DATAFLOW_COLOR = 0xb89b4a
const PORT_RIM = 0x1a1f29
const NAME_COLOR = 0xf2f4f8

const TYPE_ACCENT: Record<string, number> = {
  ExecutionState: 0x83868c,
  HierarchyState: 0x2896ff,
  BarrierConcurrencyState: 0xfd8009,
  PreemptiveConcurrencyState: 0xa06cff,
  LibraryState: 0x5dcd61,
}

/** lighten an 0xRRGGBB color toward white by `amount` (0..1) */
function lighten(color: number, amount: number): number {
  const r = (color >> 16) & 0xff
  const g = (color >> 8) & 0xff
  const b = color & 0xff
  const mix = (c: number) => Math.min(255, Math.round(c + (255 - c) * amount))
  return (mix(r) << 16) | (mix(g) << 8) | mix(b)
}

/** draw a Catmull-Rom smoothed path through the points; returns the end tangent angle */
function drawSmoothPath(gfx: Graphics, points: [number, number][]): number {
  gfx.moveTo(points[0][0], points[0][1])
  const last = points[points.length - 1]
  const prev = points[points.length - 2]
  if (points.length === 2) {
    gfx.lineTo(last[0], last[1])
    return Math.atan2(last[1] - prev[1], last[0] - prev[0])
  }
  let endTangent: [number, number] = [last[0] - prev[0], last[1] - prev[1]]
  for (let i = 0; i < points.length - 1; i++) {
    const p0 = points[Math.max(0, i - 1)]
    const p1 = points[i]
    const p2 = points[i + 1]
    const p3 = points[Math.min(points.length - 1, i + 2)]
    const c1x = p1[0] + (p2[0] - p0[0]) / 6
    const c1y = p1[1] + (p2[1] - p0[1]) / 6
    const c2x = p2[0] - (p3[0] - p1[0]) / 6
    const c2y = p2[1] - (p3[1] - p1[1]) / 6
    gfx.bezierCurveTo(c1x, c1y, c2x, c2y, p2[0], p2[1])
    if (i === points.length - 2) endTangent = [p2[0] - c2x, p2[1] - c2y]
  }
  return Math.atan2(endTangent[1], endTangent[0])
}

export class NodeView {
  readonly container: Container
  readonly frame: Graphics
  readonly ringGfx: Graphics
  readonly connectionsGfx: Graphics
  readonly portsGfx: Graphics
  readonly childrenLayer: Container
  private nameText: Text | null = null

  tier: LodTier = 'full'
  status: StateExecutionStatus | null = null
  selected = false
  hovered = false
  private frameDirty = true
  private drawnTier: LodTier | null = null

  constructor(readonly state: RenderState) {
    this.container = new Container()
    this.container.position.set(state.step.tx, state.step.ty)
    this.container.scale.set(state.step.s)
    this.container.eventMode = 'none'
    this.container.interactiveChildren = false

    this.frame = new Graphics()
    this.connectionsGfx = new Graphics()
    this.childrenLayer = new Container()
    this.portsGfx = new Graphics()
    this.ringGfx = new Graphics()
    this.container.addChild(this.frame, this.connectionsGfx, this.childrenLayer, this.portsGfx, this.ringGfx)
  }

  setStatus(status: StateExecutionStatus | null): void {
    if (this.status !== status) {
      this.status = status
      this.frameDirty = true
    }
  }

  setSelected(selected: boolean): void {
    if (this.selected !== selected) {
      this.selected = selected
      this.frameDirty = true
    }
  }

  setHovered(hovered: boolean): void {
    if (this.hovered !== hovered) {
      this.hovered = hovered
      this.frameDirty = true
    }
  }

  /** modulate the status/selection ring without re-tessellating (execution pulse) */
  setRingAlpha(alpha: number): void {
    this.ringGfx.alpha = alpha
  }

  get isActive(): boolean {
    return this.status !== null && this.status !== 'WAIT_FOR_NEXT_STATE'
  }

  setTier(tier: LodTier): void {
    if (tier === 'hidden') {
      this.container.visible = false
      return
    }
    this.container.visible = true
    if (this.tier !== tier) {
      this.tier = tier
      this.frameDirty = true
    }
    const showDetails = tier === 'full'
    this.frame.visible = tier !== 'chrome'
    this.ringGfx.visible = tier !== 'chrome'
    this.connectionsGfx.visible = showDetails
    this.portsGfx.visible = showDetails
    this.childrenLayer.visible = tier !== 'dot'
    if (this.nameText) this.nameText.visible = showDetails
  }

  /** redraw graphics if anything changed; cheap when clean */
  update(): void {
    if (!this.container.visible) return
    if (this.frameDirty || this.drawnTier !== this.tier) {
      this.drawFrame()
      this.drawRing()
      if (this.tier === 'full') {
        this.ensureName()
        if (this.drawnTier !== 'full') {
          this.drawPorts()
          this.drawConnections()
        }
      }
      this.drawnTier = this.tier
      this.frameDirty = false
    }
  }

  private cardFill(): number {
    if (this.state.backgroundColor !== null) return this.state.backgroundColor
    // nested cards read as stacked surfaces: slightly lighter with each level
    return lighten(CARD_BASE, Math.min(this.state.depth, 6) * 0.035)
  }

  private drawFrame(): void {
    const { width: w, height: h } = this.state
    const bw = Math.min(w, h) / 25
    const radius = Math.min(w, h) * 0.06
    const fill = this.cardFill()
    this.frame.clear()
    if (this.tier === 'dot') {
      this.frame.roundRect(0, 0, w, h, radius).fill(this.isActive ? RING_ACTIVE : fill)
      return
    }

    // fake soft drop shadow: two expanded translucent rounds below the card
    this.frame
      .roundRect(-bw * 0.3, bw * 0.6, w + bw * 0.6, h + bw * 0.6, radius * 1.15)
      .fill({ color: 0x000000, alpha: 0.1 })
      .roundRect(-bw * 0.1, bw * 0.25, w + bw * 0.2, h + bw * 0.25, radius * 1.05)
      .fill({ color: 0x000000, alpha: 0.14 })

    const borderColor = this.hovered && !this.selected ? BORDER_HOVERED : BORDER_DEFAULT
    this.frame
      .roundRect(0, 0, w, h, radius)
      .fill(fill)
      .stroke({ width: bw * 0.5, color: borderColor, alignment: 1 })

    if (this.tier === 'full') {
      // type chip in the top-left corner
      const chip = Math.min(w, h) * 0.045
      this.frame
        .roundRect(bw * 0.9, bw * 0.9, chip, chip, chip * 0.3)
        .fill(TYPE_ACCENT[this.state.type] ?? 0x83868c)

      // library content gets an inset backdrop panel behind the scaled copy
      if (this.state.type === 'LibraryState' && this.state.children.length === 1) {
        const copy = this.state.children[0]
        const cw = copy.width * copy.step.s
        const ch = copy.height * copy.step.s
        const pad = bw * 0.5
        this.frame
          .roundRect(copy.step.tx - pad, copy.step.ty - pad, cw + pad * 2, ch + pad * 2, radius * 0.7)
          .fill({ color: 0x000000, alpha: 0.16 })
      }
    }
  }

  /** selection / execution ring in its own layer so pulses only change alpha */
  private drawRing(): void {
    this.ringGfx.clear()
    this.ringGfx.alpha = 1
    if (this.tier === 'dot') return
    let color: number | null = null
    if (this.status === 'WAIT_FOR_NEXT_STATE') color = RING_WAITING
    else if (this.status) color = RING_ACTIVE
    else if (this.selected) color = RING_SELECTED
    if (color === null) return
    const { width: w, height: h } = this.state
    const bw = Math.min(w, h) / 25
    const radius = Math.min(w, h) * 0.06
    // single opaque stroke: stacked translucent strokes overlap at the rounded
    // corners and produce blotchy alpha artifacts
    this.ringGfx
      .roundRect(0, 0, w, h, radius)
      .stroke({ width: bw * 0.9, color, alpha: 1, alignment: 1 })
  }

  private ensureName(): void {
    if (this.nameText) return
    const box = this.state.nameBox
    const text = new Text({
      text: this.state.name,
      resolution: 2,
      style: {
        fontFamily: "'Source Sans Pro', 'Source Sans 3', Inter, ui-sans-serif, system-ui, sans-serif",
        fontSize: 32,
        fontWeight: '600',
        fill: NAME_COLOR,
      },
    })
    const scale = Math.min(box.w / Math.max(text.width, 1), box.h / Math.max(text.height, 1))
    text.scale.set(scale)
    text.position.set(box.x, box.y)
    this.nameText = text
    this.container.addChild(text)
    // keep the ring above everything
    this.container.setChildIndex(this.ringGfx, this.container.children.length - 1)
  }

  private portRadius(): number {
    return Math.min(this.state.width, this.state.height) / 32
  }

  private drawPort(gfx: Graphics, anchor: PortAnchor, color: number, square = false): void {
    const radius = this.portRadius()
    if (square) {
      const side = radius * 1.5
      gfx
        .roundRect(anchor.x - side / 2, anchor.y - side / 2, side, side, side * 0.3)
        .fill(color)
        .stroke({ width: radius * 0.3, color: PORT_RIM })
    } else {
      gfx
        .circle(anchor.x, anchor.y, radius)
        .fill(color)
        .stroke({ width: radius * 0.35, color: PORT_RIM })
    }
  }

  private drawPorts(): void {
    const gfx = this.portsGfx
    gfx.clear()
    this.drawPort(gfx, this.state.income, INCOME_COLOR)
    for (const [outcomeId, anchor] of this.state.outcomes) {
      const color =
        outcomeId === -1 ? OUTCOME_ABORTED : outcomeId === -2 ? OUTCOME_PREEMPTED : OUTCOME_COLOR
      this.drawPort(gfx, anchor, color)
    }
    for (const anchor of this.state.inputPorts.values()) this.drawPort(gfx, anchor, DATA_PORT_COLOR, true)
    for (const anchor of this.state.outputPorts.values()) this.drawPort(gfx, anchor, DATA_PORT_COLOR, true)
    for (const anchor of this.state.scopedPorts.values()) this.drawPort(gfx, anchor, SCOPED_COLOR, true)
  }

  private drawConnections(): void {
    const gfx = this.connectionsGfx
    const lineWidth = Math.min(this.state.width, this.state.height) / 120
    gfx.clear()
    for (const connection of this.state.connections) {
      const isTransition = connection.kind === 'transition'
      const color = isTransition ? TRANSITION_COLOR : DATAFLOW_COLOR
      const width = isTransition ? lineWidth : lineWidth * 0.55
      const alpha = isTransition ? 0.95 : 0.65
      const points = connection.points
      const angle = drawSmoothPath(gfx, points)
      gfx.stroke({ width, color, alpha, cap: 'round', join: 'round' })
      // arrow head aligned with the curve's end tangent
      const [x2, y2] = points[points.length - 1]
      const size = width * 4
      gfx
        .moveTo(x2, y2)
        .lineTo(x2 - size * Math.cos(angle - 0.42), y2 - size * Math.sin(angle - 0.42))
        .lineTo(x2 - size * Math.cos(angle + 0.42), y2 - size * Math.sin(angle + 0.42))
        .closePath()
        .fill({ color, alpha })
    }
  }
}

/** Build the NodeView tree mirroring a RenderState tree; returns all views by path */
export function buildViews(root: RenderState): { rootView: NodeView; byPath: Map<string, NodeView> } {
  const byPath = new Map<string, NodeView>()

  function build(state: RenderState): NodeView {
    const view = new NodeView(state)
    byPath.set(state.path, view)
    for (const child of state.children) {
      view.childrenLayer.addChild(build(child).container)
    }
    return view
  }

  return { rootView: build(root), byPath }
}
