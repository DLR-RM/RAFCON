// Builds an immutable render tree from the sm_json document.
//
// Coordinate design: gaphas meta data uses ONE global unit system — a nested state's
// meta size is numerically tiny at depth (sizes shrink ~5-15x per level). Rendering
// those raw units would underflow f32 on the GPU around depth ~35. The render tree
// therefore CANONICALIZES units per level: every state's local space is rescaled so
// its width is CANON (100) units, and the child `step` transforms carry the shrink
// factor. All local coordinates everywhere are O(1..100), which is what makes the
// camera's anchor re-rooting keep every GPU-bound number f32-safe at any depth.

import type { StateJson, StateType, Vec2 } from '@/services/protocol'
import type { Sim } from './transform'

export const CANON = 100

export interface PortAnchor {
  x: number
  y: number
  side: 'left' | 'right' | 'top' | 'bottom'
}

export interface RenderConnection {
  id: number
  kind: 'transition' | 'dataflow'
  /** polyline in the owning (parent) state's canonical local space */
  points: [number, number][]
}

export interface RenderState {
  id: string
  path: string
  name: string
  type: StateType
  json: StateJson
  /** canonical size: width === CANON, height keeps the aspect ratio */
  width: number
  height: number
  /** maps this state's canonical local space into the parent's canonical local space */
  step: Sim
  backgroundColor: number | null
  nameBox: { x: number; y: number; w: number; h: number }
  income: PortAnchor
  outcomes: Map<number, PortAnchor>
  inputPorts: Map<number, PortAnchor>
  outputPorts: Map<number, PortAnchor>
  scopedPorts: Map<number, PortAnchor>
  children: RenderState[]
  connections: RenderConnection[]
  isLibraryContent: boolean
  parent: RenderState | null
  depth: number
}

function vec(value: Vec2 | null | undefined): [number, number] | null {
  if (Array.isArray(value) && value.length >= 2 && isFinite(value[0]) && isFinite(value[1])) {
    return [value[0], value[1]]
  }
  return null
}

function colorFrom(meta: StateJson['meta']): number | null {
  const raw = meta?.background_color
  if (Array.isArray(raw) && raw.length >= 3) {
    const [r, g, b] = raw
    return (Math.round(r * 255) << 16) | (Math.round(g * 255) << 8) | Math.round(b * 255)
  }
  return null
}

export function buildRenderTree(root: StateJson): RenderState {
  const metaSize = vec(root.meta?.size) ?? [300, 200]
  return buildState(root, { s: 1, tx: 0, ty: 0 }, metaSize[0], metaSize[1], null, 0)
}

/**
 * @param metaW/metaH the state's size in its own meta units (the gaphas global units
 *   of its level); everything inside is multiplied by k = CANON / metaW
 */
function buildState(
  state: StateJson,
  step: Sim,
  metaW: number,
  metaH: number,
  parent: RenderState | null,
  depth: number,
): RenderState {
  const k = CANON / metaW
  const width = CANON
  const height = metaH * k
  const isLibrary = state.type === 'LibraryState' && !!state.state_copy

  const node: RenderState = {
    id: state.state_id,
    path: state.path,
    name: state.name,
    type: state.type,
    json: state,
    width,
    height,
    step,
    backgroundColor: colorFrom(state.meta) ?? (isLibrary ? colorFrom(state.state_copy!.meta) : null),
    nameBox: nameBoxFor(state, k, width, height),
    income: incomeAnchor(state, k, height),
    outcomes: outcomeAnchors(state, k, width, height),
    inputPorts: dataPortAnchors(state.input_data_ports, k, 'left', width, height),
    outputPorts: dataPortAnchors(state.output_data_ports, k, 'right', width, height),
    scopedPorts: new Map(),
    children: [],
    connections: [],
    isLibraryContent: false,
    parent,
    depth,
  }

  if (isLibrary && state.state_copy) {
    // library content fills the frame below the name, scaled to fit — this is
    // where depth shrinks geometrically
    const copy = state.state_copy
    const copyMetaSize = vec(copy.meta?.size) ?? [300, 200]
    const copyAspect = copyMetaSize[1] / copyMetaSize[0]
    const copyCanonW = CANON
    const copyCanonH = CANON * copyAspect
    const inset = Math.min(width, height) * 0.06
    const availW = width - 2 * inset
    const availH = height - node.nameBox.h - 2 * inset
    const scale = Math.max(1e-9, Math.min(availW / copyCanonW, availH / copyCanonH))
    const childStep: Sim = {
      s: scale,
      tx: inset + (availW - copyCanonW * scale) / 2,
      ty: node.nameBox.h + inset + (availH - copyCanonH * scale) / 2,
    }
    const copyNode = buildState(copy, childStep, copyMetaSize[0], copyMetaSize[1], node, depth + 1)
    copyNode.isLibraryContent = true
    node.children.push(copyNode)
    return node
  }

  const childStates = state.states ? Object.values(state.states) : []
  if (childStates.length) {
    const layouts = layoutChildren(childStates, k, width, height, node.nameBox.h)
    for (const { child, rect, childMetaSize } of layouts) {
      // child canonical space is CANON wide; the step carries the shrink factor
      const childStep: Sim = { s: rect.w / CANON, tx: rect.x, ty: rect.y }
      node.children.push(
        buildState(child, childStep, childMetaSize[0], childMetaSize[1], node, depth + 1),
      )
    }
    buildConnections(state, node, k)
  }

  return node
}

interface ChildLayout {
  child: StateJson
  /** rect in the parent's canonical space */
  rect: { x: number; y: number; w: number; h: number }
  /** the child's size in its own meta units */
  childMetaSize: [number, number]
}

const CHILD_AREA_FILL = 0.8

function layoutChildren(
  children: StateJson[],
  k: number,
  width: number,
  height: number,
  nameHeight: number,
): ChildLayout[] {
  const layouts: ChildLayout[] = []
  const withoutMeta: StateJson[] = []

  for (const child of children) {
    const pos = vec(child.meta?.rel_pos)
    const size = vec(child.meta?.size)
    if (pos && size && size[0] > 0 && size[1] > 0) {
      layouts.push({
        child,
        rect: { x: pos[0] * k, y: pos[1] * k, w: size[0] * k, h: size[1] * k },
        childMetaSize: size,
      })
    } else {
      withoutMeta.push(child)
    }
  }

  if (withoutMeta.length) {
    // auto-layout fallback: grid inside the content area below the name
    const areaX = width * (1 - CHILD_AREA_FILL) * 0.5
    const areaY = Math.max(nameHeight, height * (1 - CHILD_AREA_FILL) * 0.5)
    const areaW = width * CHILD_AREA_FILL
    const areaH = height - areaY - height * (1 - CHILD_AREA_FILL) * 0.5
    const columns = Math.ceil(Math.sqrt(withoutMeta.length))
    const rows = Math.ceil(withoutMeta.length / columns)
    const cellW = areaW / columns
    const cellH = areaH / rows
    withoutMeta.forEach((child, index) => {
      const col = index % columns
      const row = Math.floor(index / columns)
      const rect = {
        x: areaX + col * cellW + cellW * 0.1,
        y: areaY + row * cellH + cellH * 0.1,
        w: cellW * 0.8,
        h: cellH * 0.8,
      }
      // meta units of an auto-laid-out child are its rect in parent canonical units
      layouts.push({ child, rect, childMetaSize: [rect.w, rect.h] })
    })
  }
  return layouts
}

function nameBoxFor(state: StateJson, k: number, width: number, height: number): RenderState['nameBox'] {
  const pos = vec(state.meta?.name?.rel_pos)
  const size = vec(state.meta?.name?.size)
  if (pos && size) return { x: pos[0] * k, y: pos[1] * k, w: size[0] * k, h: size[1] * k }
  return { x: width * 0.03, y: height * 0.03, w: width * 0.94, h: Math.min(height * 0.15, width * 0.12) }
}

function incomeAnchor(state: StateJson, k: number, height: number): PortAnchor {
  const pos = vec(state.meta?.income?.rel_pos)
  if (pos) return { x: 0, y: pos[1] * k, side: 'left' }
  return { x: 0, y: height * 0.35, side: 'left' }
}

function outcomeAnchors(
  state: StateJson,
  k: number,
  width: number,
  height: number,
): Map<number, PortAnchor> {
  const anchors = new Map<number, PortAnchor>()
  const regular = state.outcomes.filter((o) => o.outcome_id >= 0)
  let regularIndex = 0
  for (const outcome of state.outcomes) {
    const pos = vec(outcome.rel_pos)
    if (pos) {
      const side = outcome.outcome_id < 0 ? 'top' : 'right'
      anchors.set(outcome.outcome_id, { x: Math.min(pos[0] * k, width), y: pos[1] * k, side })
      if (outcome.outcome_id >= 0) regularIndex += 1
      continue
    }
    if (outcome.outcome_id === -1) {
      anchors.set(-1, { x: width * 0.96, y: 0, side: 'top' })
    } else if (outcome.outcome_id === -2) {
      anchors.set(-2, { x: width * 0.9, y: 0, side: 'top' })
    } else {
      const spacing = height / (regular.length + 1)
      regularIndex += 1
      anchors.set(outcome.outcome_id, { x: width, y: spacing * regularIndex, side: 'right' })
    }
  }
  return anchors
}

function dataPortAnchors(
  ports: StateJson['input_data_ports'],
  k: number,
  side: 'left' | 'right',
  width: number,
  height: number,
): Map<number, PortAnchor> {
  const anchors = new Map<number, PortAnchor>()
  ports.forEach((port, index) => {
    const pos = vec(port.rel_pos)
    if (pos) {
      anchors.set(port.data_port_id, { x: side === 'left' ? 0 : width, y: pos[1] * k, side })
    } else {
      const y = height * (0.6 + (0.35 * (index + 1)) / (ports.length + 1))
      anchors.set(port.data_port_id, { x: side === 'left' ? 0 : width, y, side })
    }
  })
  return anchors
}

/** Resolve transition and data flow polylines in the container's canonical space */
function buildConnections(state: StateJson, node: RenderState, k: number): void {
  const childById = new Map(node.children.map((child) => [child.id, child]))

  state.scoped_variables?.forEach((variable, index) => {
    const pos = vec(variable.rel_pos)
    node.scopedPorts.set(
      variable.scoped_variable_id,
      pos
        ? { x: pos[0] * k, y: Math.min(pos[1] * k, node.height), side: 'bottom' }
        : { x: (node.width * (index + 1)) / 6, y: node.height, side: 'bottom' },
    )
  })

  const childAnchorToParent = (child: RenderState, anchor: PortAnchor): [number, number] => [
    child.step.tx + child.step.s * anchor.x,
    child.step.ty + child.step.s * anchor.y,
  ]

  for (const transition of state.transitions ?? []) {
    const points: [number, number][] = []
    if (transition.from_state && childById.has(transition.from_state)) {
      const child = childById.get(transition.from_state)!
      const anchor = child.outcomes.get(transition.from_outcome ?? 0)
      if (anchor) points.push(childAnchorToParent(child, anchor))
    } else {
      points.push([node.income.x, node.income.y])
    }
    for (const waypoint of transition.waypoints ?? []) {
      const w = vec(waypoint)
      if (w) points.push([w[0] * k, w[1] * k])
    }
    if (transition.to_state && childById.has(transition.to_state)) {
      const child = childById.get(transition.to_state)!
      points.push(childAnchorToParent(child, child.income))
    } else if (transition.to_outcome !== null && transition.to_outcome !== undefined) {
      const anchor = node.outcomes.get(transition.to_outcome)
      if (anchor) points.push([anchor.x, anchor.y])
    }
    if (points.length >= 2) {
      node.connections.push({ id: transition.transition_id, kind: 'transition', points })
    }
  }

  for (const flow of state.data_flows ?? []) {
    const points: [number, number][] = []
    const resolve = (stateId: string, key: number, isSource: boolean): [number, number] | null => {
      if (stateId === state.state_id) {
        const scoped = node.scopedPorts.get(key)
        if (scoped) return [scoped.x, scoped.y]
        const own = isSource ? node.inputPorts.get(key) : node.outputPorts.get(key)
        if (own) return [own.x, own.y]
        const other = isSource ? node.outputPorts.get(key) : node.inputPorts.get(key)
        return other ? [other.x, other.y] : null
      }
      const child = childById.get(stateId)
      if (!child) return null
      const anchor =
        (isSource ? child.outputPorts.get(key) : child.inputPorts.get(key)) ??
        (isSource ? child.inputPorts.get(key) : child.outputPorts.get(key))
      return anchor ? childAnchorToParent(child, anchor) : null
    }
    const source = resolve(flow.from_state, flow.from_key, true)
    const target = resolve(flow.to_state, flow.to_key, false)
    if (!source || !target) continue
    points.push(source)
    for (const waypoint of flow.waypoints ?? []) {
      const w = vec(waypoint)
      if (w) points.push([w[0] * k, w[1] * k])
    }
    points.push(target)
    node.connections.push({ id: flow.data_flow_id, kind: 'dataflow', points })
  }
}

/** Depth-first iteration over the render tree */
export function walkRenderTree(node: RenderState, visit: (node: RenderState) => boolean | void): void {
  if (visit(node) === false) return
  for (const child of node.children) walkRenderTree(child, visit)
}

/** Find the render node for an execution path (descends through library content) */
export function findByPath(root: RenderState, path: string): RenderState | null {
  let found: RenderState | null = null
  walkRenderTree(root, (node) => {
    if (node.path === path) {
      found = node
      return false
    }
    if (node !== root && !path.startsWith(node.path + '/') && node.path !== path) return false
    return true
  })
  return found
}
