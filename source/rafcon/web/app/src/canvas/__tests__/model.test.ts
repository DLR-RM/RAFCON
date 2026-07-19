import { describe, expect, it } from 'vitest'
import type { StateJson, StateMachineJson } from '@/services/protocol'
import { buildRenderTree, CANON, findByPath, walkRenderTree } from '../model'
import backwardFixture from '@/__fixtures__/backward_step_barrier.json'
import deepLibrariesFixture from '@/__fixtures__/deep_libraries.json'

const backward = backwardFixture as unknown as StateMachineJson
const deepLibraries = deepLibrariesFixture as unknown as StateMachineJson

describe('buildRenderTree', () => {
  it('uses meta geometry when present, canonicalized to width 100', () => {
    const root = buildRenderTree(backward.root_state)
    const metaW = 310.837765957447
    const metaH = 208.85970744680878
    const k = CANON / metaW
    expect(root.width).toBe(CANON)
    expect(root.height).toBeCloseTo(metaH * k)
    const concurrency = root.children.find((child) => child.id === 'OOECFM')!
    expect(concurrency.step.tx).toBeCloseTo(74.03536402925539 * k)
    // the step scale carries the level shrink: childMetaW * k / CANON
    expect(concurrency.step.s).toBeCloseTo((73.77659574468102 * k) / CANON)
    expect(concurrency.width).toBe(CANON)
    expect(concurrency.type).toBe('BarrierConcurrencyState')
  })

  it('resolves transitions into polylines', () => {
    const root = buildRenderTree(backward.root_state)
    expect(root.connections.filter((c) => c.kind === 'transition').length).toBeGreaterThan(0)
    for (const connection of root.connections) {
      expect(connection.points.length).toBeGreaterThanOrEqual(2)
      for (const point of connection.points) {
        expect(isFinite(point[0])).toBe(true)
        expect(isFinite(point[1])).toBe(true)
      }
    }
  })

  it('scales library content to fit and keeps execution paths intact', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    let libraryContent = 0
    let maxDepth = 0
    walkRenderTree(root, (node) => {
      if (node.isLibraryContent) {
        libraryContent += 1
        expect(node.step.s).toBeGreaterThan(0)
        expect(node.step.s).toBeLessThan(1)
      }
      maxDepth = Math.max(maxDepth, node.depth)
    })
    expect(libraryContent).toBeGreaterThan(0)
    expect(maxDepth).toBeGreaterThanOrEqual(5)
  })

  it('auto-layouts children without meta', () => {
    const bare: StateJson = {
      state_id: 'ROOT',
      name: 'root',
      type: 'HierarchyState',
      path: 'ROOT',
      description: null,
      input_data_ports: [],
      output_data_ports: [],
      outcomes: [{ outcome_id: 0, name: 'success', rel_pos: null }],
      meta: null,
      start_state_id: 'A',
      states: {
        A: {
          state_id: 'A',
          name: 'a',
          type: 'ExecutionState',
          path: 'ROOT/A',
          description: null,
          input_data_ports: [],
          output_data_ports: [],
          outcomes: [],
          meta: null,
        },
        B: {
          state_id: 'B',
          name: 'b',
          type: 'ExecutionState',
          path: 'ROOT/B',
          description: null,
          input_data_ports: [],
          output_data_ports: [],
          outcomes: [],
          meta: null,
        },
      },
      transitions: [],
      data_flows: [],
      scoped_variables: [],
    }
    const root = buildRenderTree(bare)
    expect(root.children).toHaveLength(2)
    for (const child of root.children) {
      expect(child.step.tx).toBeGreaterThanOrEqual(0)
      expect(child.step.tx + child.step.s * child.width).toBeLessThanOrEqual(root.width)
      expect(child.step.ty + child.step.s * child.height).toBeLessThanOrEqual(root.height)
    }
  })

  it('findByPath descends through library content', () => {
    const root = buildRenderTree(deepLibraries.root_state)
    let deepest: string | null = null
    let deepestDepth = -1
    walkRenderTree(root, (node) => {
      if (node.depth > deepestDepth) {
        deepestDepth = node.depth
        deepest = node.path
      }
    })
    expect(deepest).not.toBeNull()
    expect(findByPath(root, deepest!)?.path).toBe(deepest)
  })
})
