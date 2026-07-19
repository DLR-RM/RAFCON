import { beforeEach, describe, expect, it } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { useStateMachinesStore } from '../statemachines'
import type { StateMachineJson, SyncPayload } from '@/services/protocol'

function smJson(id: number): StateMachineJson {
  return {
    state_machine_id: id,
    file_system_path: `/sm/${id}`,
    root_state: {
      state_id: `ROOT${id}`,
      name: `root ${id}`,
      type: 'HierarchyState',
      path: `ROOT${id}`,
      description: null,
      input_data_ports: [],
      output_data_ports: [],
      outcomes: [],
      meta: null,
      states: {},
    },
  }
}

function sync(ids: number[], active: number | null): SyncPayload {
  return {
    state_machines: ids.map((id) => ({
      state_machine_id: id,
      path: `/sm/${id}`,
      sm_json: smJson(id),
      state_statuses: [],
    })),
    execution_status: 'STOPPED',
    active_state_machine_id: active,
    global_variables: [],
    log_backlog: [],
  }
}

describe('statemachines store pinning', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('pinned window keeps the pinned id through sync, even before the SM arrives', () => {
    const store = useStateMachinesStore()
    store.pin(7)
    store.applySync(sync([1, 2], 1))
    expect(store.selectedSmId).toBe(7)
    expect(store.selectedMachine).toBeNull()

    store.applySync(sync([1, 7], 1))
    expect(store.selectedSmId).toBe(7)
    expect(store.selectedMachine?.state_machine_id).toBe(7)
  })

  it('selectMachine is a no-op when pinned', () => {
    const store = useStateMachinesStore()
    store.pin(2)
    store.applySync(sync([1, 2], 1))
    store.selectMachine(1)
    expect(store.selectedSmId).toBe(2)
  })

  it('removing the pinned SM keeps the pin instead of falling back', () => {
    const store = useStateMachinesStore()
    store.pin(2)
    store.applySync(sync([1, 2], 1))
    store.removeMachine(2)
    expect(store.selectedSmId).toBe(2)
    expect(store.selectedMachine).toBeNull()
  })

  it('select for a foreign SM is ignored when pinned', () => {
    const store = useStateMachinesStore()
    store.pin(2)
    store.applySync(sync([1, 2], 1))
    store.select(1, 'ROOT1')
    expect(store.selection).toBeNull()
    store.select(2, 'ROOT2')
    expect(store.selection).toEqual({ smId: 2, path: 'ROOT2' })
  })

  it('unpinned behavior unchanged: active id wins, removal falls back', () => {
    const store = useStateMachinesStore()
    store.applySync(sync([1, 2], 2))
    expect(store.selectedSmId).toBe(2)
    store.removeMachine(2)
    expect(store.selectedSmId).toBe(1)
  })
})
