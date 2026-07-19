import { defineStore } from 'pinia'
import type {
  StateExecutionStatus,
  StateJson,
  StateMachineJson,
  SyncPayload,
  SyncStateMachineEntry,
} from '@/services/protocol'
import { CLOSE_STATE_MACHINE } from '@/services/protocol'
import { getSocket } from '@/services/socket'

export interface Selection {
  smId: number
  path: string
}

/** Walk a state tree depth-first, descending into container children and library copies */
export function walkStates(state: StateJson, visit: (state: StateJson) => void): void {
  visit(state)
  if (state.states) {
    for (const child of Object.values(state.states)) walkStates(child, visit)
  }
  if (state.state_copy) walkStates(state.state_copy, visit)
}

function buildPathIndex(sm: StateMachineJson): Map<string, StateJson> {
  const index = new Map<string, StateJson>()
  walkStates(sm.root_state, (state) => index.set(state.path, state))
  return index
}

export const useStateMachinesStore = defineStore('statemachines', {
  state: () => ({
    machines: {} as Record<number, StateMachineJson>,
    pathIndexes: {} as Record<number, Map<string, StateJson>>,
    statuses: {} as Record<number, Record<string, StateExecutionStatus>>,
    selectedSmId: null as number | null,
    selection: null as Selection | null,
    // set once at startup for ?sm=<id> windows: the window is locked to this state machine
    pinnedSmId: null as number | null,
  }),

  getters: {
    selectedMachine(state): StateMachineJson | null {
      return state.selectedSmId !== null ? (state.machines[state.selectedSmId] ?? null) : null
    },
    selectedState(state): StateJson | null {
      if (!state.selection) return null
      return state.pathIndexes[state.selection.smId]?.get(state.selection.path) ?? null
    },
    machineList(state): StateMachineJson[] {
      return Object.values(state.machines)
    },
  },

  actions: {
    applySync(payload: SyncPayload) {
      const previousSelected = this.selectedSmId
      this.machines = {}
      this.pathIndexes = {}
      this.statuses = {}
      this.selection = null
      for (const entry of payload.state_machines) {
        this.addMachine(entry)
      }
      if (this.pinnedSmId !== null) {
        this.selectedSmId = this.pinnedSmId
        return
      }
      const ids = Object.keys(this.machines).map(Number)
      const active = payload.active_state_machine_id
      if (previousSelected !== null && previousSelected in this.machines) {
        this.selectedSmId = previousSelected
      } else if (active !== null && active in this.machines) {
        this.selectedSmId = active
      } else {
        this.selectedSmId = ids[0] ?? null
      }
    },

    addMachine(entry: SyncStateMachineEntry) {
      const smId = entry.state_machine_id
      if (!entry.sm_json) {
        console.error(`State machine ${smId} arrived without sm_json — ignoring`)
        return
      }
      this.machines[smId] = entry.sm_json
      this.pathIndexes[smId] = buildPathIndex(entry.sm_json)
      this.statuses[smId] = {}
      for (const status of entry.state_statuses ?? []) {
        this.statuses[smId][status.state_path] = status.status
      }
      if (this.selectedSmId === null && this.pinnedSmId === null) this.selectedSmId = smId
    },

    removeMachine(smId: number) {
      delete this.machines[smId]
      delete this.pathIndexes[smId]
      delete this.statuses[smId]
      if (this.selection?.smId === smId) this.selection = null
      // a pinned window keeps pointing at its (now closed) state machine
      if (this.selectedSmId === smId && this.pinnedSmId === null) {
        const ids = Object.keys(this.machines).map(Number)
        this.selectedSmId = ids[0] ?? null
      }
    },

    setStateStatus(smId: number, path: string, status: StateExecutionStatus) {
      if (!this.statuses[smId]) this.statuses[smId] = {}
      if (status === 'INACTIVE') {
        delete this.statuses[smId][path]
      } else {
        this.statuses[smId][path] = status
      }
    },

    select(smId: number, path: string) {
      if (this.pinnedSmId !== null && smId !== this.pinnedSmId) return
      this.selection = { smId, path }
      this.selectedSmId = smId
    },

    clearSelection() {
      this.selection = null
    },

    selectMachine(smId: number) {
      if (this.pinnedSmId !== null) return
      this.selectedSmId = smId
    },

    /** Ask the server to close a state machine; removal arrives via STATE_MACHINE_REMOVED */
    closeMachine(smId: number) {
      getSocket()?.send(CLOSE_STATE_MACHINE, { state_machine_id: smId })
    },

    /** Lock this window to one state machine (?sm=<id> windows); call before the first SYNC */
    pin(smId: number) {
      this.pinnedSmId = smId
      this.selectedSmId = smId
    },
  },
})
