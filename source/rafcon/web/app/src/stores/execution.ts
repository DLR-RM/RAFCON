import { defineStore } from 'pinia'
import type { ExecutionCommand, ExecutionStatus } from '@/services/protocol'
import { EXECUTION_COMMAND } from '@/services/protocol'
import { getSocket } from '@/services/socket'

export const useExecutionStore = defineStore('execution', {
  state: () => ({
    status: 'STOPPED' as ExecutionStatus,
  }),
  getters: {
    isRunning(state): boolean {
      return !['STOPPED', 'FINISHED', 'PAUSED'].includes(state.status)
    },
  },
  actions: {
    setStatus(status: ExecutionStatus) {
      this.status = status
    },
    sendCommand(command: ExecutionCommand, smId: number | null = null, statePath: string | null = null) {
      getSocket()?.send(EXECUTION_COMMAND, {
        command,
        state_machine_id: smId,
        state_path: statePath,
      })
    },
  },
})
