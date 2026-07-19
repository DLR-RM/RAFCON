import { defineStore } from 'pinia'
import type { ConnectionState } from '@/services/ws'

export const useConnectionStore = defineStore('connection', {
  state: () => ({
    state: 'connecting' as ConnectionState,
    lastError: null as string | null,
  }),
  actions: {
    setState(state: ConnectionState) {
      this.state = state
    },
    setError(message: string) {
      this.lastError = message
    },
  },
})
