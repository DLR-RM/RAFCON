import { defineStore } from 'pinia'
import type { LogRecordJson } from '@/services/protocol'

const MAX_LINES = 5000

export type LogLevel = 'VERBOSE' | 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR'

export const useLogsStore = defineStore('logs', {
  state: () => ({
    records: [] as LogRecordJson[],
    minLevel: 'INFO' as LogLevel,
    followTail: true,
  }),
  getters: {
    filtered(state): LogRecordJson[] {
      const order: Record<string, number> = { VERBOSE: 0, DEBUG: 1, INFO: 2, WARNING: 3, ERROR: 4 }
      const min = order[state.minLevel] ?? 2
      return state.records.filter((record) => (order[record.level] ?? 2) >= min)
    },
  },
  actions: {
    addRecords(records: LogRecordJson[]) {
      this.records.push(...records)
      if (this.records.length > MAX_LINES) {
        this.records.splice(0, this.records.length - MAX_LINES)
      }
    },
    setRecords(records: LogRecordJson[]) {
      this.records = [...records]
    },
    setMinLevel(level: LogLevel) {
      this.minLevel = level
    },
    setFollowTail(follow: boolean) {
      this.followTail = follow
    },
  },
})
