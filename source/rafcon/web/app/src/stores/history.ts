import { defineStore } from 'pinia'
import type { HistoryItemJson } from '@/services/protocol'

const MAX_ITEMS = 10000

export const useHistoryStore = defineStore('history', {
  state: () => ({
    items: [] as HistoryItemJson[],
  }),
  actions: {
    addItems(items: HistoryItemJson[]) {
      this.items.push(...items)
      if (this.items.length > MAX_ITEMS) {
        this.items.splice(0, this.items.length - MAX_ITEMS)
      }
    },
    clear() {
      this.items = []
    },
  },
})
