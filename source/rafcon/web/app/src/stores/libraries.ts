import { defineStore } from 'pinia'
import type { LibraryTreeNode } from '@/services/protocol'
import { OPEN_STATE_MACHINE } from '@/services/protocol'
import { getSocket } from '@/services/socket'

export const useLibrariesStore = defineStore('libraries', {
  state: () => ({
    tree: {} as LibraryTreeNode,
    selectedPath: null as string | null,
  }),

  getters: {
    isEmpty(state): boolean {
      return Object.keys(state.tree).length === 0
    },
  },

  actions: {
    setTree(tree: LibraryTreeNode | undefined) {
      this.tree = tree ?? {}
      this.selectedPath = null
    },

    select(path: string | null) {
      this.selectedPath = path
    },

    /** Ask the server to load the state machine at the given OS path */
    openStateMachine(path: string) {
      const trimmed = path.trim()
      if (!trimmed) return
      getSocket()?.send(OPEN_STATE_MACHINE, { path: trimmed })
    },
  },
})
