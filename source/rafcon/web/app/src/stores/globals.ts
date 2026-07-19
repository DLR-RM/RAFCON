import { defineStore } from 'pinia'
import type { GlobalVariableJson } from '@/services/protocol'

export const useGlobalsStore = defineStore('globals', {
  state: () => ({
    variables: [] as GlobalVariableJson[],
  }),
  actions: {
    setVariables(variables: GlobalVariableJson[]) {
      this.variables = variables
    },
  },
})
