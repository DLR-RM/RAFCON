<script setup lang="ts">
import ConnectionBadge from '../toolbar/ConnectionBadge.vue'
import LogConsole from '../panels/LogConsole.vue'
import ExecutionHistoryPanel from '../panels/ExecutionHistoryPanel.vue'
import GlobalVariablesPanel from '../panels/GlobalVariablesPanel.vue'
import type { DetachablePanel } from '@/services/windowMode'

const props = defineProps<{ panel: DetachablePanel }>()

const titles: Record<DetachablePanel, string> = {
  logs: 'Logs',
  history: 'Execution history',
  globals: 'Global variables',
}
</script>

<template>
  <div class="flex h-full flex-col">
    <header
      class="flex h-9 shrink-0 items-center gap-3 border-b border-surface-700 bg-surface-900 px-3"
    >
      <span class="text-xs font-semibold tracking-widest text-ink-100">
        RAFCON<span class="ml-1 font-normal text-ink-500">{{ titles[props.panel] }}</span>
      </span>
      <ConnectionBadge class="ml-auto" />
    </header>
    <div class="min-h-0 flex-1 overflow-hidden bg-surface-950">
      <LogConsole v-if="props.panel === 'logs'" />
      <ExecutionHistoryPanel v-else-if="props.panel === 'history'" />
      <GlobalVariablesPanel v-else />
    </div>
  </div>
</template>
