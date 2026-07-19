<script setup lang="ts">
import { ref } from 'vue'
import LogConsole from './LogConsole.vue'
import ExecutionHistoryPanel from './ExecutionHistoryPanel.vue'
import GlobalVariablesPanel from './GlobalVariablesPanel.vue'
import { openPanelWindow } from '@/services/windowMode'

const tabs = [
  { id: 'logs', label: 'Logs' },
  { id: 'history', label: 'Execution history' },
  { id: 'globals', label: 'Global variables' },
] as const

const activeTab = ref<(typeof tabs)[number]['id']>('logs')
const collapsed = ref(false)
</script>

<template>
  <section class="shrink-0 border-t border-surface-700 bg-surface-900">
    <div class="flex h-8 items-center gap-1 px-2">
      <div
        v-for="tab in tabs"
        :key="tab.id"
        class="group flex items-center rounded-md transition-colors"
        :class="
          activeTab === tab.id && !collapsed
            ? 'bg-surface-700 text-ink-100'
            : 'text-ink-500 hover:text-ink-300'
        "
      >
        <button class="px-2 py-0.5 text-xs" @click="((activeTab = tab.id), (collapsed = false))">
          {{ tab.label }}
        </button>
        <button
          class="hidden pr-1.5 text-ink-500 hover:text-accent-400 group-hover:block"
          :title="`Open ${tab.label} in its own window`"
          @click.stop="openPanelWindow(tab.id)"
        >
          <svg viewBox="0 0 14 14" class="h-3 w-3 fill-none stroke-current" stroke-width="1.4">
            <path d="M5.5 2.5h-3v9h9v-3" />
            <path d="M8 2h4v4M12 2L7 7" />
          </svg>
        </button>
      </div>
      <button
        class="ml-auto px-2 text-xs text-ink-500 hover:text-ink-300"
        @click="collapsed = !collapsed"
      >
        {{ collapsed ? '▴' : '▾' }}
      </button>
    </div>
    <div v-if="!collapsed" class="h-48 overflow-hidden">
      <LogConsole v-if="activeTab === 'logs'" />
      <ExecutionHistoryPanel v-else-if="activeTab === 'history'" />
      <GlobalVariablesPanel v-else />
    </div>
  </section>
</template>
