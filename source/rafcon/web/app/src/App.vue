<script setup lang="ts">
import { computed, ref } from 'vue'
import TopBar from './components/toolbar/TopBar.vue'
import LibraryTree from './components/library/LibraryTree.vue'
import StateMachineTree from './components/tree/StateMachineTree.vue'
import StateInspector from './components/inspector/StateInspector.vue'
import BottomPanel from './components/panels/BottomPanel.vue'
import CanvasView from './components/canvas/CanvasView.vue'
import PanelWindow from './components/layout/PanelWindow.vue'
import { windowMode } from './services/windowMode'
import { useStateMachinesStore } from './stores/statemachines'

const leftTab = ref<'libraries' | 'tree'>(windowMode.kind === 'sm' ? 'tree' : 'libraries')
const machines = useStateMachinesStore()

const pinnedMissing = computed(
  () =>
    windowMode.kind === 'sm' &&
    machines.selectedSmId !== null &&
    !(machines.selectedSmId in machines.machines),
)
</script>

<template>
  <PanelWindow v-if="windowMode.kind === 'panel'" :panel="windowMode.panel" />
  <div v-else class="flex h-full flex-col">
    <TopBar />
    <div class="flex min-h-0 flex-1">
      <aside class="flex w-64 shrink-0 flex-col border-r border-surface-700 bg-surface-900">
        <nav class="flex shrink-0 gap-1 border-b border-surface-700 p-1.5">
          <button
            v-for="tab in [
              { id: 'libraries', label: 'Libraries' },
              { id: 'tree', label: 'States' },
            ] as const"
            :key="tab.id"
            class="flex-1 rounded-md px-2 py-1 text-xs font-medium transition-colors"
            :class="
              leftTab === tab.id
                ? 'bg-surface-700 text-ink-100'
                : 'text-ink-500 hover:bg-surface-800 hover:text-ink-300'
            "
            @click="leftTab = tab.id"
          >
            {{ tab.label }}
          </button>
        </nav>
        <div class="min-h-0 flex-1 overflow-y-auto">
          <LibraryTree v-if="leftTab === 'libraries'" />
          <StateMachineTree v-else />
        </div>
      </aside>
      <main class="relative min-w-0 flex-1 bg-surface-950">
        <CanvasView />
        <div
          v-if="pinnedMissing"
          class="absolute inset-0 z-10 flex items-center justify-center bg-surface-950/80"
        >
          <p class="rounded-md border border-surface-700 bg-surface-900 px-4 py-2 text-sm text-ink-500">
            Waiting for state machine {{ machines.selectedSmId }} — it is not (or no longer) open on
            the server.
          </p>
        </div>
      </main>
      <aside class="w-80 shrink-0 overflow-y-auto border-l border-surface-700 bg-surface-900">
        <StateInspector />
      </aside>
    </div>
    <BottomPanel />
  </div>
</template>
