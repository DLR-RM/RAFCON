<script setup lang="ts">
import { computed } from 'vue'
import { useExecutionStore } from '@/stores/execution'
import { useStateMachinesStore } from '@/stores/statemachines'
import type { ExecutionCommand } from '@/services/protocol'

const execution = useExecutionStore()
const machines = useStateMachinesStore()

const smId = computed(() => machines.selectedSmId)
const selectionPath = computed(() => machines.selection?.path ?? null)

interface Button {
  command: ExecutionCommand
  svg: string
  title: string
  needsSelection?: boolean
  accent?: boolean
}

const mainButtons: Button[] = [
  {
    command: 'start',
    svg: '<path d="M3.5 2.2v9.6l8.4-4.8z"/>',
    title: 'Start / resume execution',
    accent: true,
  },
  {
    command: 'pause',
    svg: '<rect x="3.2" y="2.5" width="2.6" height="9" rx="0.6"/><rect x="8.2" y="2.5" width="2.6" height="9" rx="0.6"/>',
    title: 'Pause execution',
  },
  {
    command: 'stop',
    svg: '<rect x="3" y="3" width="8" height="8" rx="1"/>',
    title: 'Stop execution',
  },
  {
    command: 'step_mode',
    svg: '<path d="M4.2 1.8v6.4l5.2-3.2z"/><rect x="2" y="10.5" width="10" height="1.7" rx="0.6"/>',
    title: 'Enter step mode',
  },
]

const stepButtons: Button[] = [
  {
    command: 'step_into',
    svg: '<rect x="6.2" y="1.2" width="1.6" height="4.6" rx="0.5"/><path d="M3.8 5.6l3.2 3.6 3.2-3.6z"/><rect x="3" y="10.8" width="8" height="1.6" rx="0.6"/>',
    title: 'Step into',
  },
  {
    command: 'step_over',
    svg: '<path d="M2.3 10.2a4.8 4.8 0 0 1 9.1-1.6" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/><path d="M12.6 5.6l-.2 4-3.4-1.8z"/>',
    title: 'Step over',
  },
  {
    command: 'step_out',
    svg: '<rect x="3" y="1.6" width="8" height="1.6" rx="0.6"/><path d="M3.8 8.4l3.2-3.6 3.2 3.6z"/><rect x="6.2" y="8.2" width="1.6" height="4.6" rx="0.5"/>',
    title: 'Step out',
  },
  {
    command: 'backward_step',
    svg: '<path d="M11.7 10.2a4.8 4.8 0 0 0-9.1-1.6" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/><path d="M1.4 5.6l.2 4 3.4-1.8z"/>',
    title: 'Backward step',
  },
]

const selectionButtons: Button[] = [
  {
    command: 'run_to_selected_state',
    svg: '<rect x="1.5" y="6.2" width="6.4" height="1.6" rx="0.5"/><path d="M7.3 3.6L10.7 7l-3.4 3.4z"/><rect x="11.2" y="2.5" width="1.6" height="9" rx="0.5"/>',
    title: 'Run to selected state',
    needsSelection: true,
  },
  {
    command: 'run_selected_state',
    svg: '<rect x="1.5" y="2.5" width="1.6" height="9" rx="0.5"/><path d="M4.6 2.5v9l7.2-4.5z"/>',
    title: 'Run from selected state',
    needsSelection: true,
  },
  {
    command: 'run_only_selected_state',
    svg: '<circle cx="7" cy="7" r="5.4" fill="none" stroke="currentColor" stroke-width="1.4"/><path d="M5.6 4.4v5.2l4.2-2.6z"/>',
    title: 'Run only selected state',
    needsSelection: true,
  },
]

const inStepMode = computed(() =>
  ['STEP_MODE', 'FORWARD_INTO', 'FORWARD_OVER', 'FORWARD_OUT', 'BACKWARD', 'PAUSED'].includes(
    execution.status,
  ),
)

function isEnabled(button: Button): boolean {
  if (button.needsSelection && !selectionPath.value) return false
  if (stepButtons.includes(button)) return inStepMode.value
  return true
}

function run(button: Button): void {
  execution.sendCommand(button.command, smId.value, button.needsSelection ? selectionPath.value : null)
}

const groups = [mainButtons, stepButtons, selectionButtons]

const statusClass = computed(() => {
  if (execution.isRunning) return 'text-run-400 border-run-400/40'
  if (execution.status === 'PAUSED') return 'text-wait-400 border-wait-400/40'
  return 'text-ink-500 border-surface-600'
})
</script>

<template>
  <div class="flex items-center gap-2">
    <div
      v-for="(group, groupIndex) in groups"
      :key="groupIndex"
      class="flex overflow-hidden rounded-md border border-surface-600 bg-surface-800"
    >
      <button
        v-for="button in group"
        :key="button.command"
        :title="button.title"
        :disabled="!isEnabled(button)"
        class="px-2.5 py-1.5 transition-colors hover:bg-surface-700 disabled:cursor-not-allowed disabled:opacity-25"
        :class="button.accent ? 'text-run-400' : 'text-ink-300'"
        @click="run(button)"
      >
        <!-- eslint-disable-next-line vue/no-v-html — static icon markup defined above -->
        <svg viewBox="0 0 14 14" class="h-3.5 w-3.5 fill-current" v-html="button.svg"></svg>
      </button>
    </div>
    <span
      class="ml-1 rounded-md border bg-surface-800 px-2 py-0.5 font-mono text-[11px] tracking-wide"
      :class="statusClass"
    >
      {{ execution.status }}
    </span>
  </div>
</template>
