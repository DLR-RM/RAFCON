<script setup lang="ts">
import { computed } from 'vue'
import { useStateMachinesStore } from '@/stores/statemachines'
import { openSmWindow } from '@/services/windowMode'

const machines = useStateMachinesStore()
const pinned = computed(() => machines.pinnedSmId !== null)

function label(path: string | null, smId: number): string {
  if (!path) return `SM ${smId}`
  const parts = path.split('/')
  return parts[parts.length - 1] || `SM ${smId}`
}

const pinnedLabel = computed(() => {
  const smId = machines.pinnedSmId
  if (smId === null) return ''
  return label(machines.machines[smId]?.file_system_path ?? null, smId)
})
</script>

<template>
  <div v-if="pinned" class="flex items-center gap-1.5 overflow-hidden">
    <svg viewBox="0 0 14 14" class="h-3 w-3 shrink-0 fill-accent-400">
      <path d="M8.5 1.5l4 4-2.1.7-2.4 2.4.3 2.9-1.9-1.9L3 13l-.9-.9 3.4-3.4-1.9-1.9 2.9.3L8.9 4.7z" />
    </svg>
    <span class="truncate text-xs text-ink-300" :title="`Pinned to state machine ${machines.pinnedSmId}`">
      {{ pinnedLabel }}
    </span>
  </div>
  <nav v-else class="flex items-center gap-1 overflow-x-auto">
    <div
      v-for="machine in machines.machineList"
      :key="machine.state_machine_id"
      class="group flex items-center whitespace-nowrap rounded-md border transition-colors"
      :class="
        machine.state_machine_id === machines.selectedSmId
          ? 'border-accent-600 bg-surface-700 text-ink-100'
          : 'border-transparent text-ink-500 hover:bg-surface-800 hover:text-ink-300'
      "
    >
      <button class="px-2 py-1 text-xs" @click="machines.selectMachine(machine.state_machine_id)">
        {{ label(machine.file_system_path, machine.state_machine_id) }}
      </button>
      <button
        class="hidden pr-1 text-ink-500 hover:text-accent-400 group-hover:block"
        title="Open this state machine in its own window"
        @click.stop="openSmWindow(machine.state_machine_id)"
      >
        <svg viewBox="0 0 14 14" class="h-3 w-3 fill-none stroke-current" stroke-width="1.4">
          <path d="M5.5 2.5h-3v9h9v-3" />
          <path d="M8 2h4v4M12 2L7 7" />
        </svg>
      </button>
      <button
        class="hidden pr-1.5 text-ink-500 hover:text-err-400 group-hover:block"
        title="Close this state machine on the server (all windows)"
        @click.stop="machines.closeMachine(machine.state_machine_id)"
      >
        <svg viewBox="0 0 14 14" class="h-3 w-3 fill-none stroke-current" stroke-width="1.6">
          <path d="M3.5 3.5l7 7M10.5 3.5l-7 7" />
        </svg>
      </button>
    </div>
  </nav>
</template>
