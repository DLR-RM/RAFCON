<script setup lang="ts">
import { computed } from 'vue'
import { useHistoryStore } from '@/stores/history'
import { useStateMachinesStore } from '@/stores/statemachines'
import type { HistoryItemJson } from '@/services/protocol'

const history = useHistoryStore()
const machines = useStateMachinesStore()

// pinned windows only show the history of their own state machine
const items = computed(() =>
  machines.pinnedSmId === null
    ? history.items
    : history.items.filter(
        (item) => item.state_machine_id === null || item.state_machine_id === machines.pinnedSmId,
      ),
)

const typeIcon: Record<string, string> = {
  StateMachineStartItem: '⚑',
  CallItem: '→',
  ReturnItem: '←',
  ConcurrencyItem: '∥',
}

function selectItem(item: HistoryItemJson): void {
  const smId = item.state_machine_id ?? machines.selectedSmId
  if (smId !== null) machines.select(smId, item.path)
}

function formatTime(ts: number): string {
  return new Date(ts * 1000).toLocaleTimeString()
}
</script>

<template>
  <div class="h-full overflow-y-auto px-2 py-1">
    <p v-if="!items.length" class="text-xs text-ink-500">No execution history yet</p>
    <div
      v-for="item in items"
      :key="item.history_item_id"
      class="flex cursor-pointer items-center gap-2 rounded px-1 py-0.5 font-mono text-[11px] text-ink-300 hover:bg-surface-800"
      :title="item.path_by_name"
      @click="selectItem(item)"
    >
      <span class="text-ink-500">{{ formatTime(item.timestamp) }}</span>
      <span class="w-3 text-accent-400">{{ typeIcon[item.item_type] ?? '·' }}</span>
      <span class="truncate">{{ item.state_name }}</span>
      <span class="truncate text-ink-500">{{ item.item_type }}</span>
      <span v-if="item.outcome_name" class="text-run-400">{{ item.outcome_name }}</span>
    </div>
  </div>
</template>
