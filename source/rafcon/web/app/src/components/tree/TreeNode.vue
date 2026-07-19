<script setup lang="ts">
import { computed, ref } from 'vue'
import type { StateJson } from '@/services/protocol'
import { useStateMachinesStore } from '@/stores/statemachines'
import { zoomToStatePath } from '@/canvas/bridge'

const props = defineProps<{ state: StateJson; depth: number }>()

const machines = useStateMachinesStore()
const expanded = ref(props.depth < 2)

const children = computed<StateJson[]>(() => {
  if (props.state.states) return Object.values(props.state.states)
  if (props.state.state_copy?.states) return Object.values(props.state.state_copy.states)
  return []
})

const status = computed(() => {
  const smId = machines.selectedSmId
  if (smId === null) return undefined
  return machines.statuses[smId]?.[props.state.path]
})

const isSelected = computed(() => machines.selection?.path === props.state.path)

const typeIcon = computed(
  () =>
    ({
      ExecutionState: '⚙',
      HierarchyState: '☰',
      BarrierConcurrencyState: '∥',
      PreemptiveConcurrencyState: '⚡',
      LibraryState: '📚',
    })[props.state.type] ?? '?',
)

const statusColor = computed(() => {
  if (!status.value) return null
  if (status.value === 'WAIT_FOR_NEXT_STATE') return 'bg-wait-400'
  return 'bg-run-400'
})

function select(): void {
  if (machines.selectedSmId !== null) {
    machines.select(machines.selectedSmId, props.state.path)
  }
}

function focus(): void {
  select()
  zoomToStatePath(props.state.path)
}
</script>

<template>
  <div>
    <div
      class="flex cursor-pointer items-center gap-1 rounded px-1 py-0.5 text-sm"
      :class="isSelected ? 'bg-surface-600 text-ink-100' : 'text-ink-300 hover:bg-surface-800'"
      :style="{ paddingLeft: `${depth * 12 + 4}px` }"
      @click="select"
      @dblclick="focus"
    >
      <button
        v-if="children.length"
        class="w-3 text-xs text-ink-500"
        @click.stop="expanded = !expanded"
      >
        {{ expanded ? '▾' : '▸' }}
      </button>
      <span v-else class="w-3"></span>
      <span class="text-xs opacity-70">{{ typeIcon }}</span>
      <span class="truncate">{{ state.name }}</span>
      <span v-if="statusColor" class="ml-auto h-2 w-2 shrink-0 rounded-full" :class="statusColor"></span>
    </div>
    <template v-if="expanded">
      <TreeNode v-for="child in children" :key="child.state_id" :state="child" :depth="depth + 1" />
    </template>
  </div>
</template>
