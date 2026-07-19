<script setup lang="ts">
import { computed } from 'vue'
import { useStateMachinesStore } from '@/stores/statemachines'
import TreeNode from './TreeNode.vue'

const machines = useStateMachinesStore()
const root = computed(() => machines.selectedMachine?.root_state ?? null)
</script>

<template>
  <div class="p-2">
    <h2 class="mb-2 px-1 text-xs font-semibold uppercase tracking-wider text-ink-500">States</h2>
    <TreeNode v-if="root" :state="root" :depth="0" />
    <p v-else class="px-1 text-xs text-ink-500">No state machine loaded</p>
  </div>
</template>
