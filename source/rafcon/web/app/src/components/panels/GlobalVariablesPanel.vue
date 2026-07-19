<script setup lang="ts">
import { useGlobalsStore } from '@/stores/globals'

const globals = useGlobalsStore()
</script>

<template>
  <div class="h-full overflow-y-auto px-2 py-1">
    <p v-if="!globals.variables.length" class="text-xs text-ink-500">No global variables</p>
    <table v-else class="w-full text-xs">
      <thead>
        <tr class="text-left text-[10px] uppercase tracking-wider text-ink-500">
          <th class="py-1 pr-2 font-medium">Key</th>
          <th class="py-1 pr-2 font-medium">Value</th>
          <th class="py-1 pr-2 font-medium">Type</th>
          <th class="py-1 font-medium">Locked</th>
        </tr>
      </thead>
      <tbody>
        <tr v-for="variable in globals.variables" :key="variable.key" class="border-t border-surface-800">
          <td class="py-0.5 pr-2 font-mono text-ink-100">{{ variable.key }}</td>
          <td class="max-w-md truncate py-0.5 pr-2 font-mono text-ink-300">{{ variable.value_repr }}</td>
          <td class="py-0.5 pr-2 font-mono text-ink-500">{{ variable.data_type }}</td>
          <td class="py-0.5">{{ variable.is_locked ? '🔒' : '' }}</td>
        </tr>
      </tbody>
    </table>
  </div>
</template>
