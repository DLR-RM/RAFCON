<script setup lang="ts">
import type { DataPortJson } from '@/services/protocol'

defineProps<{ ports: DataPortJson[] }>()
</script>

<template>
  <table class="w-full text-xs">
    <tbody>
      <tr v-for="port in ports" :key="port.data_port_id" class="border-b border-surface-800">
        <td class="py-0.5 pr-2 text-ink-300">{{ port.name }}</td>
        <td class="py-0.5 pr-2 font-mono text-ink-500">{{ port.data_type }}</td>
        <td class="py-0.5 font-mono text-ink-500">{{ port.default_value }}</td>
      </tr>
    </tbody>
  </table>
</template>
