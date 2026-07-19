<script setup lang="ts">
import { computed } from 'vue'
import { useConnectionStore } from '@/stores/connection'

const connection = useConnectionStore()

const color = computed(
  () =>
    ({
      connected: 'bg-run-400',
      connecting: 'bg-warn-400',
      disconnected: 'bg-err-400',
    })[connection.state],
)
</script>

<template>
  <div class="flex items-center gap-1.5 text-xs text-ink-300" :title="connection.lastError ?? ''">
    <span class="inline-block h-2 w-2 rounded-full" :class="color"></span>
    {{ connection.state }}
  </div>
</template>
