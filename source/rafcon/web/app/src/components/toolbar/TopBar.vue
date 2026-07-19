<script setup lang="ts">
import { ref, watch } from 'vue'
import ExecutionControls from './ExecutionControls.vue'
import ConnectionBadge from './ConnectionBadge.vue'
import SmTabs from './SmTabs.vue'
import { useConnectionStore } from '@/stores/connection'

const connection = useConnectionStore()
const toast = ref<string | null>(null)
let toastTimer: ReturnType<typeof setTimeout> | null = null

watch(
  () => connection.lastError,
  (message) => {
    if (!message) return
    toast.value = message
    if (toastTimer) clearTimeout(toastTimer)
    toastTimer = setTimeout(() => (toast.value = null), 6000)
  },
)
</script>

<template>
  <header
    class="relative flex h-12 shrink-0 items-center gap-4 border-b border-surface-700 bg-surface-900 px-3"
  >
    <span class="text-sm font-semibold tracking-widest text-ink-100">
      RAFCON<span class="ml-1 font-normal text-ink-500">web</span>
    </span>
    <ExecutionControls />
    <SmTabs class="min-w-0 flex-1" />
    <ConnectionBadge />
    <transition
      enter-active-class="transition-opacity duration-150"
      leave-active-class="transition-opacity duration-300"
      enter-from-class="opacity-0"
      leave-to-class="opacity-0"
    >
      <div
        v-if="toast"
        class="absolute left-1/2 top-full z-50 mt-2 max-w-xl -translate-x-1/2 rounded-md border border-err-400/50 bg-surface-800 px-3 py-1.5 text-xs text-err-400 shadow-lg"
        @click="toast = null"
      >
        {{ toast }}
      </div>
    </transition>
  </header>
</template>
