<script setup lang="ts">
import { nextTick, ref, watch } from 'vue'
import { useLogsStore, type LogLevel } from '@/stores/logs'

const logs = useLogsStore()
const container = ref<HTMLElement | null>(null)
const levels: LogLevel[] = ['VERBOSE', 'DEBUG', 'INFO', 'WARNING', 'ERROR']

const levelColor: Record<string, string> = {
  VERBOSE: 'text-ink-500',
  DEBUG: 'text-ink-500',
  INFO: 'text-ink-300',
  WARNING: 'text-warn-400',
  ERROR: 'text-err-400',
}

watch(
  () => logs.records.length,
  async () => {
    if (logs.followTail && container.value) {
      await nextTick()
      container.value.scrollTop = container.value.scrollHeight
    }
  },
)

function formatTime(ts: number): string {
  return new Date(ts * 1000).toLocaleTimeString()
}

function onScroll(): void {
  const el = container.value
  if (!el) return
  logs.setFollowTail(el.scrollHeight - el.scrollTop - el.clientHeight < 10)
}
</script>

<template>
  <div class="flex h-full flex-col">
    <div class="flex items-center gap-1 border-b border-surface-800 px-2 py-1">
      <button
        v-for="level in levels"
        :key="level"
        class="rounded px-1.5 py-0.5 text-[10px]"
        :class="logs.minLevel === level ? 'bg-surface-700 text-ink-100' : 'text-ink-500'"
        @click="logs.setMinLevel(level)"
      >
        {{ level }}
      </button>
      <span v-if="!logs.followTail" class="ml-auto text-[10px] text-warn-400">scroll paused</span>
    </div>
    <div ref="container" class="flex-1 overflow-y-auto px-2 py-1" @scroll="onScroll">
      <div
        v-for="(record, index) in logs.filtered"
        :key="index"
        class="whitespace-pre-wrap break-all font-mono text-[11px] leading-snug"
      >
        <span class="text-ink-500">{{ formatTime(record.ts) }}</span>
        <span class="mx-1" :class="levelColor[record.level] ?? 'text-ink-300'">{{ record.level }}</span>
        <span class="text-ink-500">{{ record.logger }}</span>
        <span class="mx-1 text-ink-300">{{ record.message }}</span>
      </div>
    </div>
  </div>
</template>
