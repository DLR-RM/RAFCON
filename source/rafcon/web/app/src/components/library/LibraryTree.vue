<script setup lang="ts">
import { computed, ref } from 'vue'
import { useLibrariesStore } from '@/stores/libraries'
import LibraryTreeNode from './LibraryTreeNode.vue'

const libraries = useLibrariesStore()
const manualPath = ref('')

const rootEntries = computed(() => Object.entries(libraries.tree))

function openSelected(): void {
  if (libraries.selectedPath) libraries.openStateMachine(libraries.selectedPath)
}

function openManual(): void {
  libraries.openStateMachine(manualPath.value)
}
</script>

<template>
  <div class="flex h-full flex-col">
    <div class="min-h-0 flex-1 overflow-y-auto p-2">
      <h2 class="mb-2 px-1 text-xs font-semibold uppercase tracking-wider text-ink-500">Libraries</h2>
      <template v-if="rootEntries.length">
        <LibraryTreeNode
          v-for="[name, node] in rootEntries"
          :key="name"
          :name="name"
          :node="node"
          :depth="0"
        />
      </template>
      <p v-else class="px-1 text-xs text-ink-500">
        No libraries configured — add LIBRARY_PATHS to the core config.
      </p>
    </div>
    <div class="shrink-0 space-y-2 border-t border-surface-700 p-2">
      <button
        class="w-full rounded-md bg-accent-500 px-2 py-1 text-xs font-medium text-white transition-colors hover:bg-accent-400 disabled:cursor-not-allowed disabled:opacity-30"
        :disabled="!libraries.selectedPath"
        title="Open the selected library as a state machine"
        @click="openSelected"
      >
        Open selected library
      </button>
      <form class="flex gap-1" @submit.prevent="openManual">
        <input
          v-model="manualPath"
          type="text"
          placeholder="/path/to/state_machine"
          spellcheck="false"
          class="min-w-0 flex-1 rounded-md border border-surface-700 bg-surface-950 px-2 py-1 font-mono text-xs text-ink-100 placeholder:text-ink-500 focus:border-accent-500 focus:outline-none"
        />
        <button
          type="submit"
          class="rounded-md border border-surface-600 px-2 py-1 text-xs text-ink-300 transition-colors hover:bg-surface-700 disabled:opacity-30"
          :disabled="!manualPath.trim()"
        >
          Open
        </button>
      </form>
    </div>
  </div>
</template>
