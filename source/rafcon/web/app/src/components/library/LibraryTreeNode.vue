<script setup lang="ts">
import { computed, ref } from 'vue'
import type { LibraryTreeNode as TreeNodeJson } from '@/services/protocol'
import { useLibrariesStore } from '@/stores/libraries'

const props = defineProps<{ name: string; node: TreeNodeJson | string; depth: number }>()

const libraries = useLibrariesStore()
const expanded = ref(props.depth < 1)

const isLeaf = computed(() => typeof props.node === 'string')
const leafPath = computed(() => (typeof props.node === 'string' ? props.node : null))
const isSelected = computed(() => leafPath.value !== null && libraries.selectedPath === leafPath.value)

const childEntries = computed<[string, TreeNodeJson | string][]>(() =>
  typeof props.node === 'string' ? [] : Object.entries(props.node),
)

function click(): void {
  if (isLeaf.value) {
    libraries.select(leafPath.value)
  } else {
    expanded.value = !expanded.value
  }
}

function doubleClick(): void {
  if (leafPath.value) libraries.openStateMachine(leafPath.value)
}
</script>

<template>
  <div>
    <div
      class="flex cursor-pointer select-none items-center gap-1.5 rounded-md px-1 py-0.5 text-sm transition-colors"
      :class="isSelected ? 'bg-accent-500/20 text-ink-100' : 'text-ink-300 hover:bg-surface-700'"
      :style="{ paddingLeft: `${depth * 12 + 4}px` }"
      :title="leafPath ?? undefined"
      @click="click"
      @dblclick="doubleClick"
    >
      <span v-if="!isLeaf" class="w-3 text-xs text-ink-500">{{ expanded ? '▾' : '▸' }}</span>
      <span v-else class="w-3"></span>
      <svg v-if="!isLeaf" viewBox="0 0 14 14" class="h-3.5 w-3.5 shrink-0 fill-ink-500">
        <path d="M1.5 3.5A1.5 1.5 0 0 1 3 2h2.6l1.3 1.5H11A1.5 1.5 0 0 1 12.5 5v5.5A1.5 1.5 0 0 1 11 12H3a1.5 1.5 0 0 1-1.5-1.5z" />
      </svg>
      <svg v-else viewBox="0 0 14 14" class="h-3.5 w-3.5 shrink-0 fill-accent-400">
        <rect x="2" y="2.5" width="10" height="9" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.4" class="stroke-accent-400" />
        <circle cx="5" cy="7" r="1.1" />
        <circle cx="9" cy="7" r="1.1" />
        <path d="M6.1 7h1.8" stroke="currentColor" stroke-width="1" class="stroke-accent-400" />
      </svg>
      <span class="truncate">{{ name }}</span>
    </div>
    <template v-if="!isLeaf && expanded">
      <LibraryTreeNode
        v-for="[childName, child] in childEntries"
        :key="childName"
        :name="childName"
        :node="child"
        :depth="depth + 1"
      />
    </template>
  </div>
</template>
