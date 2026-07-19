<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useStateMachinesStore } from '@/stores/statemachines'
import { SceneManager } from '@/canvas/scene'
import { registerCanvasBridge } from '@/canvas/bridge'

const machines = useStateMachinesStore()
const canvasRef = ref<HTMLCanvasElement | null>(null)
let scene: SceneManager | null = null

const rootState = computed(() => machines.selectedMachine?.root_state ?? null)
const statuses = computed(() => {
  const smId = machines.selectedSmId
  return smId !== null ? (machines.statuses[smId] ?? {}) : {}
})

onMounted(async () => {
  if (!canvasRef.value) return
  scene = new SceneManager({
    onSelect(path) {
      if (path && machines.selectedSmId !== null) {
        machines.select(machines.selectedSmId, path)
      } else {
        machines.clearSelection()
      }
    },
    onHover() {},
  })
  await scene.init(canvasRef.value)
  scene.setStateMachine(rootState.value)
  scene.setSelection(machines.selection?.path ?? null)
  registerCanvasBridge({
    zoomToStatePath(path) {
      scene?.zoomToPath(path)
    },
  })
})

onBeforeUnmount(() => {
  registerCanvasBridge(null)
  scene?.destroy()
  scene = null
})

watch(rootState, (state) => scene?.setStateMachine(state))
watch(statuses, (value) => scene?.setStatuses({ ...value }), { deep: true })
watch(
  () => machines.selection?.path ?? null,
  (path) => scene?.setSelection(path),
)
</script>

<template>
  <div class="absolute inset-0">
    <canvas ref="canvasRef" class="block h-full w-full"></canvas>
    <div
      v-if="!rootState"
      class="pointer-events-none absolute inset-0 flex items-center justify-center text-sm text-ink-500"
    >
      Waiting for a state machine…
    </div>
  </div>
</template>
