<script setup lang="ts">
import { computed } from 'vue'
import { useStateMachinesStore } from '@/stores/statemachines'
import PortTable from './PortTable.vue'

const machines = useStateMachinesStore()
const state = computed(() => machines.selectedState)
const status = computed(() => {
  if (!machines.selection) return null
  return machines.statuses[machines.selection.smId]?.[machines.selection.path] ?? null
})
</script>

<template>
  <div class="flex flex-col gap-3 p-3 text-sm">
    <h2 class="text-xs font-semibold uppercase tracking-wider text-ink-500">Inspector</h2>
    <p v-if="!state" class="text-xs text-ink-500">Select a state to inspect it</p>
    <template v-else>
      <div>
        <div class="text-base font-semibold text-ink-100">{{ state.name }}</div>
        <div class="text-xs text-ink-500">{{ state.type }}</div>
        <div class="mt-1 break-all font-mono text-[11px] text-ink-500">{{ state.path }}</div>
        <div v-if="status" class="mt-1 inline-block rounded bg-surface-700 px-1.5 py-0.5 text-xs">
          {{ status }}
        </div>
      </div>

      <div v-if="state.type === 'LibraryState'" class="text-xs text-ink-300">
        <span class="text-ink-500">Library:</span> {{ state.library_path }}/{{ state.library_name }}
      </div>

      <div v-if="state.description">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Description</h3>
        <p class="whitespace-pre-wrap text-xs text-ink-300">{{ state.description }}</p>
      </div>

      <div v-if="state.outcomes.length">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Outcomes</h3>
        <table class="w-full text-xs">
          <tbody>
            <tr v-for="outcome in state.outcomes" :key="outcome.outcome_id" class="border-b border-surface-800">
              <td class="py-0.5 pr-2 font-mono text-ink-500">{{ outcome.outcome_id }}</td>
              <td class="py-0.5 text-ink-300">{{ outcome.name }}</td>
            </tr>
          </tbody>
        </table>
      </div>

      <div v-if="state.input_data_ports.length">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Input ports</h3>
        <PortTable :ports="state.input_data_ports" />
      </div>
      <div v-if="state.output_data_ports.length">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Output ports</h3>
        <PortTable :ports="state.output_data_ports" />
      </div>
      <div v-if="state.scoped_variables?.length">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Scoped variables</h3>
        <table class="w-full text-xs">
          <tbody>
            <tr
              v-for="variable in state.scoped_variables"
              :key="variable.scoped_variable_id"
              class="border-b border-surface-800"
            >
              <td class="py-0.5 pr-2 text-ink-300">{{ variable.name }}</td>
              <td class="py-0.5 pr-2 font-mono text-ink-500">{{ variable.data_type }}</td>
              <td class="py-0.5 font-mono text-ink-500">{{ variable.default_value }}</td>
            </tr>
          </tbody>
        </table>
      </div>

      <div v-if="state.script_text">
        <h3 class="mb-1 text-xs font-semibold text-ink-500">Script</h3>
        <pre
          class="max-h-96 overflow-auto rounded bg-surface-950 p-2 font-mono text-[11px] leading-snug text-ink-300"
          >{{ state.script_text }}</pre
        >
      </div>
    </template>
  </div>
</template>
