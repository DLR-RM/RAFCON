import { beforeEach, describe, expect, it } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { useLibrariesStore } from '../libraries'

describe('libraries store', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('starts empty and applies a sync tree', () => {
    const store = useLibrariesStore()
    expect(store.isEmpty).toBe(true)

    store.setTree({
      generic: { wait: '/lib/generic/wait', dialog: { show: '/lib/generic/dialog/show' } },
    })
    expect(store.isEmpty).toBe(false)
    expect((store.tree.generic as Record<string, unknown>).wait).toBe('/lib/generic/wait')
  })

  it('resets selection when a new tree arrives', () => {
    const store = useLibrariesStore()
    store.setTree({ generic: { wait: '/lib/generic/wait' } })
    store.select('/lib/generic/wait')
    expect(store.selectedPath).toBe('/lib/generic/wait')

    store.setTree({})
    expect(store.selectedPath).toBeNull()
    expect(store.isEmpty).toBe(true)
  })

  it('treats an undefined sync payload as empty', () => {
    const store = useLibrariesStore()
    store.setTree({ generic: {} })
    store.setTree(undefined)
    expect(store.isEmpty).toBe(true)
  })

  it('ignores openStateMachine with a blank path', () => {
    const store = useLibrariesStore()
    // no socket in tests — must not throw either way
    expect(() => store.openStateMachine('   ')).not.toThrow()
    expect(() => store.openStateMachine('/some/path')).not.toThrow()
  })
})
