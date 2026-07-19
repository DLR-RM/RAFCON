import { describe, expect, it } from 'vitest'
import { parseWindowMode, windowTitle } from '../windowMode'

describe('parseWindowMode', () => {
  it('defaults to main', () => {
    expect(parseWindowMode('')).toEqual({ kind: 'main' })
    expect(parseWindowMode('?ws=ws://host:9999')).toEqual({ kind: 'main' })
  })

  it('parses panel windows', () => {
    expect(parseWindowMode('?panel=logs')).toEqual({ kind: 'panel', panel: 'logs' })
    expect(parseWindowMode('?panel=history')).toEqual({ kind: 'panel', panel: 'history' })
    expect(parseWindowMode('?panel=globals')).toEqual({ kind: 'panel', panel: 'globals' })
    expect(parseWindowMode('?panel=bogus')).toEqual({ kind: 'main' })
  })

  it('parses pinned state machine windows', () => {
    expect(parseWindowMode('?sm=3')).toEqual({ kind: 'sm', smId: 3 })
    expect(parseWindowMode('?sm=abc')).toEqual({ kind: 'main' })
    expect(parseWindowMode('?sm=-1')).toEqual({ kind: 'main' })
  })

  it('panel wins over sm when both present', () => {
    expect(parseWindowMode('?panel=logs&sm=2')).toEqual({ kind: 'panel', panel: 'logs' })
  })

  it('builds window titles', () => {
    expect(windowTitle({ kind: 'main' })).toBe('RAFCON')
    expect(windowTitle({ kind: 'panel', panel: 'logs' })).toBe('RAFCON — Logs')
    expect(windowTitle({ kind: 'sm', smId: 2 })).toBe('RAFCON — SM 2')
  })
})
