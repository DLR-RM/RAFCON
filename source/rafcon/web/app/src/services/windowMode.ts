// Multi-window support: each browser window is a full, independent websocket client.
// The URL decides what a window shows:
//   (no params)      -> main window, full layout
//   ?panel=logs      -> single detached panel (logs | history | globals)
//   ?sm=<id>         -> full layout pinned to one state machine
// A ?ws= override (see ws.ts) is preserved when opening child windows.

export type DetachablePanel = 'logs' | 'history' | 'globals'

export type WindowMode =
  | { kind: 'main' }
  | { kind: 'panel'; panel: DetachablePanel }
  | { kind: 'sm'; smId: number }

const PANELS: DetachablePanel[] = ['logs', 'history', 'globals']

export function parseWindowMode(search: string): WindowMode {
  const params = new URLSearchParams(search)
  const panel = params.get('panel')
  if (panel && (PANELS as string[]).includes(panel)) {
    return { kind: 'panel', panel: panel as DetachablePanel }
  }
  const sm = params.get('sm')
  if (sm !== null && /^\d+$/.test(sm)) {
    return { kind: 'sm', smId: Number(sm) }
  }
  return { kind: 'main' }
}

export const windowMode: WindowMode = parseWindowMode(
  typeof window !== 'undefined' ? window.location.search : '',
)

export function windowTitle(mode: WindowMode): string {
  if (mode.kind === 'panel') {
    const names: Record<DetachablePanel, string> = {
      logs: 'Logs',
      history: 'Execution history',
      globals: 'Global variables',
    }
    return `RAFCON — ${names[mode.panel]}`
  }
  if (mode.kind === 'sm') return `RAFCON — SM ${mode.smId}`
  return 'RAFCON'
}

function childUrl(params: Record<string, string>): string {
  const url = new URL(window.location.href)
  url.search = ''
  const ws = new URLSearchParams(window.location.search).get('ws')
  if (ws) url.searchParams.set('ws', ws)
  for (const [key, value] of Object.entries(params)) url.searchParams.set(key, value)
  return url.toString()
}

export function openPanelWindow(panel: DetachablePanel): void {
  window.open(childUrl({ panel }), `rafcon-panel-${panel}`, 'popup,width=900,height=420')
}

export function openSmWindow(smId: number): void {
  window.open(childUrl({ sm: String(smId) }), `rafcon-sm-${smId}`, 'popup,width=1400,height=900')
}
