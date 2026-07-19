// Websocket connection to the RAFCON core: handshake, reconnect, message dispatch.

import * as protocol from './protocol'
import type { Message } from './protocol'

export type ConnectionState = 'connecting' | 'connected' | 'disconnected'

export interface WsCallbacks {
  onStateChange: (state: ConnectionState) => void
  onMessage: (message: Message) => void
}

const RECONNECT_MIN_MS = 1000
const RECONNECT_MAX_MS = 15000

async function resolveUrl(): Promise<string> {
  const override = new URLSearchParams(window.location.search).get('ws')
  if (override) return override
  const envUrl = import.meta.env.VITE_WS_URL as string | undefined
  if (envUrl) return envUrl
  try {
    const response = await fetch('config.json')
    if (response.ok) {
      const config = await response.json()
      if (config.websocket_port) {
        return `ws://${window.location.hostname}:${config.websocket_port}`
      }
    }
  } catch {
    // fall through to default
  }
  return `ws://${window.location.hostname}:${protocol.DEFAULT_WS_PORT}`
}

export class RafconWebSocket {
  private socket: WebSocket | null = null
  private reconnectDelay = RECONNECT_MIN_MS
  private stopped = false

  constructor(private callbacks: WsCallbacks) {}

  async start(): Promise<void> {
    const url = await resolveUrl()
    this.connect(url)
  }

  stop(): void {
    this.stopped = true
    this.socket?.close()
  }

  send(type: string, payload: Record<string, unknown> = {}): void {
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.socket.send(protocol.makeMessage(type, payload))
    }
  }

  private connect(url: string): void {
    if (this.stopped) return
    this.callbacks.onStateChange('connecting')
    const socket = new WebSocket(url)
    this.socket = socket

    socket.onopen = () => {
      this.reconnectDelay = RECONNECT_MIN_MS
      socket.send(
        protocol.makeMessage(protocol.HELLO, {
          client_name: 'rafcon-web',
          protocol_version: protocol.PROTOCOL_VERSION,
          flavor: 'web',
        }),
      )
    }

    socket.onmessage = (event) => {
      let message: Message
      try {
        message = protocol.parseMessage(event.data)
      } catch (error) {
        console.error('Invalid message from server', error)
        return
      }
      if (message.type === protocol.WELCOME) {
        this.callbacks.onStateChange('connected')
        return
      }
      this.callbacks.onMessage(message)
    }

    socket.onclose = () => {
      this.callbacks.onStateChange('disconnected')
      if (!this.stopped) {
        setTimeout(() => this.connect(url), this.reconnectDelay)
        this.reconnectDelay = Math.min(this.reconnectDelay * 2, RECONNECT_MAX_MS)
      }
    }

    socket.onerror = () => {
      socket.close()
    }
  }
}
