// Owns the app-wide websocket instance and dispatches incoming messages to the Pinia stores.

import * as protocol from './protocol'
import type { Message } from './protocol'
import { RafconWebSocket } from './ws'
import { useConnectionStore } from '@/stores/connection'
import { useStateMachinesStore } from '@/stores/statemachines'
import { useExecutionStore } from '@/stores/execution'
import { useGlobalsStore } from '@/stores/globals'
import { useHistoryStore } from '@/stores/history'
import { useLibrariesStore } from '@/stores/libraries'
import { useLogsStore } from '@/stores/logs'
import type {
  ExecutionStatus,
  GlobalVariableJson,
  HistoryItemJson,
  LogRecordJson,
  StateExecutionStatus,
  SyncPayload,
  SyncStateMachineEntry,
} from './protocol'

let socket: RafconWebSocket | null = null

export function getSocket(): RafconWebSocket | null {
  return socket
}

function dispatch(message: Message): void {
  const machines = useStateMachinesStore()
  const execution = useExecutionStore()
  const globals = useGlobalsStore()
  const history = useHistoryStore()
  const libraries = useLibrariesStore()
  const logs = useLogsStore()
  const connection = useConnectionStore()

  switch (message.type) {
    case protocol.SYNC: {
      const payload = message.payload as unknown as SyncPayload
      machines.applySync(payload)
      execution.setStatus(payload.execution_status)
      globals.setVariables(payload.global_variables ?? [])
      logs.setRecords(payload.log_backlog ?? [])
      libraries.setTree(payload.libraries)
      history.clear()
      break
    }
    case protocol.EXECUTION_STATUS_CHANGED:
      execution.setStatus(message.payload.status as ExecutionStatus)
      break
    case protocol.STATE_EXECUTION_STATUS_CHANGED:
      machines.setStateStatus(
        message.payload.state_machine_id as number,
        message.payload.state_path as string,
        message.payload.status as StateExecutionStatus,
      )
      break
    case protocol.STATE_MACHINE_ADDED:
      machines.addMachine(message.payload as unknown as SyncStateMachineEntry)
      break
    case protocol.STATE_MACHINE_REMOVED:
      machines.removeMachine(message.payload.state_machine_id as number)
      break
    case protocol.GLOBAL_VARIABLES_CHANGED:
      globals.setVariables((message.payload.variables ?? []) as GlobalVariableJson[])
      break
    case protocol.EXECUTION_HISTORY_EVENT:
      history.addItems((message.payload.items ?? []) as HistoryItemJson[])
      break
    case protocol.LOG_RECORD:
      logs.addRecords((message.payload.records ?? []) as LogRecordJson[])
      break
    case protocol.ERROR:
      connection.setError(`${message.payload.code}: ${message.payload.message}`)
      logs.addRecords([
        {
          ts: Date.now() / 1000,
          level: 'ERROR',
          logger: 'server',
          message: `${message.payload.code}: ${message.payload.message}`,
        },
      ])
      console.error('Server error', message.payload)
      break
    default:
      console.debug('Unhandled message type', message.type)
  }
}

export function startSocket(): void {
  const connection = useConnectionStore()
  socket = new RafconWebSocket({
    onStateChange: (state) => connection.setState(state),
    onMessage: dispatch,
  })
  void socket.start()
}
