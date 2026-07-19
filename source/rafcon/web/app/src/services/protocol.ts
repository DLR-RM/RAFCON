// Mirrors source/rafcon/network/protocol.py and the output of web_serializer.py

export const PROTOCOL_VERSION = 1
export const DEFAULT_WS_PORT = 9999

// client -> server
export const HELLO = 'hello'
export const EXECUTION_COMMAND = 'execution_command'
export const OPEN_STATE_MACHINE = 'open_state_machine'
export const CLOSE_STATE_MACHINE = 'close_state_machine'

// server -> client
export const WELCOME = 'welcome'
export const SYNC = 'sync'
export const EXECUTION_STATUS_CHANGED = 'execution_status_changed'
export const STATE_EXECUTION_STATUS_CHANGED = 'state_execution_status_changed'
export const STATE_MACHINE_ADDED = 'state_machine_added'
export const STATE_MACHINE_REMOVED = 'state_machine_removed'
export const ERROR = 'error'
export const GLOBAL_VARIABLES_CHANGED = 'global_variables_changed'
export const EXECUTION_HISTORY_EVENT = 'execution_history_event'
export const LOG_RECORD = 'log_record'

export type ExecutionCommand =
  | 'start'
  | 'pause'
  | 'stop'
  | 'step_mode'
  | 'step_into'
  | 'step_over'
  | 'step_out'
  | 'backward_step'
  | 'run_to_selected_state'
  | 'run_selected_state'
  | 'run_only_selected_state'

export type ExecutionStatus =
  | 'STARTED'
  | 'STOPPED'
  | 'PAUSED'
  | 'FINISHED'
  | 'STEP_MODE'
  | 'FORWARD_INTO'
  | 'FORWARD_OVER'
  | 'FORWARD_OUT'
  | 'BACKWARD'
  | 'RUN_TO_SELECTED_STATE'
  | 'RUN_SELECTED_STATE'
  | 'RUN_ONLY_SELECTED_STATE'

export type StateExecutionStatus =
  | 'INACTIVE'
  | 'ACTIVE'
  | 'EXECUTE_CHILDREN'
  | 'WAIT_FOR_NEXT_STATE'

export interface Message {
  type: string
  seq: number
  payload: Record<string, unknown>
}

export interface Vec2 extends Array<number> {}

export interface StateMeta {
  rel_pos?: Vec2
  size?: Vec2
  background_color?: number[]
  name?: { rel_pos?: Vec2; size?: Vec2 }
  income?: { rel_pos?: Vec2 }
  [key: string]: unknown
}

export interface DataPortJson {
  data_port_id: number
  name: string
  data_type: string
  default_value: string
  rel_pos: Vec2 | null
}

export interface OutcomeJson {
  outcome_id: number
  name: string
  rel_pos: Vec2 | null
}

export interface ScopedVariableJson {
  scoped_variable_id: number
  name: string
  data_type: string
  default_value: string
  rel_pos: Vec2 | null
}

export interface TransitionJson {
  transition_id: number
  from_state: string | null
  from_outcome: number | null
  to_state: string | null
  to_outcome: number | null
  waypoints: Vec2[]
}

export interface DataFlowJson {
  data_flow_id: number
  from_state: string
  from_key: number
  to_state: string
  to_key: number
  waypoints: Vec2[]
}

export type StateType =
  | 'ExecutionState'
  | 'HierarchyState'
  | 'BarrierConcurrencyState'
  | 'PreemptiveConcurrencyState'
  | 'LibraryState'

export interface StateJson {
  state_id: string
  name: string
  type: StateType
  path: string
  description: string | null
  input_data_ports: DataPortJson[]
  output_data_ports: DataPortJson[]
  outcomes: OutcomeJson[]
  meta: StateMeta | null
  // ExecutionState
  script_text?: string
  // ContainerState
  start_state_id?: string | null
  states?: Record<string, StateJson>
  transitions?: TransitionJson[]
  data_flows?: DataFlowJson[]
  scoped_variables?: ScopedVariableJson[]
  // LibraryState
  library_path?: string
  library_name?: string
  state_copy?: StateJson
}

export interface StateMachineJson {
  state_machine_id: number
  file_system_path: string | null
  root_state: StateJson
}

export interface SyncStateMachineEntry {
  state_machine_id: number
  path: string | null
  sm_json: StateMachineJson
  state_statuses: { state_path: string; status: StateExecutionStatus }[]
}

export interface GlobalVariableJson {
  key: string
  value_repr: string
  data_type: string
  is_locked: boolean
}

export interface LogRecordJson {
  ts: number
  level: string
  logger: string
  message: string
}

export interface HistoryItemJson {
  path: string
  path_by_name: string
  state_type: string
  is_library: boolean
  library_state_name: string | null
  library_name: string | null
  library_path: string | null
  state_name: string
  timestamp: number
  run_id: string
  history_item_id: string
  prev_history_item_id: string | null
  item_type: string
  description: string | null
  state_machine_id: number | null
  call_type?: string
  outcome_name?: string
  outcome_id?: number
  [key: string]: unknown
}

/** Library manager tree: folders are nested objects, leaves are OS paths of library folders */
export interface LibraryTreeNode {
  [name: string]: LibraryTreeNode | string
}

export interface SyncPayload {
  state_machines: SyncStateMachineEntry[]
  execution_status: ExecutionStatus
  active_state_machine_id: number | null
  global_variables: GlobalVariableJson[]
  log_backlog: LogRecordJson[]
  libraries?: LibraryTreeNode
}

export function makeMessage(type: string, payload: Record<string, unknown> = {}, seq = 0): string {
  return JSON.stringify({ type, seq, payload })
}

export function parseMessage(data: string): Message {
  const message = JSON.parse(data)
  if (typeof message !== 'object' || message === null || !('type' in message)) {
    throw new Error(`Invalid message envelope: ${data}`)
  }
  return { type: message.type, seq: message.seq ?? 0, payload: message.payload ?? {} }
}
