"""
.. module:: core_observer
   :synopsis: Observes the core singletons and feeds change events to the network server

Uses the lightweight :meth:`WrapperBase.add_observer` hook of the design_patterns observer module,
which works without any GUI model classes and thus in a headless core process.
"""

import threading

from rafcon.network import protocol

from rafcon.utils import log
logger = log.get_logger(__name__)

# batching window for per-state execution status broadcasts
STATUS_BATCH_INTERVAL = 0.05


class CoreObserver:
    """Observes execution engine, state machine manager and all states of all open state machines

    :param broadcast: callable taking a message dict, called from arbitrary threads
    """

    GVM_OBSERVED_METHODS = ("set_variable", "delete_variable", "lock_variable", "unlock_variable",
                            "set_locked_variable")

    def __init__(self, broadcast):
        self._broadcast = broadcast
        self._lock = threading.Lock()
        self._pending_state_status = {}  # (sm_id, state_path) -> status name
        self._flush_timer = None
        self._observed_states = []

        import rafcon.core.singleton as core_singletons
        self._execution_engine = core_singletons.state_machine_execution_engine
        self._state_machine_manager = core_singletons.state_machine_manager
        self._global_variable_manager = core_singletons.global_variable_manager
        self._gvm_flush_timer = None

        self._execution_engine.add_observer(self, "set_execution_mode", None, self._on_execution_mode_set)
        self._state_machine_manager.add_observer(self, "add_state_machine", None, self._on_state_machine_added)
        self._state_machine_manager.add_observer(self, "remove_state_machine", None, self._on_state_machine_removed)
        for method_name in self.GVM_OBSERVED_METHODS:
            self._global_variable_manager.add_observer(self, method_name, None, self._on_global_variables_changed)

        for state_machine in self._state_machine_manager.state_machines.values():
            self._observe_state_machine(state_machine)

    def shutdown(self):
        with self._lock:
            if self._flush_timer:
                self._flush_timer.cancel()
                self._flush_timer = None
            if self._gvm_flush_timer:
                self._gvm_flush_timer.cancel()
                self._gvm_flush_timer = None
        self._remove_observer(self._execution_engine, "set_execution_mode")
        self._remove_observer(self._state_machine_manager, "add_state_machine")
        self._remove_observer(self._state_machine_manager, "remove_state_machine")
        for method_name in self.GVM_OBSERVED_METHODS:
            self._remove_observer(self._global_variable_manager, method_name)
        for state in self._observed_states:
            self._remove_observer(state, "state_execution_status")
        self._observed_states = []

    def _remove_observer(self, observable, observable_name):
        observable._observers.pop((self, observable_name), None)

    def _observe_state_machine(self, state_machine):
        for state in self._iterate_states(state_machine.root_state):
            state.add_observer(self, "state_execution_status", None, self._on_state_execution_status_set)
            self._observed_states.append(state)

    @staticmethod
    def _iterate_states(state):
        yield state
        # descend into library content so states inside LibraryStates are observed too
        state_copy = getattr(state, "state_copy", None)
        if state_copy is not None:
            yield from CoreObserver._iterate_states(state_copy)
        for child_state in getattr(state, "states", {}).values():
            yield from CoreObserver._iterate_states(child_state)

    def current_execution_status_name(self):
        return self._execution_engine.status.execution_mode.name

    def collect_active_state_statuses(self, state_machine):
        """Return the current non-INACTIVE state statuses of a state machine for the initial sync"""
        from rafcon.core.states.state import StateExecutionStatus
        statuses = []
        for state in self._iterate_states(state_machine.root_state):
            if state.state_execution_status is not StateExecutionStatus.INACTIVE:
                statuses.append({"state_path": state.get_path(),
                                 "status": state.state_execution_status.name})
        return statuses

    # observer callbacks (called from core/execution threads)

    def _on_execution_mode_set(self, instance, result, args):
        self._broadcast({"type": protocol.EXECUTION_STATUS_CHANGED,
                         "payload": {"status": self.current_execution_status_name()}})

    def _on_state_machine_added(self, instance, result, args):
        from rafcon.network import mirror, web_serializer
        state_machine = args[1]
        self._observe_state_machine(state_machine)
        payload = {"state_machine_id": state_machine.state_machine_id,
                   "path": state_machine.file_system_path}
        try:
            payload["sm_zip_b64"] = mirror.pack_state_machine(state_machine)
        except Exception:
            logger.exception("Could not pack state machine {0} for broadcast".format(state_machine.state_machine_id))
            return
        try:
            payload["sm_json"] = web_serializer.state_machine_to_web_dict(state_machine)
        except Exception:
            logger.exception("Could not serialize state machine {0} for web clients"
                             "".format(state_machine.state_machine_id))
        self._broadcast({"type": protocol.STATE_MACHINE_ADDED, "payload": payload})

    def _on_state_machine_removed(self, instance, result, args):
        state_machine_id = args[1]
        self._broadcast({"type": protocol.STATE_MACHINE_REMOVED,
                         "payload": {"state_machine_id": state_machine_id}})

    def _on_state_execution_status_set(self, state, result, args):
        state_machine = state.get_state_machine()
        if state_machine is None:
            return
        key = (state_machine.state_machine_id, state.get_path())
        with self._lock:
            self._pending_state_status[key] = args[1].name
            if self._flush_timer is None:
                self._flush_timer = threading.Timer(STATUS_BATCH_INTERVAL, self._flush_state_statuses)
                self._flush_timer.daemon = True
                self._flush_timer.start()

    def _on_global_variables_changed(self, instance, result, args):
        """Batched like state statuses; the payload always carries a full snapshot"""
        with self._lock:
            if self._gvm_flush_timer is None:
                self._gvm_flush_timer = threading.Timer(STATUS_BATCH_INTERVAL, self._flush_global_variables)
                self._gvm_flush_timer.daemon = True
                self._gvm_flush_timer.start()

    def _flush_global_variables(self):
        from rafcon.network.server import _global_variables_snapshot
        with self._lock:
            self._gvm_flush_timer = None
        self._broadcast({"type": protocol.GLOBAL_VARIABLES_CHANGED,
                         "payload": {"variables": _global_variables_snapshot()},
                         "flavor": protocol.FLAVOR_WEB})

    def _flush_state_statuses(self):
        with self._lock:
            pending = self._pending_state_status
            self._pending_state_status = {}
            self._flush_timer = None
        for (state_machine_id, state_path), status_name in pending.items():
            self._broadcast({"type": protocol.STATE_EXECUTION_STATUS_CHANGED,
                             "payload": {"state_machine_id": state_machine_id,
                                         "state_path": state_path,
                                         "status": status_name}})
