# Copyright (C) 2014-2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Code creation and review were performed with the assistance of generative AI.

import logging
import time

import pytest

from rafcon.core.config import global_config
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.core.execution.execution_history_items import ReturnItem, ConcurrencyItem
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.execution_state import ExecutionState


CAPTURE_LOG_MESSAGES_CONFIG = "CAPTURE_LOG_MESSAGES_IN_EXECUTION_HISTORY"


@pytest.fixture
def execution_history_config():
    """Use in-memory history and restore the global core config after each test."""
    config_keys = (
        CAPTURE_LOG_MESSAGES_CONFIG,
        "IN_MEMORY_EXECUTION_HISTORY_ENABLE",
        "FILE_SYSTEM_EXECUTION_HISTORY_ENABLE",
    )
    old_values = {key: global_config.get_config_value(key) for key in config_keys}

    global_config.set_config_value("IN_MEMORY_EXECUTION_HISTORY_ENABLE", True)
    global_config.set_config_value("FILE_SYSTEM_EXECUTION_HISTORY_ENABLE", False)

    def set_capture_enabled(enabled):
        global_config.set_config_value(CAPTURE_LOG_MESSAGES_CONFIG, enabled)

    try:
        yield set_capture_enabled
    finally:
        for key, value in old_values.items():
            global_config.set_config_value(key, value)


def _set_execution_script(state, script_body):
    """Set the execute function body on an ExecutionState."""
    state.script_text = (
        "def execute(self, inputs, outputs, gvm):\n"
        + "".join("    {}\n".format(line) for line in script_body)
    )


def _execute_state_machine(root_state):
    """Execute a state machine synchronously and return its in-memory history."""
    state_machine = StateMachine(root_state=root_state)
    state_machine.start()
    state_machine.join()

    assert state_machine.execution_histories
    return state_machine.execution_histories[-1]


def _iter_history_items(history):
    """Iterate over history items, including concurrent branch histories."""
    for item in history:
        yield item

        if isinstance(item, ConcurrencyItem):
            for child_history in item.execution_histories:
                yield from _iter_history_items(child_history)


def _return_item_for(history, state):
    """Find the ReturnItem belonging to a particular state."""
    return next(
        item
        for item in _iter_history_items(history)
        if isinstance(item, ReturnItem)
        and item.state_reference is state
    )


def _find_log_record(return_item, message):
    return next(
        (record for record in (return_item.log_messages or [])
         if record["message"] == message),
        None,
    )


def test_log_messages_follow_enable_preference(execution_history_config, caplog):
    """Enabled capture stores log fields; disabling it omits them, even after handler installation."""
    caplog.set_level(logging.DEBUG, logger="rafcon")

    # The enabled run installs the process-wide history handler.
    execution_history_config(True)
    enabled_state = ExecutionState(name="Enabled logging state")
    _set_execution_script(enabled_state, [
        'self.logger.info("history-capture-info")',
        'self.logger.warning("history-capture-warning")',
        'return "success"',
    ])

    start_time = time.time()
    enabled_history = _execute_state_machine(enabled_state)
    end_time = time.time()
    enabled_return = _return_item_for(enabled_history, enabled_state)

    assert enabled_return.log_messages is not None
    info_record = _find_log_record(enabled_return, "history-capture-info")
    warning_record = _find_log_record(enabled_return, "history-capture-warning")
    assert info_record is not None
    assert warning_record is not None
    assert info_record["level"] == "INFO"
    assert warning_record["level"] == "WARNING"
    for record in (info_record, warning_record):
        assert isinstance(record["timestamp"], (int, float))
        assert start_time <= record["timestamp"] <= end_time

    enabled_record_dict = enabled_return.to_dict()
    assert "log_messages" in enabled_record_dict
    assert enabled_record_dict["log_messages"]

    # Disable capture while the handler is already installed by the first run.
    execution_history_config(False)
    disabled_state = ExecutionState(name="Disabled logging state")
    _set_execution_script(disabled_state, [
        'self.logger.warning("history-capture-disabled")',
        'return "success"',
    ])
    disabled_history = _execute_state_machine(disabled_state)
    disabled_return = _return_item_for(disabled_history, disabled_state)

    assert disabled_return.log_messages is None
    assert "log_messages" not in disabled_return.to_dict()


def test_log_messages_are_isolated_between_concurrent_execution_states(execution_history_config, caplog):
    """Concurrent branches capture their own logs and the concurrency state has no log field."""
    caplog.set_level(logging.DEBUG, logger="rafcon")
    execution_history_config(True)

    concurrency_state = BarrierConcurrencyState(name="Parallel logging")
    branch_a = ExecutionState(name="Logging branch A")
    branch_b = ExecutionState(name="Logging branch B")

    # Small waits encourage both branches to overlap during execution.
    _set_execution_script(branch_a, [
        "import time",
        'self.logger.info("branch-a-start")',
        "time.sleep(0.02)",
        'self.logger.info("branch-a-end")',
        'return "success"',
    ])
    _set_execution_script(branch_b, [
        "import time",
        'self.logger.warning("branch-b-start")',
        "time.sleep(0.02)",
        'self.logger.warning("branch-b-end")',
        'return "success"',
    ])

    concurrency_state.add_state(branch_a)
    concurrency_state.add_state(branch_b)
    # Give the decider an immediate script so the test does not wait for the
    # default example script's preemptive_wait().
    decider_state = concurrency_state.states[UNIQUE_DECIDER_STATE_ID]
    _set_execution_script(decider_state, ['return "success"'])
    concurrency_state.add_transition(
        UNIQUE_DECIDER_STATE_ID, 0, concurrency_state.state_id, 0
    )

    history = _execute_state_machine(concurrency_state)

    branch_a_return = _return_item_for(history, branch_a)
    branch_b_return = _return_item_for(history, branch_b)
    concurrency_return = _return_item_for(history, concurrency_state)

    a_messages = [record["message"] for record in (branch_a_return.log_messages or [])]
    b_messages = [record["message"] for record in (branch_b_return.log_messages or [])]

    assert "branch-a-start" in a_messages
    assert "branch-a-end" in a_messages
    assert "branch-b-start" not in a_messages
    assert "branch-b-end" not in a_messages

    assert "branch-b-start" in b_messages
    assert "branch-b-end" in b_messages
    assert "branch-a-start" not in b_messages
    assert "branch-a-end" not in b_messages

    assert _find_log_record(branch_a_return, "branch-a-start")["level"] == "INFO"
    assert _find_log_record(branch_b_return, "branch-b-start")["level"] == "WARNING"

    # A BarrierConcurrencyState is not itself an ExecutionState, so its
    # serialized history item must not contain the optional log field.
    assert concurrency_return.log_messages is None
    assert "log_messages" not in concurrency_return.to_dict()


def test_exception_log_captures_traceback(execution_history_config, caplog):
    """The optional traceback field is preserved for logger.exception() calls."""
    caplog.set_level(logging.DEBUG, logger="rafcon")
    execution_history_config(True)

    state = ExecutionState(name="Exception logging state")
    _set_execution_script(state, [
        "try:",
        '    raise ValueError("captured traceback marker")',
        "except ValueError:",
        '    self.logger.exception("history-capture-exception")',
        'return "success"',
    ])

    history = _execute_state_machine(state)
    return_item = _return_item_for(history, state)
    exception_record = _find_log_record(return_item, "history-capture-exception")

    assert exception_record is not None
    assert exception_record["level"] == "ERROR"
    assert "ValueError: captured traceback marker" in exception_record["exception"]
