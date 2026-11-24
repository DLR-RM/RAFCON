# Copyright (C) 2014-2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Johannes Ernst <j.ernst@dlr.de>
#
# Code creation and review were performed with the assistance of generative AI.

import contextvars
import logging
from contextlib import contextmanager
from threading import Lock
from functools import wraps
from rafcon.core.config import global_config


_current_state = contextvars.ContextVar(
    "rafcon_current_state",
    default=None,
)

_handler_lock = Lock()
_handler = None


class ExecutionHistoryLogHandler(logging.Handler):
    """Collect RAFCON log records for the currently executing state."""

    def emit(self, record):
        if not is_log_message_capturing_enabled():
            return

        state = _current_state.get()

        if state is None:
            return

        logs = getattr(state, "_log_message_records", None)
        if logs is None:
            return

        entry = {
            "timestamp": record.created,
            "level": record.levelname,
            "message": record.getMessage(),
        }

        if record.exc_info:
            entry["exception"] = logging.Formatter().formatException(
                record.exc_info
            )

        logs.append(entry)


def _install_handler():
    """Install the history handler only once."""
    global _handler

    with _handler_lock:
        if _handler is None:
            _handler = ExecutionHistoryLogHandler()
            logging.getLogger("rafcon").addHandler(_handler)


@contextmanager
def capture_execution_state_logs(state):
    """Collect log records emitted during one state execution."""
    if not is_log_message_capturing_enabled():
        state._log_message_records = None

        # Mask any outer execution-state context.
        token = _current_state.set(None)
        try:
            yield
        finally:
            _current_state.reset(token)

        return

    _install_handler()

    if not state.backward_execution:
        state._log_message_records = []

    token = _current_state.set(state)
    try:
        yield
    finally:
        _current_state.reset(token)

def capture_log_messages_decorator(func):
    @wraps(func)
    def wrapper(state, *args, **kwargs):
        with capture_execution_state_logs(state):
            return func(state, *args, **kwargs)

    return wrapper

def is_log_message_capturing_enabled():
    return global_config.get_config_value(
        "CAPTURE_LOG_MESSAGES_IN_EXECUTION_HISTORY",
        False,
    )
