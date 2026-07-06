"""
.. module:: protocol
   :synopsis: Message format shared between the RAFCON core server and remote GUI clients

All messages are JSON objects sent as one websocket text frame with the envelope
``{"type": <str>, "seq": <int>, "payload": {...}}``.

Client -> server message types: ``hello``, ``execution_command``, ``open_state_machine``
Server -> client message types: ``welcome``, ``sync``, ``execution_status_changed``,
``state_execution_status_changed``, ``state_machine_added``, ``state_machine_removed``, ``error``
"""

import json

PROTOCOL_VERSION = 1

DEFAULT_PORT = 9999

# client -> server
HELLO = "hello"
EXECUTION_COMMAND = "execution_command"
OPEN_STATE_MACHINE = "open_state_machine"

# server -> client
WELCOME = "welcome"
SYNC = "sync"
EXECUTION_STATUS_CHANGED = "execution_status_changed"
STATE_EXECUTION_STATUS_CHANGED = "state_execution_status_changed"
STATE_MACHINE_ADDED = "state_machine_added"
STATE_MACHINE_REMOVED = "state_machine_removed"
ERROR = "error"

# commands allowed in EXECUTION_COMMAND payloads; maps 1:1 to ExecutionEngine methods
EXECUTION_COMMANDS = ("start", "pause", "stop", "step_mode", "step_into", "step_over", "step_out",
                      "backward_step", "run_to_selected_state", "run_selected_state", "run_only_selected_state")


def make_message(message_type, payload=None, seq=0):
    return json.dumps({"type": message_type, "seq": seq, "payload": payload or {}})


def parse_message(data):
    """Parse a raw websocket frame into a message dict

    :param data: the raw text frame
    :return: dict with keys ``type``, ``seq`` and ``payload``
    :raises ValueError: if the frame is no valid message envelope
    """
    message = json.loads(data)
    if not isinstance(message, dict) or "type" not in message:
        raise ValueError("Invalid message envelope: {0}".format(data))
    message.setdefault("seq", 0)
    message.setdefault("payload", {})
    return message
