"""
.. module:: mirror
   :synopsis: Transfer of whole state machines between core and GUI clients

State machines are shipped as base64-encoded zip archives of their storage folder. This reuses the
complete existing persistence layer (JSON files plus state scripts), so the client-side mirror core
can rebuild an identical state machine with :func:`rafcon.core.storage.storage.load_state_machine_from_path`.
"""

import base64
import io
import os
import tempfile
import zipfile

from rafcon.core.storage import storage
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.execution.execution_status import StateMachineExecutionStatus

from rafcon.utils import log
logger = log.get_logger(__name__)


def pack_state_machine(state_machine):
    """Serialize a state machine into a base64-encoded zip archive of its storage folder

    :param rafcon.core.state_machine.StateMachine state_machine: the state machine to pack
    :return: base64 string of the zipped state machine
    """
    with tempfile.TemporaryDirectory(prefix="rafcon_network_pack_") as temp_dir:
        storage.save_state_machine_to_path(state_machine, temp_dir, as_copy=True)
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
            for root, _, files in os.walk(temp_dir):
                for file_name in files:
                    file_path = os.path.join(root, file_name)
                    archive.write(file_path, os.path.relpath(file_path, temp_dir))
        return base64.b64encode(buffer.getvalue()).decode("ascii")


def unpack_state_machine(sm_zip_b64, state_machine_id=None):
    """Rebuild a state machine from a base64-encoded zip archive

    The archive is extracted into a temporary directory which is kept for the lifetime of the
    process, as the loaded state machine references its script files there.

    :param str sm_zip_b64: base64 string as created by :func:`pack_state_machine`
    :param int state_machine_id: the state machine id to enforce (the id used on the core side)
    :return: the loaded state machine
    """
    temp_dir = tempfile.mkdtemp(prefix="rafcon_network_mirror_")
    buffer = io.BytesIO(base64.b64decode(sm_zip_b64))
    with zipfile.ZipFile(buffer) as archive:
        archive.extractall(temp_dir)
    return storage.load_state_machine_from_path(temp_dir, state_machine_id)


def add_state_machine_to_manager(state_machine_manager, sm_zip_b64, state_machine_id):
    """Unpack a state machine and add it to the given (mirror) state machine manager"""
    if state_machine_id in state_machine_manager.state_machines:
        return state_machine_manager.state_machines[state_machine_id]
    state_machine = unpack_state_machine(sm_zip_b64, state_machine_id)
    state_machine_manager.add_state_machine(state_machine)
    return state_machine


def apply_state_execution_status(state_machine_manager, state_machine_id, state_path, status_name):
    """Set the execution status of a single state in a mirrored state machine

    :param str status_name: name of a :class:`StateExecutionStatus` member
    """
    state_machine = state_machine_manager.get_state_machine(state_machine_id)
    if state_machine is None:
        return
    try:
        state = state_machine.get_state_by_path(state_path)
        state.state_execution_status = StateExecutionStatus[status_name]
    except Exception:
        logger.warning("Could not apply execution status '{0}' to state '{1}' of state machine {2}"
                       "".format(status_name, state_path, state_machine_id))


def apply_execution_status(execution_engine, status_name):
    """Set the execution mode of the mirror execution engine from a remote status name

    This triggers the regular Observable notifications, so all GUI widgets observing the execution
    engine update as if the engine changed its mode locally. ``notify=False`` avoids waking local
    execution threads, of which the mirror has none.
    """
    execution_engine.set_execution_mode(StateMachineExecutionStatus[status_name], notify=False)
