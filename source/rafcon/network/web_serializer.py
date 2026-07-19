"""
.. module:: web_serializer
   :synopsis: Serializes a state machine into one nested JSON document for web clients

The GTK remote GUI receives state machines as zipped storage folders (see :mod:`rafcon.network.mirror`).
Browser clients instead receive a single nested JSON document per state machine, produced here by
walking the in-memory state tree and merging in the graphical meta data (``meta_data.json`` files)
read from the state machine's storage folder on disk. Reading meta from disk is acceptable for a
viewer: state machines are not edited in a headless core process, so the files cannot be stale.

The ``path`` entry of every serialized state equals ``state.get_path()``, which is also the path sent
in ``STATE_EXECUTION_STATUS_CHANGED`` messages, so clients can key execution highlighting by it.
"""

import os

from rafcon.utils import log
from rafcon.utils import storage_utils

logger = log.get_logger(__name__)

#: meta scheme used by the gaphas editor; the only one whose coordinates the web client trusts
GAPHAS_EDITOR_KEY = "editor_gaphas"


def state_machine_to_web_dict(state_machine):
    """Serialize a state machine into one nested JSON-compatible dict

    :param rafcon.core.state_machine.StateMachine state_machine: the state machine to serialize
    :return: dict with ``state_machine_id``, ``file_system_path`` and the recursive ``root_state``
    """
    root_state = state_machine.root_state
    meta_dir = _root_state_meta_dir(state_machine)
    return {
        "state_machine_id": state_machine.state_machine_id,
        "file_system_path": state_machine.file_system_path,
        "root_state": state_to_web_dict(root_state, meta_dir),
    }


def state_to_web_dict(state, meta_dir):
    """Recursively serialize a state including its graphical meta data

    :param rafcon.core.states.state.State state: the state to serialize
    :param str meta_dir: storage folder of this state (containing its ``meta_data.json``) or None
    :return: JSON-compatible dict
    """
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.library_state import LibraryState

    meta_file_content = _load_meta_file(meta_dir)

    state_dict = {
        "state_id": state.state_id,
        "name": state.name,
        "type": type(state).__name__,
        "path": state.get_path(),
        "description": state.description,
        "input_data_ports": [_data_port_to_dict(port, meta_file_content, "input_data_port")
                             for port in state.input_data_ports.values()],
        "output_data_ports": [_data_port_to_dict(port, meta_file_content, "output_data_port")
                              for port in state.output_data_ports.values()],
        "outcomes": [_outcome_to_dict(outcome, meta_file_content) for outcome in state.outcomes.values()],
        "meta": _own_state_meta(meta_file_content),
    }

    if isinstance(state, ExecutionState):
        state_dict["script_text"] = state.script_text

    if isinstance(state, LibraryState):
        state_dict["library_path"] = state.library_path
        state_dict["library_name"] = state.library_name
        # inline the library content completely so the viewer can zoom into it; the library's own
        # storage folder carries the meta files of its root state
        state_dict["state_copy"] = state_to_web_dict(state.state_copy, _library_root_meta_dir(state))

    if isinstance(state, ContainerState):
        state_dict["start_state_id"] = state.start_state_id
        state_dict["states"] = {child_id: state_to_web_dict(child, _child_meta_dir(meta_dir, child))
                                for child_id, child in state.states.items()}
        state_dict["transitions"] = [_transition_to_dict(t, meta_file_content)
                                     for t in state.transitions.values()]
        state_dict["data_flows"] = [_data_flow_to_dict(df, meta_file_content)
                                    for df in state.data_flows.values()]
        state_dict["scoped_variables"] = [_scoped_variable_to_dict(sv, meta_file_content)
                                          for sv in state.scoped_variables.values()]

    return state_dict


def _root_state_meta_dir(state_machine):
    """Resolve the storage folder of the root state (``<sm_path>/<root_state_storage_id>``)"""
    sm_path = state_machine.file_system_path
    if not sm_path or not os.path.isdir(sm_path):
        return None
    from rafcon.core.storage import storage
    sm_file = os.path.join(sm_path, storage.STATEMACHINE_FILE)
    try:
        sm_info = storage_utils.load_objects_from_json(sm_file, as_dict=True)
        root_storage_id = sm_info.get("root_state_storage_id") or sm_info.get("root_state_id")
        if root_storage_id:
            return os.path.join(sm_path, root_storage_id)
        # newer storage format: the root state lives directly in the state machine folder
        return sm_path
    except Exception:
        logger.debug("Could not resolve root state storage folder of {0}".format(sm_path))
    return None


def _library_root_meta_dir(library_state):
    """The library's own storage folder contains the meta of the library root state"""
    lib_os_path = getattr(library_state, "lib_os_path", None)
    if not lib_os_path or not os.path.isdir(lib_os_path):
        return None

    class _LibPath(object):
        file_system_path = lib_os_path
    return _root_state_meta_dir(_LibPath())


def _child_meta_dir(meta_dir, child_state):
    if meta_dir is None:
        return None
    from rafcon.core.storage import storage
    return os.path.join(meta_dir, storage.get_storage_id_for_state(child_state))


def _load_meta_file(meta_dir):
    """Load a state's ``meta_data.json`` as a plain dict, or None if unavailable"""
    if meta_dir is None:
        return None
    from rafcon.core.storage import storage
    meta_path = storage.get_meta_data_path(meta_dir)
    if not os.path.isfile(meta_path):
        return None
    try:
        return storage_utils.load_objects_from_json(meta_path, as_dict=True)
    except Exception:
        logger.debug("Could not read meta data file {0}".format(meta_path))
        return None


def _sanitize(value):
    """Convert jsonconversion tuple wrappers into plain lists, recursively"""
    if isinstance(value, dict):
        qualname = value.get("__jsonqualname__", "")
        if qualname.endswith(".tuple") and "items" in value:
            return [_sanitize(item) for item in value["items"]]
        return {key: _sanitize(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(item) for item in value]
    return value


def _gaphas_meta(meta_entry):
    """Extract the ``gui.editor_gaphas`` dict of a meta entry, sanitized, or None"""
    if not isinstance(meta_entry, dict):
        return None
    gaphas = meta_entry.get("gui", {}).get(GAPHAS_EDITOR_KEY)
    if not isinstance(gaphas, dict) or not gaphas:
        return None
    return _sanitize(gaphas)


def _own_state_meta(meta_file_content):
    """The state's own meta (top-level ``gui`` key of its meta file): rel_pos, size, name, income, ..."""
    if not isinstance(meta_file_content, dict):
        return None
    return _gaphas_meta(meta_file_content)


def _element_meta(meta_file_content, element_key):
    """Meta of a child element (``outcome<id>``, ``transition<id>``, ...) from the state's meta file"""
    if not isinstance(meta_file_content, dict):
        return None
    return _gaphas_meta(meta_file_content.get(element_key))


def _rel_pos_of(meta_file_content, element_key):
    meta = _element_meta(meta_file_content, element_key)
    if meta and isinstance(meta.get("rel_pos"), list):
        return meta["rel_pos"]
    return None


def _waypoints_of(meta_file_content, element_key):
    meta = _element_meta(meta_file_content, element_key)
    if meta and isinstance(meta.get("waypoints"), list):
        return meta["waypoints"]
    return []


def _data_port_to_dict(port, meta_file_content, meta_prefix):
    return {
        "data_port_id": port.data_port_id,
        "name": port.name,
        "data_type": getattr(port.data_type, "__name__", str(port.data_type)),
        "default_value": repr(port.default_value),
        "rel_pos": _rel_pos_of(meta_file_content, "{0}{1}".format(meta_prefix, port.data_port_id)),
    }


def _outcome_to_dict(outcome, meta_file_content):
    return {
        "outcome_id": outcome.outcome_id,
        "name": outcome.name,
        "rel_pos": _rel_pos_of(meta_file_content, "outcome{0}".format(outcome.outcome_id)),
    }


def _scoped_variable_to_dict(scoped_variable, meta_file_content):
    return {
        "scoped_variable_id": scoped_variable.data_port_id,
        "name": scoped_variable.name,
        "data_type": getattr(scoped_variable.data_type, "__name__", str(scoped_variable.data_type)),
        "default_value": repr(scoped_variable.default_value),
        "rel_pos": _rel_pos_of(meta_file_content, "scoped_variable{0}".format(scoped_variable.data_port_id)),
    }


def _transition_to_dict(transition, meta_file_content):
    return {
        "transition_id": transition.transition_id,
        "from_state": transition.from_state,
        "from_outcome": transition.from_outcome,
        "to_state": transition.to_state,
        "to_outcome": transition.to_outcome,
        "waypoints": _waypoints_of(meta_file_content, "transition{0}".format(transition.transition_id)),
    }


def _data_flow_to_dict(data_flow, meta_file_content):
    return {
        "data_flow_id": data_flow.data_flow_id,
        "from_state": data_flow.from_state,
        "from_key": data_flow.from_key,
        "to_state": data_flow.to_state,
        "to_key": data_flow.to_key,
        "waypoints": _waypoints_of(meta_file_content, "data_flow{0}".format(data_flow.data_flow_id)),
    }
