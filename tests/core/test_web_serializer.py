import json
import os

import pytest

from tests import utils as testing_utils


def _load(sm_path):
    from rafcon.core.storage import storage
    return storage.load_state_machine_from_path(sm_path)


def _walk(state_dict):
    yield state_dict
    for child in (state_dict.get("states") or {}).values():
        yield from _walk(child)
    if state_dict.get("state_copy"):
        yield from _walk(state_dict["state_copy"])


def test_web_serializer_full_meta(caplog):
    testing_utils.initialize_environment_core()
    try:
        from rafcon.network import web_serializer

        sm_path = testing_utils.get_test_sm_path(
            os.path.join("unit_test_state_machines", "backward_step_barrier_test"))
        state_machine = _load(sm_path)
        result = web_serializer.state_machine_to_web_dict(state_machine)

        # the whole document must be JSON serializable (no tuple wrappers left)
        serialized = json.dumps(result)
        assert "__jsonqualname__" not in serialized

        root = result["root_state"]
        assert root["type"] == "HierarchyState"
        assert root["path"] == state_machine.root_state.get_path()
        assert root["meta"] is not None
        assert len(root["meta"]["rel_pos"]) == 2
        assert len(root["meta"]["size"]) == 2

        # every serialized path must equal the core state's get_path() (used as the
        # key for execution status highlighting)
        core_paths = set()

        def collect(state):
            core_paths.add(state.get_path())
            for child in getattr(state, "states", {}).values():
                collect(child)

        collect(state_machine.root_state)
        serialized_paths = {entry["path"] for entry in _walk(root) }
        assert core_paths <= serialized_paths

        # transitions and scoped variables of the root container are serialized
        assert root["transitions"]
        assert root["scoped_variables"]
        assert all("waypoints" in transition for transition in root["transitions"])

        # execution states carry their script
        execution_states = [entry for entry in _walk(root) if entry["type"] == "ExecutionState"]
        assert execution_states
        assert all(entry.get("script_text") for entry in execution_states)
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_web_serializer_deep_libraries(caplog):
    testing_utils.initialize_environment_core()
    try:
        from rafcon.network import web_serializer

        sm_path = testing_utils.get_test_sm_path(
            os.path.join("unit_test_state_machines", "deep_libraries", "sm_with_deep_libraries"))
        state_machine = _load(sm_path)
        result = web_serializer.state_machine_to_web_dict(state_machine)
        json.dumps(result)

        library_states = [entry for entry in _walk(result["root_state"])
                          if entry["type"] == "LibraryState"]
        assert library_states
        # library content is inlined recursively
        assert all(entry.get("state_copy") for entry in library_states)
        # nesting reaches through several library levels
        max_depth = max(entry["path"].count("/") for entry in _walk(result["root_state"]))
        assert max_depth >= 5
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_web_serializer_without_meta(caplog):
    testing_utils.initialize_environment_core()
    try:
        from rafcon.network import web_serializer

        sm_path = testing_utils.get_test_sm_path(
            os.path.join("unit_test_state_machines", "99_bottles_of_beer_monitoring"))
        state_machine = _load(sm_path)
        result = web_serializer.state_machine_to_web_dict(state_machine)
        json.dumps(result)
        # missing meta files must not break serialization; meta is simply None
        assert result["root_state"]["states"]
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    pytest.main([__file__, '-xvs'])
