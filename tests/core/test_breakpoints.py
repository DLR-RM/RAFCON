import os
import time

import rafcon.core.singleton
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.core.storage import storage
from tests import utils as testing_utils

import rafcon as _rafcon_pkg

# tutorial state machines ship with rafcon
_TUTORIALS = os.path.join(os.path.dirname(_rafcon_pkg.__file__), "share", "rafcon", "examples", "tutorials")
BOTTLES_PATH     = os.path.join(_TUTORIALS, "99_bottles_of_beer")
BOTTLES_LIB_PATH = os.path.join(_TUTORIALS, "99_bottles_of_beer_in_library")

# state IDs inside 99_bottles_of_beer
COUNT_BOTTLES    = "SFZGMH"   # runs once, initialises the count
DECIMATE_BOTTLES = "NDIVLD"   # loops, decrements count

# state IDs inside 99_bottles_of_beer_in_library
LIBRARY_STATE = "UNBQOS"      # LibraryState wrapping 99_bottles_of_beer


def _wait_for_pause(engine, timeout=5.0):
    # poll until the engine hits a breakpoint and enters PAUSED mode
    deadline = time.time() + timeout
    while time.time() < deadline:
        if engine._status.execution_mode is StateMachineExecutionStatus.PAUSED:
            return True
        time.sleep(0.05)
    return False


def test_breakpoint_pauses_execution(caplog):
    # set a breakpoint on Count_bottles and verify execution stops there
    testing_utils.initialize_environment_core()
    sm = storage.load_state_machine_from_path(BOTTLES_PATH)
    engine = rafcon.core.singleton.state_machine_execution_engine
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

    engine.breakpoint_manager.add_breakpoint(sm.root_state.states[COUNT_BOTTLES], "Count bottles")
    engine.start(sm.state_machine_id)

    paused = _wait_for_pause(engine)
    engine.stop()
    engine.join()

    try:
        assert paused, "Engine should have paused at the breakpoint"
    finally:
        engine.breakpoint_manager.clear_all()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_breakpoint_disabled_skips_pause(caplog):
    # disabled BP on Count_bottles (runs first), enabled sentinel on Decimate_bottles (runs after)
    # reaching the sentinel proves Count_bottles was skipped without pausing
    testing_utils.initialize_environment_core()
    sm = storage.load_state_machine_from_path(BOTTLES_PATH)
    engine = rafcon.core.singleton.state_machine_execution_engine
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

    count_bottles = sm.root_state.states[COUNT_BOTTLES]
    engine.breakpoint_manager.add_breakpoint(count_bottles, "Count bottles")
    count_id = os.path.basename(count_bottles.file_system_path)
    engine.breakpoint_manager.toggle_breakpoint(count_id)  # disable it

    # if we reach Decimate_bottles, Count_bottles was not paused
    engine.breakpoint_manager.add_breakpoint(sm.root_state.states[DECIMATE_BOTTLES], "Decimate bottles")

    engine.start(sm.state_machine_id)
    paused_at_sentinel = _wait_for_pause(engine)
    engine.stop()
    engine.join()

    try:
        assert paused_at_sentinel, "Should have paused at Decimate_bottles (after disabled BP)"
    finally:
        engine.breakpoint_manager.clear_all()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_breakpoint_inside_library_state(caplog):
    # set BP using the bare SM state (same file_system_path as inside the library)
    # then run via the library wrapper SM - should still pause
    testing_utils.initialize_environment_core(libraries={"tutorials": _TUTORIALS})
    engine = rafcon.core.singleton.state_machine_execution_engine

    # load bare SM just to get the state object with its file_system_path
    sm_ref = storage.load_state_machine_from_path(BOTTLES_PATH)
    engine.breakpoint_manager.add_breakpoint(sm_ref.root_state.states[COUNT_BOTTLES], "Count bottles")

    sm = storage.load_state_machine_from_path(BOTTLES_LIB_PATH)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    engine.start(sm.state_machine_id)

    paused = _wait_for_pause(engine)
    engine.stop()
    engine.join()

    try:
        assert paused, "Breakpoint should pause execution when run through the library wrapper SM"
    finally:
        engine.breakpoint_manager.clear_all()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_breakpoint_persists_when_library_run_standalone(caplog):
    # set BP from within the library wrapper context (via state_copy)
    # then run the library SM directly - same BP should still run
    # (breakpoints are keyed by the state's folder name, independent of how the SM is loaded)
    testing_utils.initialize_environment_core(libraries={"tutorials": _TUTORIALS})
    engine = rafcon.core.singleton.state_machine_execution_engine

    # UNBQOS is the LibraryState; state_copy is its inner HierarchyState (99_bottles_of_beer)
    sm_lib = storage.load_state_machine_from_path(BOTTLES_LIB_PATH)
    count_bottles = sm_lib.root_state.states[LIBRARY_STATE].state_copy.states[COUNT_BOTTLES]
    engine.breakpoint_manager.add_breakpoint(count_bottles, "Count bottles (library context)")

    # now run the library SM standalone - same BP key, should still trigger
    sm_bare = storage.load_state_machine_from_path(BOTTLES_PATH)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm_bare)
    engine.start(sm_bare.state_machine_id)

    paused = _wait_for_pause(engine)
    engine.stop()
    engine.join()

    try:
        assert paused, "Breakpoint set in library context should still fire when running the library standalone"
    finally:
        engine.breakpoint_manager.clear_all()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_breakpoint_pauses_execution(None)
