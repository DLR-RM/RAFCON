import os

import rafcon.core.singleton
from rafcon.core.execution.execution_history_items import CallItem
from tests import utils as testing_utils

SM_PATH = os.path.join("unit_test_state_machines", "start_from_here_test", "example_demo_state_machine")
LIB_PATH = os.path.join(testing_utils.TEST_STATE_MACHINES_PATH, "start_from_here_test", "develop_test_libraries")
ROOT = "QBYTMD"

# top-level tasks
TASK_1 = ROOT + "/QIZZOB"
TASK_2 = ROOT + "/RYZGLE"

# inside task_1 (LZLXFW is the HierarchyState wrapping the library)
BARRIER_CONCURRENCY    = ROOT + "/QIZZOB/LZLXFW/KEQYNU"   # all branches must finish before moving on
PREEMPTIVE_CONCURRENCY = ROOT + "/QIZZOB/LZLXFW/AESIQF"   # fastest branch cancels the rest

# inside task_2 (EAWGCM is the HierarchyState wrapping the library)
HIERARCHY2_IN_TASK2 = ROOT + "/RYZGLE/EAWGCM/SFSEDE"              # first hierarchy_2 in task_2
NESTED_LIBRARY      = ROOT + "/RYZGLE/EAWGCM/SFSEDE/MQRVHE/BNLTCG/IKAANM"  # hierarchy_4, 3 libs deep


def _replay_from(sm, target_path):
    engine = rafcon.core.singleton.state_machine_execution_engine
    run_1 = sm.execution_histories[0]
    call_item = next(
        (item for item in run_1
         if isinstance(item, CallItem) and item.call_type_str == 'EXECUTE' and item.path == target_path),
        None
    )
    assert call_item is not None, f"No EXECUTE CallItem found at '{target_path}'"
    engine.replay_from_history(call_item, sm)
    engine.start(sm.state_machine_id)
    engine.join()
    engine.stop()


def _output(sm):
    return sm.root_state.states["RYZGLE"].output_data["output_1"]


def test_full_run(caplog):
    # baseline: full run from the beginning
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    try:
        assert len(sm.execution_histories) == 1
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_task1(caplog):
    # start from task_1 - verify both task_1 and task_2 produce correct outputs
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, TASK_1)
    try:
        assert sm.root_state.states["QIZZOB"].output_data["output_1"] == 79
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_task2(caplog):
    # task_1 is skipped, its output is restored from history, task_2 re-runs
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, TASK_2)
    try:
        assert len(sm.execution_histories) == 2
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_barrier_concurrency(caplog):
    # start from inside task_1 at the barrier concurrency 
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, BARRIER_CONCURRENCY)
    try:
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_preemptive_concurrency(caplog):
    # start from the preemptive concurrency inside task_1
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, PREEMPTIVE_CONCURRENCY)
    try:
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_deep_hierarchy(caplog):
    # start from hierarchy_2 inside task_2, skipping task_1 entirely
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, HIERARCHY2_IN_TASK2)
    try:
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_replay_from_nested_library(caplog):
    # IKAANM is hierarchy_4 - a library state three levels deep inside task_2
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, NESTED_LIBRARY)
    try:
        assert _output(sm) == 295
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_multiple_replays_from_run1(caplog):
    # run 1 stays intact - each replay creates a new run, all sourced from run 1
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    _replay_from(sm, TASK_2)
    _replay_from(sm, TASK_1)
    try:
        assert _output(sm) == 295
        assert len(sm.execution_histories) == 3
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_full_run(None)
