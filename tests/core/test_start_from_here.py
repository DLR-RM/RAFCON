import os

import rafcon.core.singleton
from rafcon.core.execution.execution_history_items import CallItem
from tests import utils as testing_utils

SM_PATH = os.path.join("unit_test_state_machines", "start_from_here_test", "start_from_here_test")
LIB_PATH = os.path.join(testing_utils.TEST_STATE_MACHINES_PATH, "start_from_here_test")
ROOT = "QBYTMD"

# top-level tasks
TASK_1 = ROOT + "/QIZZOB"
TASK_2 = ROOT + "/RYZGLE"

# inside task_1 (LZLXFW is the HierarchyState wrapping the library)
BARRIER_CONCURRENCY    = ROOT + "/QIZZOB/LZLXFW/KEQYNU"   # all branches must finish before moving on
PREEMPTIVE_CONCURRENCY = ROOT + "/QIZZOB/LZLXFW/AESIQF"   # fastest branch cancels the rest

# inside task_2 (EAWGCM is the HierarchyState wrapping the library)
NESTED_LIBRARY      = ROOT + "/RYZGLE/EAWGCM/SFSEDE/MQRVHE/FLOSSL/PHPOHQ"  # hierarchy_4, 3 libs deep


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


def test_output_from_multiple_start_points(caplog):
    # baseline: full run from the beginning
    testing_utils.initialize_environment_core(libraries={"develop_test": LIB_PATH})
    engine = rafcon.core.singleton.state_machine_execution_engine
    sm = engine.execute_state_machine_from_path(path=testing_utils.get_test_sm_path(SM_PATH))
    try:
        # test output correct on full run
        assert len(sm.execution_histories) == 1
        assert _output(sm) == 82

        # test output correct from task 1
        _replay_from(sm, TASK_1)
        assert sm.root_state.states["QIZZOB"].output_data["output_1"] == 79
        assert _output(sm) == 82

        # test output correct from task 2
        _replay_from(sm, TASK_2)
        assert _output(sm) == 82

        # test output correct from barrier concurrency
        _replay_from(sm, BARRIER_CONCURRENCY)
        assert _output(sm) == 82

        # test output correct from preemptive concurrency
        _replay_from(sm, PREEMPTIVE_CONCURRENCY)
        assert _output(sm) == 82

        # test output correct from lower nested hierarchy
        _replay_from(sm, NESTED_LIBRARY)
        assert _output(sm) == 82

        # test output correct after multiple replays
        _replay_from(sm, TASK_2)
        _replay_from(sm, TASK_1)
        assert _output(sm) == 82
        assert len(sm.execution_histories) == 8
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_output_from_multiple_start_points(None)
