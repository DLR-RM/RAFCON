import pytest
import os
import hashlib
from distutils.version import StrictVersion

# general tool elements
import testing_utils
from testing_utils import call_gui_callback, run_gui, initialize_environment

from rafcon.utils import log
logger = log.get_logger(__name__)


def run_backward_compatibility_state_machines(state_machines_path):
    for state_machine_folder in os.listdir(state_machines_path):
        state_machine_path = os.path.join(state_machines_path, state_machine_folder)
        if not os.path.isdir(state_machine_path):
            continue

        run_state_machine(state_machine_path)


def assert_correctness_of_execution():
    import rafcon
    gvm = rafcon.core.singleton.global_variable_manager
    assert gvm.get_variable("b1") == 1
    assert gvm.get_variable("b2") == 1
    assert gvm.get_variable("h1") == 1
    assert gvm.get_variable("e1") == 1
    assert gvm.get_variable("l1") == 1


def run_state_machine(state_machine_path):
    import rafcon.core.config
    import rafcon.core.singleton as singletons
    import rafcon.gui.helpers.state_machine as gui_helper_statemachine

    gvm = rafcon.core.singleton.global_variable_manager
    execution_engine = singletons.state_machine_execution_engine
    state_machine_manager = singletons.state_machine_manager

    if not execution_engine.finished_or_stopped():
        raise RuntimeError("The execution engine is not stopped")

    print "Loading state machine from path: {}".format(state_machine_path)

    call_gui_callback(gui_helper_statemachine.open_state_machine, state_machine_path)
    call_gui_callback(testing_utils.remove_all_gvm_variables)
    call_gui_callback(execution_engine.start)

    if not execution_engine.join(3):
        raise RuntimeError("State machine did not finish within the given time")

    call_gui_callback(assert_correctness_of_execution)
    call_gui_callback(state_machine_manager.remove_state_machine, state_machine_manager.active_state_machine_id)


def test_backward_compatibility_storage(caplog):
    """This test ensures that old state machines storage formats can still be opened with the current RAFCON version"""
    path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_compatibility"))

    run_gui(gui_config={'HISTORY_ENABLED': False,
                        'AUTO_BACKUP_ENABLED': False},
            libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")})

    try:
        run_backward_compatibility_state_machines(path)
    except Exception:
        raise
    finally:
        # two warning per minor version lower than the current RAFCON version
        state_machines = len([filename for filename in os.listdir(path) if os.path.isdir(os.path.join(path, filename))])
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0)


def test_unchanged_storage_format(caplog):
    """This test ensures that the state machine storage format does not change in patch releases"""

    from rafcon.core.storage import storage
    from rafcon.gui.models.state_machine import StateMachineModel
    import rafcon

    path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_compatibility"))

    testing_utils.initialize_environment(
        gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
        libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")},
        gui_already_started=False
    )
    try:
        current_rafcon_version = StrictVersion(rafcon.__version__).version
        current_minor = "{}.{}".format(current_rafcon_version[0], current_rafcon_version[1])
        for filename in os.listdir(path):
            if filename.startswith(current_minor):
                old_state_machine_path = os.path.join(path, filename)
                break
        else:
            assert False, "There is no state machine for the current RAFCON minor version {}".format(current_minor)

        state_machine = storage.load_state_machine_from_path(old_state_machine_path)
        state_machine_m = StateMachineModel(state_machine)
        new_state_machine_path = testing_utils.get_unique_temp_path()
        storage.save_state_machine_to_path(state_machine, new_state_machine_path, True, True)
        state_machine_m.store_meta_data(copy_path=new_state_machine_path)

        old_state_machine_hash = calculate_state_machine_hash(old_state_machine_path)
        new_state_machine_hash = calculate_state_machine_hash(new_state_machine_path)
        assert old_state_machine_hash.digest() == new_state_machine_hash.digest()
    except Exception:
        raise
    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def calculate_state_machine_hash(path):
    """Calculates the hash of a state machine

    The function calculates the MD5 hash of all files in file system path belonging to a state machine, excluding the
    STATEMACHINE_FILE, but including the meta data.

    :param str path: The path to the state machine
    :return: The hash of the files
    :rtype: hashlib.Hashable
    """
    from rafcon.core.storage import storage

    paths_to_hash = []
    for root, dirs, filenames in os.walk(path):
        for filename in filenames:
            # The STATEMACHINE_FILE cannot be used for the hash as it e.g. includes a timestamp
            if filename != storage.STATEMACHINE_FILE:
                paths_to_hash.append(os.path.join(root, filename))

    paths_to_hash.sort()
    hash = hashlib.md5()
    for path in paths_to_hash:
        hash.update(open(path, 'rb').read())
    return hash

if __name__ == '__main__':
    test_backward_compatibility_storage(None)
    test_unchanged_storage_format(None)
    # pytest.main(['-s', __file__])
