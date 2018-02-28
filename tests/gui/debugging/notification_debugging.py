from os.path import exists, join

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from notifications import enable_debugging, disable_debugging, show_debug_graph
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


def create_small_state_machine():
    import rafcon.core.singleton
    import rafcon.gui.singleton
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    state1 = ExecutionState('State1', state_id='STATE1')
    state1.add_input_data_port("in", "str", "zero")
    state1.add_output_data_port("out", "int", 5)

    state2 = ExecutionState('State2', state_id='STATE2')
    state2.add_input_data_port("in", "int", 0)
    state2.add_output_data_port("out", "int", 3)

    root_state = HierarchyState(name='State3', state_id='STATE3')
    root_state.add_state(state1)
    root_state.add_state(state2)
    root_state.set_start_state(state1)

    root_state.add_transition(state1.state_id, 0, state2.state_id, None)

    sm = StateMachine(root_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)


def create_bigger_state_machine():
    import rafcon.core.singleton
    from gui.widget.test_storage import create_state_machine
    sm = create_state_machine()
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)


def patch_notifications():
    enable_debugging()
    # call_gui_callback(create_bigger_state_machine)
    # call_gui_callback(create_small_state_machine)
    from gui.widget.test_storage import create_models
    call_gui_callback(create_models)
    show_debug_graph()
    disable_debugging()

    from testing_utils import RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE
    assert exists(join(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, 'notification_output.gv'))
    assert exists(join(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, 'notification_print_out.txt'))


def test_notification_debugging_example(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        patch_notifications()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)


if __name__ == '__main__':
    test_notification_debugging_example(None)
    # import pytest
    # pytest.main(['-s', __file__])
