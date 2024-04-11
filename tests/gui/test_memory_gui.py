# Test environment elements
import pytest
import time
import os
from tests.utils import TEST_SCRIPT_PATH
from tests import utils as testing_utils

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)

# def create_state_machine():
#     from rafcon.core.state_machine import StateMachine
#     from rafcon.core.states.hierarchy_state import HierarchyState
#     from rafcon.core.states.execution_state import ExecutionState
#     root = HierarchyState(name='root')
#     ex1 = ExecutionState(name='1', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     root.add_state(ex1)
#     ex2 = ExecutionState(name='2', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     root.add_state(ex2)
#     ex3 = ExecutionState(name='3', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     root.add_state(ex3)

#     # hierarchy state at the beginning
#     h4 = HierarchyState('H4')
#     ex41 = ExecutionState(name='41', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     h4.add_state(ex41)
#     ex42 = ExecutionState(name='42', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     h4.add_state(ex42)
#     ex43 = ExecutionState(name='43', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
#     h4.add_state(ex43)
#     h4.start_state_id = ex41.state_id
#     h4.add_transition(ex41.state_id, 0, ex42.state_id, None)
#     h4.add_transition(ex42.state_id, 0, ex43.state_id, None)
#     h4.add_transition(ex43.state_id, 0, h4.state_id, 0)

#     root.add_state(h4)
#     root.start_state_id = ex1.state_id
#     root.add_transition(h4.state_id, 0, ex1.state_id, None)
#     root.add_transition(ex1.state_id, 0, ex2.state_id, None)
#     root.add_transition(ex2.state_id, 0, ex3.state_id, None)
#     t_id = root.add_transition(ex3.state_id, 0, root.state_id, 0)
#     return StateMachine(root_state=root), t_id, h4.state_id

@pytest.mark.parametrize('gui', [{
    "gui_config": {
        'SESSION_RESTORE_ENABLED': False
    }
}], indirect=True, ids=["session restore disabled"])
@pytest.mark.parametrize('execution_number', range(20))
def test_memory_gui(gui, execution_number):
    menubar_ctrl = gui.singletons.main_window_controller.menu_bar_controller
    gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "memory_testing"))
    )
    testing_utils.wait_for_gui()


if __name__ == '__main__':
    # Call this test with "py.test tests/gui/test_memory_gui.py --durations=1"
    test_memory_gui(None)
