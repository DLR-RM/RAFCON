import logging
import sys
import os
import gtk

from utils import log
from mvc.controllers import MainWindowController
from mvc.views import LoggingView, MainWindowView
from mvc.models import ContainerStateModel, GlobalVariableManagerModel
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.execution_state import ExecutionState
import statemachine.singleton
from mvc.models.state_machine_manager import StateMachineManagerModel
from statemachine.state_machine import StateMachine
from statemachine.states.library_state import LibraryState


import gobject
gobject.threads_init()

def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, global_var_manager_model

def create_turtle_statemachine():

    basic_turtle_demo_state = HierarchyState("BasicTurtleDemo", path="../../test_scripts/basic_turtle_demo",
                                             filename="root_state.py")
    basic_turtle_demo_state.add_outcome("Success", 0)

    lib_state = LibraryState("ros_libraries", "init_ros_node", "0.1", "init ros node")

    basic_turtle_demo_state.add_state(lib_state)
    basic_turtle_demo_state.set_start_state(lib_state.state_id)

    # basic_turtle_demo_state.add_transition(basic_turtle_demo_state.state_id, 0, init_turtle_state.state_id, None)

    return basic_turtle_demo_state


def run_turtle_demo():

    statemachine.singleton.library_manager.initialize()

    basic_turtle_demo_state = create_turtle_statemachine()

    [basic_turtle_demo_state, version, creation_time] = statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/basic_turtle_demo_sm")

    statemachine.singleton.library_manager.initialize()
    logging_view = LoggingView()
    setup_logger(logging_view)
    [logger, gvm_model] = create_models()
    main_window_view = MainWindowView(logging_view)
    state_machine = StateMachine(basic_turtle_demo_state)
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = StateMachineManagerModel(statemachine.singleton.state_machine_manager)

    # load the meta data for the state machine
    sm_manager_model.get_active_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, gvm_model,
                                                  editor_type="LogicDataGrouped")
    #main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model)

    gtk.main()
    logger.debug("Gtk main loop exited!")

    #save state machine
    statemachine.singleton.global_storage.save_statemachine_as_yaml(
        sm_manager_model.get_active_state_machine_model().root_state.state,
        "../../test_scripts/basic_turtle_demo_sm",
        delete_old_state_machine=False)

    # save the meta data for the state machine
    sm_manager_model.get_active_state_machine_model().root_state.store_meta_data_for_state()

    statemachine.singleton.state_machine_manager.get_active_state_machine().root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    #print sys.path
    run_turtle_demo()
