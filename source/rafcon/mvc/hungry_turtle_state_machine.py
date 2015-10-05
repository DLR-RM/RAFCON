import logging
import sys
import os
import gtk

from rafcon.utils import log
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.models import GlobalVariableManagerModel
import rafcon.statemachine.singleton
import rafcon.mvc.singleton

import gobject


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
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


def clean_transition_ids(container_state):
    import copy
    if hasattr(container_state, 'states'):
        transitions_copy = copy.copy(container_state.transitions)
        container_state.transitions.clear()
        for id, transition in transitions_copy.iteritems():
            container_state.transitions[transition.transition_id] = transition
        for state_id, state in container_state.states.iteritems():

            if hasattr(state, 'states'):
                clean_transition_ids(state)


def clean_data_flows_ids(container_state):
    import copy
    if hasattr(container_state, 'states'):
        data_flows_copy = copy.copy(container_state.data_flows)
        container_state.data_flows.clear()
        for id, data_flows in data_flows_copy.iteritems():
            container_state.data_flows[data_flows.data_flow_id] = data_flows
        for state_id, state in container_state.states.iteritems():

            if hasattr(state, 'states'):
                clean_data_flows_ids(state)


def run_turtle_demo():
    rafcon.statemachine.singleton.library_manager.initialize()
    # set base path of global storage
    rafcon.statemachine.singleton.global_storage.base_path = "../../test_scripts/hungry_turtle_demo"

    #load the state machine
    [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/hungry_turtle_demo")

    # clean_transition_ids(state_machine.root_state)
    # clean_data_flows_ids(state_machine.root_state)

    rafcon.statemachine.singleton.library_manager.initialize()
    [logger, gvm_model] = create_models()
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, gvm_model,
                                                  editor_type="LogicDataGrouped")
    #main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model)

    gtk.main()
    logger.debug("Gtk main loop exited!")

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    #print sys.path
    run_turtle_demo()
