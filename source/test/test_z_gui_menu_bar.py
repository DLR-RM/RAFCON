import sys
import logging
import gtk
import threading
import time
import glib
import os

from awesome_tool.utils import log
from awesome_tool.mvc.models import ContainerStateModel, StateModel, GlobalVariableManagerModel
from awesome_tool.mvc.controllers import MainWindowController, StateDataPortEditorController,\
    SingleWidgetWindowController, SourceEditorController
from awesome_tool.mvc.views.main_window import MainWindowView
from awesome_tool.mvc.views import LoggingView, StateDataportEditorView, SingleWidgetWindowView, SourceEditorView
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.execution_state import ExecutionState
import awesome_tool.mvc.singleton
from awesome_tool.statemachine.state_machine import StateMachine

import variables_for_pytest


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def create_models(*args, **kargs):
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

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = ExecutionState('Nested')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, None, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = HierarchyState(name="Container")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, None, 0)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"

    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    # this is not allowed as the output port is already connected
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return state5, logger, ctr_state, global_var_manager_model


def wait_for_values_identical_number_state_machines(sm_manager_model, val2):
    values_identical = len(sm_manager_model.state_machines) == val2
    counter = 0
    while not values_identical:
        time.sleep(0.1)
        values_identical = len(sm_manager_model.state_machines) == val2
        counter += 1
        if counter == 50:
            break


def trigger_gui_signals(*args):
    print "Wait for the gui to initialize"
    time.sleep(2.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # ctr_state = HierarchyState(name="Container")
    # sm = StateMachine(ctr_state)
    # glib.idle_add(sm_manager_model.state_machine_manager.add_state_machine, sm)
    current_sm_length = len(sm_manager_model.state_machines)
    glib.idle_add(menubar_ctrl.on_new_activate, None)

    wait_for_values_identical_number_state_machines(sm_manager_model, current_sm_length+1)
    assert len(sm_manager_model.state_machines) == current_sm_length+1

    glib.idle_add(menubar_ctrl.on_open_activate, None, None, "../../test_scripts/basic_turtle_demo_sm")
    wait_for_values_identical_number_state_machines(sm_manager_model, current_sm_length+2)
    assert len(sm_manager_model.state_machines) == current_sm_length+2

    glib.idle_add(menubar_ctrl.on_refresh_libraries_activate, None)
    glib.idle_add(menubar_ctrl.on_refresh_all_activate, None, None, True)

    wait_for_values_identical_number_state_machines(sm_manager_model, 1)
    assert len(sm_manager_model.state_machines) == 1

    glib.idle_add(menubar_ctrl.on_save_as_activate, None, None, "/tmp")

    #glib.idle_add(main_window_controller.view["main_window"].emit, "destroy")
    glib.idle_add(menubar_ctrl.on_quit_activate, None)


def test_gui():
    variables_for_pytest.test_multithrading_lock.acquire()
    # delete all old state machines
    awesome_tool.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir("../awesome_tool/mvc/")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    awesome_tool.statemachine.singleton.library_manager.initialize()
    #logging_view = SingleWidgetWindowView(LoggingView, width=500, height=200, title='Logging')
    #setup_logger(logging_view['main_frame'])
    logging_view = LoggingView()
    setup_logger(logging_view)
    [execution_state, logger, ctr_state, gvm_model] = create_models()

    state_machine = StateMachine(ctr_state)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    variables_for_pytest.sm_manager_model = awesome_tool.mvc.singleton.state_machine_manager_model
    main_window_view = MainWindowView(logging_view)
    main_window_controller = MainWindowController(variables_for_pytest.sm_manager_model, main_window_view, gvm_model,
                                                  editor_type='LogicDataGrouped')

    thread = threading.Thread(target=trigger_gui_signals, args=[variables_for_pytest.sm_manager_model,
                                                                main_window_controller])
    thread.start()

    gtk.main()
    logger.debug("after gtk main")
    os.chdir("../../test")
    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_gui()