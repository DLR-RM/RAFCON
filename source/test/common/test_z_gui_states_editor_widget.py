import logging
import gtk
import threading
import time
import os
import signal

# general tool elements
from rafcon.utils import log

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.state_machine import StateMachine

# mvc elements
from rafcon.mvc.models import GlobalVariableManagerModel, ContainerStateModel
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

# singleton elements
import rafcon.mvc.singleton
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config

# test environment elements
import test_utils
from test_utils import test_multithrading_lock, call_gui_callback, TMP_TEST_PATH
from test_z_gui_state_type_change import get_state_editor_ctrl_and_store_id_dict
import pytest


def create_models(*args, **kargs):

    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)

    state1 = ExecutionState('State1', state_id="State1")
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id="State2")
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id="Nested")
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id="Nested2")
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id="State3")
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')
    # print state3.input_data_ports
    # print state3.output_data_ports
    # exit(0)

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    #print input_ctr_state
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    #print output_ctr_state
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
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
    # print ctr_state.input_data_ports
    # print ctr_state.output_data_ports
    # exit(0)

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4, 'Nested2': state5}
    sm = StateMachine(ctr_state)

    # remove existing state machines
    for sm_id in rafcon.statemachine.singleton.state_machine_manager.state_machines.keys():
        rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm_id)
    # add new state machine
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)
    # select state machine
    rafcon.mvc.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id
    # get state machine model
    sm_m = rafcon.mvc.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model = rafcon.mvc.singleton.global_variable_manager_model
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, ctr_state, global_var_manager_model, sm_m, state_dict


def wait_for_states_editor(main_window_controller, tab_key, max_time=5.0):
    assert tab_key in main_window_controller.get_controller('states_editor_ctrl').tabs
    time_waited = 0.0
    state_editor_ctrl = None
    while state_editor_ctrl is None:
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[tab_key]['controller']
        if state_editor_ctrl is not None:
            break
        time.sleep(0.1)
        time_waited += 0.1
        assert time_waited < max_time

    return state_editor_ctrl, time_waited


def list_store_id_dict(store):
    id = 0
    list_store_id = {}
    for row in store:
        # Print values of all columns
        list_store_id[row[0]] = id
        id += 1
    return list_store_id


def check_state_editor_models(sm_m, parent_state_m, main_window_controller, logger):
    sleep_time_max = 5.0
    states_editor_controller = main_window_controller.get_controller('states_editor_ctrl')
    if isinstance(parent_state_m, ContainerStateModel):
        # logger.debug("old tabs are:")
        # for tab in states_editor_controller.tabs.itervalues():
        #     logger.debug("%s %s" % (tab['state_m'], tab['state_m'].state.get_path()))
        # for tab in states_editor_controller.closed_tabs.itervalues():
        #     logger.debug("%s %s" % (tab['controller'].model, tab['controller'].model.state.get_path()))
        for state_m in parent_state_m.states.values():
            # get widget of state-model
            # if not state_m.state.name == "Decider":
            state_identifier = states_editor_controller.get_state_identifier(state_m)
            # time.sleep(1)
            call_gui_callback(sm_m.selection.set, [state_m])
            [state_editor_ctrl, time_waited] = wait_for_states_editor(main_window_controller,
                                                                      state_identifier,
                                                                      sleep_time_max)
            # logger.debug("wait for state's state editor %s" % time_waited)
            #
            # logger.debug("models are: \n ctrl  %s path: %s\n model %s path: %s" %
            #              (state_editor_ctrl.model, state_editor_ctrl.model.state.get_path(),
            #               state_m, state_m.state.get_path()))

            # # check if models of widget and in state_machine-model are the same
            assert state_editor_ctrl.model is state_m

    state_identifier = states_editor_controller.get_state_identifier(parent_state_m)
    call_gui_callback(sm_m.selection.set, [parent_state_m])
    [state_editor_ctrl, time_waited] = wait_for_states_editor(main_window_controller, state_identifier, sleep_time_max)
    # logger.debug("wait for state's state editor %s" % time_waited)
    # logger.debug("models are: \n ctrl  %s path: %s\n model %s path: %s" %
    #              (state_editor_ctrl.model, state_editor_ctrl.model.state.get_path(),
    #               parent_state_m, parent_state_m.state.get_path()))
    assert state_editor_ctrl.model is parent_state_m


def trigger_state_type_change_tests(*args):
    print "Wait for the gui to initialize"
    time.sleep(2.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    sm_m = args[2]
    state_dict = args[3]
    with_gui = args[4]
    logger = args[5]
    sleep_time_max = 5.0

    states_editor_controller = main_window_controller.get_controller('states_editor_ctrl')

    ####### General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())

    # HS -> BCS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # BCS -> HS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # HS -> PCS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # PCS -> ES
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # TODO all test that are not root_state-test have to be performed with Preemptive and Barrier Concurrency States as parents too

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())

    # HS -> BCS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # BCS -> HS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # HS -> PCS
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # PCS -> ES
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
    call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    # - check child state editor widgets
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # simple type change of root_state -> still could be extended

    if with_gui:
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        call_gui_callback(menubar_ctrl.on_stop_activate, None)
        menubar_ctrl.model.get_selected_state_machine_model().state_machine.file_system_path = TMP_TEST_PATH + "/test_states_editor_widget"
        call_gui_callback(menubar_ctrl.on_save_activate, None)
        call_gui_callback(menubar_ctrl.on_quit_activate, None)


@pytest.mark.parametrize("with_gui", [True])
def test_state_type_change_test(with_gui, caplog):

    test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(test_utils.RAFCON_PATH + "/mvc")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    global_config.load()  # load the default config
    global_gui_config.load()  # load the default config

    logger, state, gvm_model, sm_m, state_dict = create_models()

    test_utils.remove_all_libraries()
    rafcon.statemachine.singleton.library_manager.initialize()

    if test_utils.sm_manager_model is None:
            test_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = None
    if with_gui:
        main_window_view = MainWindowView()

        # load the meta data for the state machine
        test_utils.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data()

        main_window_controller = MainWindowController(test_utils.sm_manager_model, main_window_view,
                                                      editor_type='LogicDataGrouped')
    else:
        # load the meta data for the state machine
        test_utils.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data()

    thread = threading.Thread(target=trigger_state_type_change_tests,
                              args=[test_utils.sm_manager_model, main_window_controller,
                                    sm_m, state_dict, with_gui, logger])
    thread.start()

    if with_gui:
        gtk.main()
        logger.debug("Gtk main loop exited!")
        sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
        if sm:
            sm.root_state.join()
            logger.debug("Joined currently executing state machine!")
            thread.join()
            logger.debug("Joined test triggering thread!")
        os.chdir(test_utils.TEST_SM_PATH + "/../test/common")
        test_multithrading_lock.release()
    else:
        os.chdir(test_utils.TEST_SM_PATH + "/../test/common")
        thread.join()

    test_utils.reload_config()
    test_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])
