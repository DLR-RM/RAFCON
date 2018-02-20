import time

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
import pytest

logger = log.get_logger(__name__)


def create_state_machine():
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

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

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
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

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4,
                  'Nested2': state5}
    sm = StateMachine(ctr_state)
    return state_dict, sm


def wait_for_states_editor(main_window_controller, tab_key, max_time=5.0):
    assert tab_key in main_window_controller.get_controller('states_editor_ctrl').tabs
    time_waited = 0.0
    state_editor_ctrl = None
    while state_editor_ctrl is None:
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[tab_key]['controller']
        if state_editor_ctrl is not None:
            break
        last_time = time.time()
        testing_utils.wait_for_gui()
        time.sleep(0.02)
        time_waited += time.time() - last_time
        assert time_waited < max_time

    return state_editor_ctrl, time_waited


def check_state_editor_models(sm_m, parent_state_m, logger=None):
    from rafcon.gui.models import ContainerStateModel
    import rafcon.gui.singleton
    main_window_controller = rafcon.gui.singleton.main_window_controller

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
            sm_m.selection.set([state_m])
            [state_editor_ctrl, time_waited] = wait_for_states_editor(main_window_controller,
                                                                      state_identifier,
                                                                      sleep_time_max)
            # logger.debug("wait for state's state editor %s" % time_waited)
            #
            # logger.debug("models are: \n ctrl  %s path: %s\n model %s path: %s" %
            #              (state_editor_ctrl.model, state_editor_ctrl.model.state.get_path(),
            #               state_m, state_m.state.get_path()))

            # # check if models of widget and in state_machine-model are the same
            testing_utils.wait_for_gui()
            assert state_editor_ctrl.model is state_m

    state_identifier = states_editor_controller.get_state_identifier(parent_state_m)
    parent_state_m.get_state_machine_m()
    print "try to select", parent_state_m
    sm_m.selection.set([parent_state_m])
    [state_editor_ctrl, time_waited] = wait_for_states_editor(main_window_controller, state_identifier, sleep_time_max)
    # logger.debug("wait for state's state editor %s" % time_waited)
    # logger.debug("models are: \n ctrl  %s path: %s\n model %s path: %s" %
    #              (state_editor_ctrl.model, state_editor_ctrl.model.state.get_path(),
    #               parent_state_m, parent_state_m.state.get_path()))
    assert state_editor_ctrl.model is parent_state_m


@log.log_exceptions(None, gtk_quit=True)
def trigger_state_type_change_tests(with_gui=True):
    import rafcon.core.singleton
    import rafcon.gui.singleton
    main_window_controller = rafcon.gui.singleton.main_window_controller

    state_dict, sm = create_state_machine()
    first_sm_id = sm.state_machine_id

    # add new state machine
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, sm)
    call_gui_callback(testing_utils.wait_for_gui)
    # call_gui_callback(rafcon.core.singleton.state_machine_manager.__setattr__, "active_state_machine_id", first_sm_id)
    # select state machine
    call_gui_callback(rafcon.gui.singleton.state_machine_manager_model.__setattr__, "selected_state_machine_id", first_sm_id)
    # get state machine model
    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]

    def change_state_type(input_and_return_list, new_state_type, state_of_type_change):
        from gui.widget.test_state_type_change import get_state_editor_ctrl_and_store_id_dict
        sleep_time_max = 5.
        state_m = input_and_return_list.pop()
        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger)
        # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict[new_state_type]
        state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active(state_type_row_id)
        # - check child state editor widgets
        new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
        check_state_editor_models(sm_m, new_state_m, logger)
        input_and_return_list.append(new_state_m)

    ####### General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())

    # HS -> BCS
    input_and_return_list = [state_m]
    call_gui_callback(sm_m.selection.set, input_and_return_list)
    call_gui_callback(change_state_type, input_and_return_list, 'BARRIER_CONCURRENCY', state_of_type_change)

    # BCS -> HS
    call_gui_callback(change_state_type, input_and_return_list, 'HIERARCHY', state_of_type_change)

    # HS -> PCS
    call_gui_callback(change_state_type, input_and_return_list, 'PREEMPTION_CONCURRENCY', state_of_type_change)

    # PCS -> ES
    call_gui_callback(change_state_type, input_and_return_list, 'EXECUTION', state_of_type_change)

    # TODO all test that are not root_state-test have to be performed with Preemptive and Barrier Concurrency States as parents too

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    input_and_return_list = [sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())]

    # HS -> BCS
    call_gui_callback(sm_m.selection.set, input_and_return_list)
    call_gui_callback(change_state_type, input_and_return_list, 'BARRIER_CONCURRENCY', state_of_type_change)

    # BCS -> HS
    call_gui_callback(change_state_type, input_and_return_list, 'HIERARCHY', state_of_type_change)

    # HS -> PCS
    call_gui_callback(change_state_type, input_and_return_list, 'PREEMPTION_CONCURRENCY', state_of_type_change)

    # PCS -> ES
    call_gui_callback(change_state_type, input_and_return_list, 'EXECUTION', state_of_type_change)

    # simple type change of root_state -> still could be extended


def test_state_type_change_test(caplog):

    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        trigger_state_type_change_tests(with_gui=True)
    # except:
    #     raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    pytest.main(['-s', __file__])
