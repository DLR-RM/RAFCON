import logging
import gtk
import threading
import time

# core elements
import rafcon.core.singleton
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.storage import storage

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
from test_state_type_change import store_state_elements, check_state_elements, \
     check_list_ES, check_list_HS, check_list_BCS, check_list_PCS, \
     check_list_root_ES, check_list_root_HS, check_list_root_BCS, check_list_root_PCS, \
     get_state_editor_ctrl_and_store_id_dict, check_elements_ignores
from test_states_editor import check_state_editor_models
import pytest

NO_SAVE = False
TEST_PATH = testing_utils.get_unique_temp_path()


def on_save_activate(state_machine_m, logger):
    if state_machine_m is None or NO_SAVE:
        return
    save_path = state_machine_m.state_machine.file_system_path
    if save_path is None:
        return

    logger.debug("Saving state machine to {0}".format(save_path))
    storage.save_state_machine_to_path(state_machine_m.state_machine,
                                      state_machine_m.state_machine.file_system_path, delete_old_state_machine=True)

    state_machine_m.root_state.store_meta_data()
    logger.debug("Successfully saved graphics meta data.")


def save_state_machine(sm_model, path, logger, with_gui=False, menubar_ctrl=None):

    def print_states(state):
        if isinstance(state, ContainerState):
            for state_id, child_state in state.states.iteritems():
                print child_state.get_path()
                print_states(child_state)
    print_states(sm_model.state_machine.root_state)

    print "do SAVING OF STATEMACHINE"
    if with_gui:
        call_gui_callback(sm_model.state_machine.__setattr__, "file_system_path", path)
        print "by Menubar_ctrl"
        call_gui_callback(menubar_ctrl.on_save_activate, None)
    else:
        sm_model.state_machine.file_system_path = path
        print "by Function"
        on_save_activate(sm_model, logger)

    from test_storage import check_that_all_files_are_there

    if with_gui:
        call_gui_callback(check_that_all_files_are_there, sm_model.state_machine, path, False, True)
    else:
        check_that_all_files_are_there(sm_model.state_machine, path, False, True)


def create_state_machine():

    logger = log.get_logger(__name__)
    logger.setLevel(logging.VERBOSE)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)

    state1 = ExecutionState('State1', state_id='STATE1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id='NESTED')
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id='NESTED2')
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id='STATE3')
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

    ctr_state = HierarchyState(name="Container", state_id='CONT2')
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

    return logger, sm, state_dict


def prepare_state_machine_model(state_machine):
    import rafcon.gui.singleton
    rafcon.gui.singleton.state_machine_manager_model.state_machine_manager.add_state_machine(state_machine)
    testing_utils.wait_for_gui()
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = state_machine.state_machine_id

    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    sm_m.history.fake = False
    print "with_verbose is: ", sm_m.history.with_verbose
    sm_m.history.with_verbose = False
    # return ctr_state, sm_m, state_dict


def create_sm_model(with_gui=False, add_state_machine=False):

    # create state machine model
    import rafcon.gui.singleton
    if with_gui:
        [logger, sm, state_dict] = call_gui_callback(create_state_machine)
        call_gui_callback(prepare_state_machine_model, sm)
        sm_model = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    else:
        [logger, sm, state_dict] = create_state_machine()
        if add_state_machine:
            print "sm model 1"
            prepare_state_machine_model(sm)
            print "sm model 2"
            sm_model = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
            print "sm model 3"
        else:
            from rafcon.gui.models.state_machine import StateMachineModel
            sm_model = StateMachineModel(sm)
            print "sm model 4"
    print "sm model 5"
    return logger, sm_model, state_dict


# TODO introduce test_add_remove_history with_gui=True to have a more reliable unit-test
def test_add_remove_history(caplog):
    testing_utils.dummy_gui(None)
    ##################
    # Root_state elements

    # add state
    # - change state

    # remove state

    # add outcome
    # - change outcome

    # remove outcome

    # add transition
    # - change transition

    # remove transition

    # add input_data_port
    # - change input_data_port

    # remove input_data_port

    # add output_data_port
    # - change output_data_port

    # remove output_data_port

    # add scoped_variable
    # - change scoped_variable

    # remove scoped_variable

    # add data_flow
    # - change data_flow

    # remove data_flow

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    state_machine_path = TEST_PATH + '_test_add_remove'
    save_state_machine(sm_model, state_machine_path + '_before', logger, with_gui=False, menubar_ctrl=None)

    sm_history = sm_model.history

    state1 = HierarchyState('state1', state_id='STATE1')
    state2 = ExecutionState('state2', state_id='STATE2')

    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    sm_history.modifications.reset()
    state_dict['state1'] = state1
    state_dict['state2'] = state2

    state_path_dict = {}
    for key in state_dict.keys():
        state_path_dict[key] = state_dict[key].get_path()

    def do_check_for_state(state_name):

        from test_models import check_state_for_all_models

        def check_models_for_state_with_name(state_name, state_path_dict, sm_model):
            state_m = sm_model.get_state_model_by_path(state_path_dict[state_name])
            state = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])
            check_state_for_all_models(state, state_m)

        #############
        # add outcome
        # print "\n\n###########1", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        outcome_super = sm_model.state_machine.get_state_by_path(state_path_dict[state_name]).add_outcome('super')
        assert len(sm_history.modifications.single_trail_history()) == 2
        # print "\n\n###########2", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        sm_history.undo()
        # print "\n\n###########3", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        sm_history.redo()
        # print "\n\n###########4", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()

        ################
        # remove outcome
        # def print_all(state_m):
        #     # state = state_m.state
        #     # print state_m
        #     print state_m.state.name  # , state_m.state.get_path()
        #     if isinstance(state_m.state, ContainerState):
        #         for state_m in state_m.states:
        #             print_all(state_m)

        print sm_model, "\n", sm_model.root_state
        save_state_machine(sm_model, state_machine_path + '_before', logger, with_gui=False, menubar_ctrl=None)
        # new outcome should be the third one
        sm_model.state_machine.get_state_by_path(state_path_dict[state_name]).remove_outcome(outcome_super)
        assert len(sm_history.modifications.single_trail_history()) == 3
        save_state_machine(sm_model, state_machine_path + '_after', logger, with_gui=False, menubar_ctrl=None)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.undo()
        save_state_machine(sm_model, state_machine_path + '_undo', logger, with_gui=False, menubar_ctrl=None)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        save_state_machine(sm_model, state_machine_path + '_redo', logger, with_gui=False, menubar_ctrl=None)

        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        state4 = ExecutionState('State4', state_id='STATE4')
        state5 = ExecutionState('State5', state_id='STATE5')

        #############
        # add state
        print "xyz", state_dict[state_name].states.keys(), state_name
        print "xyz", sm_model.state_machine.get_state_by_path(state_path_dict[state_name]).states.keys(), state_name
        sm_model.state_machine.get_state_by_path(state_path_dict[state_name]).add_state(state4)
        state_path_dict['state4'] = state4.get_path()
        print sm_model.state_machine.get_state_by_path(state_path_dict['state4']).get_path()
        assert len(sm_history.modifications.single_trail_history()) == 4
        sm_model.state_machine.get_state_by_path(state_path_dict[state_name]).add_state(state5)
        state_path_dict['state5'] = state5.get_path()
        print sm_model.state_machine.get_state_by_path(state_path_dict['state5']).get_path()
        assert len(sm_history.modifications.single_trail_history()) == 5
        print state_dict[state_name].states
        # store_state_machine(sm_model, test_history_path1)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        # store_state_machine(sm_model, test_history_path2)

        # resolve reference
        print state4.get_path()
        print "\n\n\n"
        print sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        print sm_model.state_machine.get_state_by_path(state_path_dict['Nested']).states
        print "\n\n\n"
        print sm_model.state_machine.get_state_by_path(state4.get_path()).get_path(), "\n", state4.get_path()
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state5 = sm_model.state_machine.get_state_by_path(state_path_dict['state5'])
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        outcome_state4 = state4.add_outcome('UsedHere')
        assert len(sm_history.modifications.single_trail_history()) == 6
        outcome_state5 = state5.add_outcome('UsedHere')
        assert len(sm_history.modifications.single_trail_history()) == 7

        ################
        # add transition from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id
        new_transition_id1 = state_dict[state_name].add_transition(from_state_id=state4.state_id,
                                                                   from_outcome=outcome_state4,
                                                                   to_state_id=state5.state_id, to_outcome=None)
        state_m = sm_model.get_state_model_by_path(state_path_dict[state_name])
        state = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])
        check_state_for_all_models(state, state_m)
        assert len(sm_history.modifications.single_trail_history()) == 8
        state_dict[state_name].add_transition(from_state_id=state5.state_id, from_outcome=outcome_state5,
                                              to_state_id=state.state_id, to_outcome=-1)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 9
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state5 = sm_model.state_machine.get_state_by_path(state_path_dict['state5'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        ###################
        # remove transition
        state_dict[state_name].remove_transition(new_transition_id1)  # new outcome should be the third one
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_model.history.modifications.single_trail_history()) == 10
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state5 = sm_model.state_machine.get_state_by_path(state_path_dict['state5'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #############
        # remove state
        state_dict[state_name].remove_state(state5.state_id)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 11
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #############
        # add input_data_port
        input_state4 = state4.add_input_data_port("input", "str", "zero")
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 12
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #############
        # remove input_data_port
        state4.remove_input_data_port(input_state4)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 13
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #############
        # add output_data_port
        output_state4 = state4.add_output_data_port("output_"+state4.state_id, "int")
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 14
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #############
        # remove output_data_port
        state4.remove_output_data_port(output_state4)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 15
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        # prepare again state4
        output_state4 = state4.add_output_data_port("output", "int")
        input_state4 = state4.add_input_data_port("input_new", "str", "zero")
        assert len(sm_history.modifications.single_trail_history()) == 17
        output_state4 = state4.add_output_data_port("output_new", "int")
        assert len(sm_history.modifications.single_trail_history()) == 18

        state5 = ExecutionState('State5', 'STATE5')
        state_dict[state_name].add_state(state5)
        print state_path_dict['state5'] + "\n" + state5.get_path()
        assert state_path_dict['state5'] == state5.get_path()
        assert len(sm_history.modifications.single_trail_history()) == 19
        input_par_state5 = state5.add_input_data_port("par", "int", 0)
        assert len(sm_history.modifications.single_trail_history()) == 20
        output_res_state5 = state5.add_output_data_port("res", "int")
        assert len(sm_history.modifications.single_trail_history()) == 21


        #####################
        # add scoped_variable
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        scoped_buffer_nested = state_dict[state_name].add_scoped_variable("buffer", "int")
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_history.modifications.single_trail_history()) == 22
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.state_machine.get_state_by_path(state_path_dict['state4'])
        state5 = sm_model.state_machine.get_state_by_path(state_path_dict['state5'])
        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_path_dict[state_name])

        #####################
        # remove scoped_variable
        state_dict[state_name].remove_scoped_variable(scoped_buffer_nested)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_model.history.modifications.single_trail_history()) == 23
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # add data_flow
        new_df_id = state_dict[state_name].add_data_flow(from_state_id=state4.state_id, from_data_port_id=output_state4,
                                                         to_state_id=state5.state_id, to_data_port_id=input_par_state5)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_model.history.modifications.single_trail_history()) == 24
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        ################
        # remove data_flow
        state_dict[state_name].remove_data_flow(new_df_id)
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        assert len(sm_model.history.modifications.single_trail_history()) == 25
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_path_dict, sm_model)

    # state_check_dict1 = print_all_states_with_path_and_name(state_dict['Container'])
    # do_check_for_state(state_dict, state_name='state1')
    # do_check_for_state(state_dict, state_name='state2')
    do_check_for_state(state_name='Nested')
    sm_model.history.modifications.reset()
    save_state_machine(sm_model, TEST_PATH + "_add_remove_child_hierarchical_state", logger, with_gui=False)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # state_check_dict2 = print_all_states_with_path_and_name(state_dict['Container'])
    do_check_for_state(state_name='Container')
    save_state_machine(sm_model, TEST_PATH + "_add_remove_root_state", logger, with_gui=False)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict2)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_state_property_modifications_history(caplog):
    ##################
    # state properties
    # TODO LibraryState test for properties like mentioned in the notification-test but also general for add and remove

    # change name

    # change parent

    # change states

    # change outcomes

    # change transitions

    # change input_data_ports

    # change output_data_ports

    # change scoped_variables

    # change data_flows

    # change script

    # change script_text

    # change description

    # change active

    # set_start_state

    # change start_state_id

    # change child_execution

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    state1 = ExecutionState('State1')
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    output_state1 = state1.add_output_data_port("output", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")

    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    input_number_state2 = state2.add_input_data_port("number", "int", 5)
    output_res_state2 = state2.add_output_data_port("res", "int")

    nested_state_path = state_dict['Nested'].get_path()
    state_dict['Nested'].add_state(state1)
    assert len(sm_model.history.modifications.single_trail_history()) == 2
    state_dict['Nested'].add_state(state2)
    assert len(sm_model.history.modifications.single_trail_history()) == 3
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")
    assert len(sm_model.history.modifications.single_trail_history()) == 4

    oc_again_state1 = state1.add_outcome("again")
    assert len(sm_model.history.modifications.single_trail_history()) == 5
    oc_counted_state1 = state1.add_outcome("counted")
    assert len(sm_model.history.modifications.single_trail_history()) == 6

    oc_done_state2 = state2.add_outcome("done")
    oc_best_state2 = state2.add_outcome("best")
    oc_full_state2 = state2.add_outcome("full")
    assert len(sm_model.history.modifications.single_trail_history()) == 9

    oc_great_nested = state_dict['Nested'].add_outcome("great")
    assert len(sm_model.history.modifications.single_trail_history()) == 10

    #######################################
    ######## Properties of State ##########

    # name(self, name)
    state_dict['Nested'].name = 'nested'
    sm_model.history.undo()
    sm_model.history.redo()

    # parent(self, parent) State
    # state_dict['Nested'].parent = state_dict['State3']
    # sm_model.history.undo()
    # sm_model.history.redo()

    # input_data_ports(self, input_data_ports) None or dict
    state_dict['Nested'].input_data_ports = {}
    sm_model.history.undo()
    sm_model.history.redo()

    # output_data_ports(self, output_data_ports) None or dict
    state_dict['Nested'].output_data_ports = {}
    sm_model.history.undo()
    sm_model.history.redo()

    # outcomes(self, outcomes) None or dict
    state_dict['Nested'].outcomes = state_dict['Nested'].outcomes
    sm_model.history.undo()
    sm_model.history.redo()
    state_dict['Nested'].outcomes = {}
    sm_model.history.undo()
    sm_model.history.redo()

    script_text = '\ndef execute(self, inputs, outputs, gvm):\n\tself.logger.debug("Hello World")\n\treturn 0\n'
    script_text1 = '\ndef execute(self, inputs, outputs, gvm):\n\tself.logger.debug("Hello NERD")\n\treturn 0\n'

    # script(self, script) Script -> no script setter any more only script_text !!!
    state_dict['Nested'].script_text = script_text
    sm_model.history.undo()
    sm_model.history.redo()

    # script_text(self, script_text)
    state_dict['Nested'].script_text = script_text1
    sm_model.history.undo()
    sm_model.history.redo()

    # description(self, description) str
    state_dict['Nested'].description = "awesome"
    sm_model.history.undo()
    sm_model.history.redo()

    # # active(self, active) bool
    # state_dict['Nested'].active = True
    # sm_model.history.undo()
    # sm_model.history.redo()
    #
    # # child_execution(self, child_execution) bool
    # state_dict['Nested'].child_execution = True
    # sm_model.history.undo()
    # sm_model.history.redo()

    ############################################
    ###### Properties of ContainerState ########

    # set_start_state(self, state) State or state_id
    state_dict['Nested'] = sm_model.state_machine.get_state_by_path(nested_state_path)
    state1_m = sm_model.get_state_model_by_path(state1.get_path())
    state_dict['Nested'].set_start_state(state1_m.state.state_id)
    sm_model.history.undo()
    sm_model.history.redo()
    # set_start_state(self, start_state)
    state2_m = sm_model.get_state_model_by_path(state2.get_path())
    state_dict['Nested'].set_start_state(state2_m.state)
    sm_model.history.undo()
    sm_model.history.redo()

    # transitions(self, transitions) None or dict
    state_dict['Nested'].transitions = {}
    sm_model.history.undo()
    sm_model.history.redo()

    # data_flows(self, data_flows) None or dict
    state_dict['Nested'].data_flows = {}
    sm_model.history.undo()
    sm_model.history.redo()

    # scoped_variables(self, scoped_variables) None or dict
    state_dict['Nested'].scoped_variables = {}
    sm_model.history.undo()
    sm_model.history.redo()

    # states(self, states) None or dict
    state_dict['Nested'].states = {}
    assert sm_model.history.modifications.single_trail_history()[-1].before_overview['method_name'][-1] == 'states'
    sm_model.history.undo()
    sm_model.history.redo()

    save_state_machine(sm_model, TEST_PATH + "_state_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_outcome_property_modifications_history(caplog):
    ##################
    # outcome properties

    # change name

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    def do_check_for_state(state_dict, state_name='Nested'):
        ####################################################
        # modify outcome and generate in previous a observer
        for outcome_id, outcome in state_dict['Nested2'].outcomes.iteritems():
            if not outcome_id < 0:
                outcome.name = "new_name_" + str(outcome_id)
                sm_model.history.undo()
                sm_model.history.redo()
                # resolve reference
                state_dict['Nested2'] = sm_model.get_state_model_by_path(state_dict['Nested2'].get_path()).state

        ##########################
        # check for ContainerState -> should be unnecessary
        state_model = sm_model.get_state_model_by_path(state_dict['Nested'].get_path())

        ####################################################
        # modify outcome
        for outcome_id, outcome in state_dict['Nested'].outcomes.iteritems():
            outcome.name = "new_name_" + str(outcome_id)
            sm_model.history.undo()
            sm_model.history.redo()
            # resolve reference
            state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

        # outcome_id(self, outcome_id) -> no data_fow_id setter anymore
        # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
        # state_dict['Nested'].outcomes.values()[0].outcome_id += 10
        # sm_model.history.undo()
        # sm_model.history.redo()

    # do_check_for_state(state_dict, history_ctrl, state_name='Nested')
    do_check_for_state(state_dict, state_name='Container')
    save_state_machine(sm_model, TEST_PATH + "_outcome_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def wait_for_states_editor(main_window_controller, tab_key, max_time=5.0):
    assert tab_key in main_window_controller.get_controller('states_editor_ctrl').tabs
    time_waited = 0.0
    state_editor_ctrl = None
    while state_editor_ctrl is None:
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[tab_key]['controller']
        time.sleep(0.1)
        time_waited += 0.1
        assert time_waited < max_time

    return state_editor_ctrl, time_waited


def test_transition_property_modifications_history(caplog):
    ##################
    # transition properties

    # change modify_origin

    # change from_outcome

    # change to_state

    # change to_outcome

    # modify_transition_from_state

    # modify_transition_from_outcome

    # modify_transition_to_outcome

    # modify_transition_to_state

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    state1 = ExecutionState('State1')
    outcome_again_state1 = state1.add_outcome("again")
    state2 = ExecutionState('State2')
    oc_done_state2 = state2.add_outcome("done")
    oc_best_state2 = state2.add_outcome("best")
    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    oc_great_nested = state_dict['Nested'].add_outcome("great")
    outcome_counted_state1 = state1.add_outcome("counted")
    oc_full_state2 = state2.add_outcome("full")
    # assert False

    new_trans_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id, from_outcome=outcome_again_state1,
                                                       to_state_id=state1.state_id, to_outcome=None)

    # modify_origin(self, from_state, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].modify_origin(from_state=state2.state_id,
                                                                 from_outcome=oc_full_state2)
    sm_model.history.undo()
    sm_model.history.redo()

    # from_outcome(self, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].from_outcome = oc_done_state2
    sm_model.history.undo()
    sm_model.history.redo()

    # to_state(self, to_state)
    state_dict['Nested'].transitions[new_trans_id].to_state = state2.state_id
    sm_model.history.undo()
    sm_model.history.redo()

    # to_outcome(self, to_outcome)
    # TODO do an example what does not violate the roles of transitions
    # TODO test the respective function in transition
    # state_dict['Nested'].transitions[new_trans_id].to_outcome = oc_great_nested
    # sm_model.history.undo()
    # sm_model.history.redo()

    # # transition_id(self, transition_id)  -> no transition_id setter anymore
    # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    # state_dict['Nested'].transitions[new_trans_id].transition_id += 1
    # sm_model.history.undo()
    # sm_model.history.redo()

    # reset observer and testbed
    state_dict['Nested'].remove_transition(new_trans_id)
    new_df_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id,
                                                    from_outcome=outcome_again_state1,
                                                    to_state_id=state1.state_id,
                                                    to_outcome=None)
    sm_model.history.undo()
    sm_model.history.redo()

    ##### modify from parent state #######
    # modify_transition_from_state(self, transition_id, from_state, from_outcome)
    # state_dict['Nested'].modify_transition_from_state(new_df_id, from_state=state2.state_id,
    #                                                   from_outcome=oc_full_state2)
    state_dict['Nested'].transitions[new_df_id].modify_origin(state2.state_id, oc_full_state2)
    sm_model.history.undo()
    sm_model.history.redo()

    # modify_transition_from_outcome(self, transition_id, from_outcome)
    # state_dict['Nested'].modify_transition_from_outcome(new_df_id, from_outcome=oc_done_state2)
    state_dict['Nested'].transitions[new_df_id].from_outcome = oc_done_state2
    sm_model.history.undo()
    sm_model.history.redo()

    # TODO test the respective function in transition
    # modify_transition_to_outcome(self, transition_id, to_outcome)
    # Invalid test: to_outcome must be None as transition goes to child state
    # state_dict['Nested'].modify_transition_to_outcome(new_df_id, to_outcome=oc_great_nested)
    # sm_model.history.undo()
    # sm_model.history.redo()

    # modify_transition_to_state(self, transition_id, to_state, to_outcome)
    # state_dict['Nested'].modify_transition_to_state(new_df_id, to_state=state1.state_id)
    state_dict['Nested'].transitions[new_df_id].to_state = state1.state_id
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, TEST_PATH + "_transition_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_input_port_modify_notification(caplog):
    ##################
    # input_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    new_input_data_port_id = state_dict['Nested2'].add_input_data_port(name='new_input', data_type='str')
    sm_model.history.undo()
    sm_model.history.redo()

    ################################
    # check for modification of name
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].name = 'changed_new_input_name'
    sm_model.history.undo()
    sm_model.history.redo()

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].change_data_type(data_type='str',
                                                                                    default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, TEST_PATH + "_input_port_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_output_port_modify_notification(caplog):

    ##################
    # output_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    new_output_data_port_id = state_dict['Nested2'].add_output_data_port(name='new_output', data_type='str')

    ################################
    # check for modification of name
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].name = 'changed_new_output_name'
    sm_model.history.undo()
    sm_model.history.redo()

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].change_data_type(data_type='str',
                                                                                      default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, TEST_PATH + "_output_port_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_scoped_variable_modify_notification(caplog):
    ##################
    # scoped_variable properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    new_scoped_variable_id = state_dict['Nested'].add_scoped_variable(name='new_output', data_type='str')

    ################################
    # check for modification of name
    # state_dict['Nested'].modify_scoped_variable_name('changed_new_scoped_var_name', new_scoped_variable_id)
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].name = 'changed_new_scoped_var_name'
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #####################################
    # check for modification of data_type
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #########################################
    # check for modification of default_value
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].change_data_type(data_type='str',
                                                                                   default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, TEST_PATH + "_scoped_variable_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_data_flow_property_modifications_history(caplog):
    ##################
    # data_flow properties

    # change modify_origin

    # change from_key

    # change modify_target

    # change to_key

    # modify_transition_from_state

    # modify_transition_from_key

    # modify_transition_to_key

    # modify_transition_to_state

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    [logger, sm_model, state_dict] = create_sm_model()

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")
    input_number_state2 = state2.add_input_data_port("number", "int", 5)

    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id, from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)

    ##### modify from data_flow #######
    # modify_origin(self, from_state, from_key)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(from_state=state1.state_id, from_key=output_state1)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # from_key(self, from_key)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_target(self, to_state, to_key)
    state_dict['Nested'].data_flows[new_df_id].modify_target(to_state=state2.state_id, to_key=input_par_state2)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # to_key(self, to_key)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # data_flow_id(self, data_flow_id) -> no data_fow_id setter anymore

    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    new_df_id = state_dict['Nested'].data_flows[new_df_id].data_flow_id  # only if history.redo was not run

    # reset observer and testbed
    state_dict['Nested'].remove_data_flow(new_df_id)
    sm_model.history.undo()
    sm_model.history.redo()
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id, from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ##### modify from parent state #######
    # modify_data_flow_from_state(self, data_flow_id, from_state, from_key)
    # state_dict['Nested'].modify_data_flow_from_state(new_df_id, from_state=state1.state_id, from_key=output_state1)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(state1.state_id, output_state1)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_from_key(self, data_flow_id, from_key)
    # state_dict['Nested'].modify_data_flow_from_key(new_df_id, from_key=output_count_state1)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_state(self, data_flow_id, to_state, to_key)
    # state_dict['Nested'].modify_data_flow_to_state(new_df_id, to_state=state2.state_id, to_key=input_par_state2)
    state_dict['Nested'].data_flows[new_df_id].modify_target(state2.state_id, input_par_state2)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_key(self, data_flow_id, to_key)
    # state_dict['Nested'].modify_data_flow_to_key(new_df_id, to_key=input_number_state2)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2
    sm_model.history.undo()
    sm_model.history.redo()

    save_state_machine(sm_model, TEST_PATH + "_data_flow_properties", logger, with_gui=False)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


@pytest.mark.parametrize("with_gui", [False, True])
def test_state_machine_modifications_with_gui(with_gui, caplog):

    testing_utils.dummy_gui(None)

    if with_gui:
        testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True})
        e = None
        try:
            trigger_state_type_change_tests(with_gui)
        except Exception as e:
            pass
        finally:
            testing_utils.close_gui()
            if e:
                logging.exception("Test failed with exception {0}".format(e))
            testing_utils.shutdown_environment(caplog=caplog)
    else:
        testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True},
                                             gui_already_started=False)
        print "start thread"
        trigger_state_type_change_tests(with_gui)
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

    print "FINISH test_state_machine_modifications_with_gui", with_gui


@pytest.mark.parametrize("with_gui", [True])
def test_state_type_change_bugs_with_gui(with_gui, caplog):

    testing_utils.dummy_gui(None)

    if with_gui:
        testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True})
        e = None
        try:
            trigger_state_type_change_typical_bug_tests(with_gui=True)
        except Exception as e:
            pass
        finally:
            testing_utils.close_gui()
            testing_utils.shutdown_environment(caplog=caplog)
            if e:
                raise e
    else:
        testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True},
                                             gui_already_started=False)
        trigger_state_type_change_typical_bug_tests(with_gui)
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

    print "FINISH", test_state_type_change_bugs_with_gui, "with_gui", with_gui


def test_multiple_undo_redo_bug_with_gui(caplog):

    testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': True, 'HISTORY_ENABLED': True})
    e = None
    try:
        trigger_multiple_undo_redo_bug_tests(with_gui=True)
    except Exception as e:
        pass
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)
        if e:
            raise e


def do_type_change(target_sm_m, target_state_m, drop_down_combo_label_of_new_target_state_type, logger=None):
    import rafcon.gui.singleton
    main_window_controller = rafcon.gui.singleton.main_window_controller
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(target_sm_m, target_state_m, main_window_controller, 5., logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict[drop_down_combo_label_of_new_target_state_type]
    state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active(state_type_row_id)


# @log.log_exceptions(None, gtk_quit=False)
def trigger_state_type_change_tests(with_gui):
    import rafcon.gui.singleton
    import rafcon.gui.helpers.state as gui_helper_state
    main_window_controller = rafcon.gui.singleton.main_window_controller

    [logger, sm_m, state_dict] = create_sm_model(with_gui, add_state_machine=True)
    sleep_time_max = 5  # 0.5

    check_elements_ignores.append("internal_transitions")

    # General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    parent_of_type_change = 'Container'

    print "start 1"
    # do state_type_change with gui
    if with_gui:
        call_gui_callback(sm_m.history.modifications.reset)
    else:
        sm_m.history.modifications.reset()
    print "start 2"

    state_parent_m = sm_m.get_state_model_by_path(state_dict[parent_of_type_change].get_path())
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    print "\n\n %s \n\n" % state_m.state.name
    if with_gui:
        call_gui_callback(sm_m.selection.set, [state_m])
    else:
        sm_m.selection.set([state_m])

    list_store_id_from_state_type_dict = {}
    state_editor_ctrl = None
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    state_machine_path = TEST_PATH + "_state_type_change_{0}".format("with_gui" if with_gui else "without_gui")
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller') if with_gui else None
    save_state_machine(sm_m, state_machine_path, logger, with_gui, menubar_ctrl)

    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, state_m, logger)

    # HS -> BCS
    save_state_machine(sm_m, state_machine_path + '_before1', logger, with_gui, menubar_ctrl)
    # - do state type change
    logger.info("HS -> BCS")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, state_m, 'BARRIER_CONCURRENCY', logger)
    else:
        gui_helper_state.change_state_type(state_m, BarrierConcurrencyState)
        # state_dict[parent_of_type_change].change_state_type(state_m.state, BarrierConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_m, state_machine_path + '_after1', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 2
    logger.info("BCS -> HS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_m, state_machine_path + '_undo1', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("HS -> BCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_m, state_machine_path + '_redo1', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_BCS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # BCS -> HS
    save_state_machine(sm_m, state_machine_path + '_before2', logger, with_gui, menubar_ctrl)
    logger.info("BCS -> HS 2")
    if with_gui:
        call_gui_callback(sm_m.state_machine.__setattr__, "file_system_path", state_machine_path)
        call_gui_callback(do_type_change, sm_m, new_state_m, 'HIERARCHY', logger)
    else:
        sm_m.state_machine.file_system_path = state_machine_path
        gui_helper_state.change_state_type(new_state_m, HierarchyState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    print "\n\n ###### State: %s" % state_dict[state_of_type_change]
    from test_storage import check_that_all_files_are_there
    check_that_all_files_are_there(sm_m.state_machine, state_machine_path + '_before2', False, True)
    check_that_all_files_are_there(sm_m.state_machine, state_machine_path + '_after2', False, True)

    save_state_machine(sm_m, state_machine_path + '_after2', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 3
    logger.info("HS -> BCS 2 (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo2', logger, with_gui, menubar_ctrl)

    logger.info("BCS -> HS 2 (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_m, state_machine_path + '_redo2', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_m, state_machine_path + '_before3', logger, with_gui, menubar_ctrl)
    logger.info("HS -> PCS")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, 'PREEMPTION_CONCURRENCY', logger)
    else:
        gui_helper_state.change_state_type(new_state_m, PreemptiveConcurrencyState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, PreemptiveConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_m, state_machine_path + '_after3', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 4
    logger.info("PCS -> HS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo3', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    logger.info("HS -> PCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_m, state_machine_path + '_redo3', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # PCS -> ES
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_m, state_machine_path + '_before4', logger, with_gui, menubar_ctrl)
    logger.info("PCS -> ES")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, 'EXECUTION', logger)
    else:
        gui_helper_state.change_state_type(new_state_m, ExecutionState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_m, state_machine_path + '_after4', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 5
    logger.info("ES -> PCS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo4', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    logger.info("PCS -> ES (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_m, state_machine_path + '_redo4', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_ES, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, state_m, logger)

    # HS -> BCS
    logger.info("HS -> BCS root")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, state_m, 'BARRIER_CONCURRENCY', logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, BarrierConcurrencyState)
        # sm_m.state_machine.change_root_state_type(BarrierConcurrencyState)
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    # import time
    # time.sleep(100)

    assert len(sm_m.history.modifications.single_trail_history()) == 6
    logger.info("BCS -> HS root (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("HS -> BCS root (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_BCS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # BCS -> HS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("BCS -> HS root 2")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, 'HIERARCHY', logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, HierarchyState)
        # sm_m.state_machine.change_root_state_type(HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    assert len(sm_m.history.modifications.single_trail_history()) == 7
    logger.info("HS -> BCS root 2 (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_BCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("BCS -> HS root 2 (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("HS -> PCS")
    # - do state type change
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, 'PREEMPTION_CONCURRENCY', logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, PreemptiveConcurrencyState)
        # sm_m.state_machine.change_root_state_type(PreemptiveConcurrencyState)
        state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    assert len(sm_m.history.modifications.single_trail_history()) == 8
    logger.info("PCS -> HS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("HS -> PCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # PCS -> ES
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("PCS -> ES")
    # - do state type change
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, 'EXECUTION', logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, ExecutionState)
        # sm_m.state_machine.change_root_state_type(ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    assert len(sm_m.history.modifications.single_trail_history()) == 9
    logger.info("ES -> PCS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    # print "path: ", state_dict[state_of_type_change].get_path(), " state ", state_dict[state_of_type_change]
    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("PCS -> ES (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_ES, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)
        # final wait for gaphas
        testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    else:
        # final wait for models
        testing_utils.wait_for_gui()

    check_elements_ignores.remove("internal_transitions")
    print check_elements_ignores


# @log.log_exceptions(None, gtk_quit=True)
def trigger_state_type_change_typical_bug_tests(with_gui):
    import rafcon.gui.singleton
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller

    [logger, sm_m, state_dict] = create_sm_model(with_gui, add_state_machine=True)

    if not with_gui:
        testing_utils.wait_for_gui()

    check_elements_ignores.append("internal_transitions")

    # General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    parent_of_type_change = 'Container'

    # do state_type_change with gui
    if with_gui:
        call_gui_callback(sm_m.history.modifications.reset)
    else:
        sm_m.history.modifications.reset()

    state_parent_m = sm_m.get_state_model_by_path(state_dict[parent_of_type_change].get_path())
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    print "\n\n %s \n\n" % state_m.state.name
    if with_gui:
        call_gui_callback(sm_m.selection.set, [state_m])
    else:
        sm_m.selection.set([state_m])

    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)

    current_sm_length = len(sm_manager_model.state_machines)
    # print "1:", sm_manager_model.state_machines.keys()
    logger.debug('number of sm is : {0}'.format(sm_manager_model.state_machines.keys()))

    def create_state_machine():
        root_state = HierarchyState("new root state", state_id="ROOT")
        state_machine = StateMachine(root_state)
        sm_manager_model.state_machine_manager.add_state_machine(state_machine)
        sm_manager_model.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
        return state_machine

    state_machine_path = TEST_PATH + '_state_type_change_bug_tests'
    if with_gui:
        # call_gui_callback(sm_manager_model.state_machine_manager.add_state_machine, state_machine)
        state_machine = call_gui_callback(create_state_machine)
    else:
        logger.debug("Creating new state-machine...")
        state_machine = create_state_machine()

    logger.debug('number of sm is : {0}'.format(sm_manager_model.state_machines.keys()))
    assert len(sm_manager_model.state_machines) == current_sm_length+1
    sm_m = sm_manager_model.state_machines[state_machine.state_machine_id]
    # save_state_machine(sm_m, state_machine_path + '_before1', logger, with_gui, menubar_ctrl)
    h_state1 = HierarchyState(state_id='HSTATE1')
    if with_gui:
        call_gui_callback(sm_m.state_machine.root_state.add_state, h_state1)
    else:
        sm_m.state_machine.root_state.add_state(h_state1)
    h_state2 = HierarchyState(state_id='HSTATE2')
    if with_gui:
        call_gui_callback(h_state1.add_state, h_state2)
    else:
        h_state1.add_state(h_state2)
    ex_state1 = ExecutionState(state_id='EXSTATE1')
    if with_gui:
        call_gui_callback(h_state1.add_state, ex_state1)
    else:
        h_state1.add_state(ex_state1)
    ex_state2 = ExecutionState(state_id='EXSTATE2')
    if with_gui:
        call_gui_callback(h_state2.add_state, ex_state2)
    else:
        h_state2.add_state(ex_state2)

    logger.info("DO_TYPE_CHANGE")
    if with_gui:
        print h_state1.get_path()
        h_state1_m = sm_m.get_state_model_by_path(h_state1.get_path())
        call_gui_callback(do_type_change, sm_m, h_state1_m, 'EXECUTION', logger)
        # call_gui_callback(sm_m.state_machine.root_state.change_state_type, h_state1, ExecutionState)
    else:
        sm_m.state_machine.root_state.change_state_type(h_state1, ExecutionState)

    logger.info("UNDO \n{0}".format(sm_m.history.modifications.single_trail_history()[-1].before_overview))
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    # save_state_machine(sm_m, state_machine_path + '_before1', logger, with_gui, menubar_ctrl)
    logger.info("UNDO finished")
    logger.info("REDO")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    logger.info("REDO finished")

    check_elements_ignores.remove("internal_transitions")
    print check_elements_ignores


def trigger_multiple_undo_redo_bug_tests(with_gui=False):
    import rafcon.gui.singleton
    sm = StateMachine(HierarchyState())
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, sm)
    call_gui_callback(rafcon.core.singleton.state_machine_manager.__setattr__,
                      "active_state_machine_id", sm.state_machine_id)
    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines.values()[-1]

    call_gui_callback(sm_m.selection.set, [sm_m.root_state])

    try:
        import time
        import pykeyboard

        keyboard = pykeyboard.PyKeyboard()

        def press_key(characters, duration=0.05):
            assert all([isinstance(character, (int, str)) for character in characters])
            assert isinstance(duration, (int, float))
            for character in characters:
                print "press_key: ", character
                keyboard.press_key(character=character)
            print "for {0} seconds".format(duration)
            time.sleep(duration)
            for character in characters:
                print "release_key: ", character
                keyboard.release_key(character=character)

        sm_id = sm_m.state_machine.state_machine_id
        state_machines_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')
        state_machines_editor_ctrl.get_controller(sm_id).view.get_top_widget().grab_focus()
        state_machines_editor_ctrl.get_controller(sm_id).view.editor.grab_focus()
        press_key([keyboard.control_l_key, 'a'], duration=0.8)
        time.sleep(0.1)  # wait that last add is fully done
        assert sm_m.history.modifications.single_trail_history()
        press_key([keyboard.control_l_key, 'z'], duration=0.8)
        press_key([keyboard.control_l_key, keyboard.shift_l_key, 'z'], duration=0.8)
    except ImportError as e:
        print "ERROR: ", e
        # TODO finish this test and make a better raise or error here


if __name__ == '__main__':
    testing_utils.dummy_gui(None)
    # test_add_remove_history(None)
    # test_state_property_modifications_history(None)
    #
    # test_outcome_property_modifications_history(None)
    # test_input_port_modify_notification(None)
    # test_output_port_modify_notification(None)
    #
    # test_transition_property_modifications_history(None)
    # test_scoped_variable_modify_notification(None)
    # test_data_flow_property_modifications_history(None)

    test_state_machine_modifications_with_gui(with_gui=True, caplog=None)
    # test_state_type_change_bugs_with_gui(with_gui=False, caplog=None)
    # test_state_type_change_bugs_with_gui(with_gui=True, caplog=None)
    # test_multiple_undo_redo_bug_with_gui(None)
    # pytest.main(['-xs', __file__])
