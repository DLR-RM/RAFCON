from __future__ import print_function
from __future__ import absolute_import
from builtins import str
import os
import logging
import threading
import time
import pytest

# core elements
import rafcon.core.singleton
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.state import State
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.storage import storage

# general tool elements
from rafcon.utils import log

logger = log.get_logger(__name__)
logger.setLevel(logging.VERBOSE)

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback
from tests.gui.widget.test_state_type_change import store_state_elements, check_state_elements, \
     check_list_ES, check_list_HS, check_list_BCS, check_list_PCS, \
     check_list_root_ES, check_list_root_HS, check_list_root_BCS, check_list_root_PCS, \
     get_state_editor_ctrl_and_store_id_dict, check_elements_ignores
from tests.gui.widget.test_states_editor import check_state_editor_models
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
            for state_id, child_state in state.states.items():
                print(child_state.get_path())
                print_states(child_state)
    print_states(sm_model.state_machine.root_state)

    print("do SAVING OF STATEMACHINE")
    if with_gui:
        call_gui_callback(sm_model.state_machine.__setattr__, "file_system_path", path)
        print("by Menubar_ctrl")
        call_gui_callback(menubar_ctrl.on_save_activate, None)
    else:
        sm_model.state_machine.file_system_path = path
        print("by Function")
        on_save_activate(sm_model, logger)

    from .test_storage import check_that_all_files_are_there

    if with_gui:
        call_gui_callback(check_that_all_files_are_there, sm_model.state_machine, path, False, True)
    else:
        check_that_all_files_are_there(sm_model.state_machine, path, False, True)


def create_state_machine():
    state_machine_path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "history_test"))
    state_machine = storage.load_state_machine_from_path(state_machine_path)
    state_dict = {
        'Container': state_machine.get_state_by_path("CONT2"),
        'State1': state_machine.get_state_by_path("CONT2/STATE1"),
        'State2': state_machine.get_state_by_path("CONT2/STATE2"),
        'State3': state_machine.get_state_by_path("CONT2/STATE3"),
        'Nested': state_machine.get_state_by_path("CONT2/STATE3/NESTED"),
        'Nested2': state_machine.get_state_by_path("CONT2/STATE3/NESTED2")
    }
    return state_machine, state_dict


def prepare_state_machine_model(state_machine):
    import rafcon.gui.singleton
    rafcon.gui.singleton.state_machine_manager_model.state_machine_manager.add_state_machine(state_machine)
    testing_utils.wait_for_gui()
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = state_machine.state_machine_id

    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    sm_m.history.fake = False
    print("with_verbose is: ", sm_m.history.with_verbose)
    sm_m.history.with_verbose = False
    return sm_m


def create_state_machine_m(with_gui=False):
    if with_gui:
        state_machine, state_dict = call_gui_callback(create_state_machine)
        state_machine_m = call_gui_callback(prepare_state_machine_model, state_machine)
    else:
        state_machine, state_dict = create_state_machine()
        state_machine_m = prepare_state_machine_model(state_machine)
    return state_machine_m, state_dict


def perform_history_action(operation, *args, **kwargs):
    """Perform an operation on a state + undo and redo it with consistency checks

    :param operation: The operation to be called
    :param args: The arguments for the operation
    :param kwargs: The keyword arguments for the operation
    :return: the result of the operation and the new state reference
    rtype: tuple(any, rafcon.core.states.state.State)
    """
    state = operation.__self__
    if not isinstance(state, State):
        state = state.parent
    state_path = state.get_path()
    parent = state.parent
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    sm_history = state_machine_m.history
    history_length = len(sm_history.modifications.single_trail_history())
    origin_hash = parent.mutable_hash().hexdigest()
    additional_operations = 0
    if "additional_operations" in kwargs:
        additional_operations = kwargs["additional_operations"]
        del kwargs["additional_operations"]
    operation_result = operation(*args, **kwargs)
    assert len(sm_history.modifications.single_trail_history()) == history_length + 1 + additional_operations
    after_operation_hash = parent.mutable_hash().hexdigest()
    for _ in range(1 + additional_operations):
        sm_history.undo()
    undo_operation_hash = parent.mutable_hash().hexdigest()
    for _ in range(1 + additional_operations):
        sm_history.redo()
    redo_operation_hash = parent.mutable_hash().hexdigest()
    assert origin_hash == undo_operation_hash
    assert after_operation_hash == redo_operation_hash
    return operation_result, state_machine_m.state_machine.get_state_by_path(state_path)


def perform_multiple_undo_redo(number):
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    sm_history = state_machine_m.history
    origin_hash = state_machine_m.mutable_hash().hexdigest()
    for _ in range(number):
        sm_history.undo()
    for _ in range(number):
        sm_history.redo()
    final_hash = state_machine_m.mutable_hash().hexdigest()
    assert origin_hash == final_hash


def get_state_by_name(state_name, state_path_dict):
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    return state_machine_m.state_machine.get_state_by_path(state_path_dict[state_name])


def get_state_model_by_name(state_name, state_path_dict):
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    return state_machine_m.get_state_model_by_path(state_path_dict[state_name])


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
    state_machine_m, state_dict = create_state_machine_m()

    state_machine_path = TEST_PATH + '_test_add_remove'
    save_state_machine(state_machine_m, state_machine_path + '_before', logger, with_gui=False, menubar_ctrl=None)

    sm_history = state_machine_m.history

    state1 = HierarchyState('state1', state_id='STATE1')
    state2 = ExecutionState('state2', state_id='STATE2')

    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    state_dict['state1'] = state1
    state_dict['state2'] = state2

    state_path_dict = {}
    for key in state_dict:
        state_path_dict[key] = state_dict[key].get_path()

    def do_check_for_state(state_name):
        sm_history.modifications.reset()

        # Note: The elements always need to be retrieved before performing an operation, as undo/redo operations replace
        # both core and model objects
        state_m = get_state_model_by_name(state_name, state_path_dict)

        #############
        # outcome add & remove
        outcome_super, state = perform_history_action(state_m.state.add_outcome, "super")

        state = get_state_by_name(state_name, state_path_dict)
        _, state = perform_history_action(state_m.state.remove_outcome, outcome_super)


        #############
        # add two states
        state4 = ExecutionState('State4', state_id='STATE4')
        state_path_dict['state4'] = state.get_path() + "/" + "STATE4"
        _, state = perform_history_action(state.add_state, state4)

        state5 = ExecutionState('State5', state_id='STATE5')
        state_path_dict['state5'] = state.get_path() + "/" + "STATE5"
        _, state = perform_history_action(state.add_state, state5)

        perform_multiple_undo_redo(2)

        state4 = get_state_by_name('state4', state_path_dict)
        outcome_state4 = state4.add_outcome('UsedHere')
        assert len(sm_history.modifications.single_trail_history()) == 6

        state5 = get_state_by_name('state5', state_path_dict)
        outcome_state5 = state5.add_outcome('UsedHere')
        assert len(sm_history.modifications.single_trail_history()) == 7

        ################
        # add transition from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id
        new_transition_id1, state = perform_history_action(state.add_transition,
                                                           from_state_id=state4.state_id, from_outcome=outcome_state4,
                                                           to_state_id=state5.state_id, to_outcome=None)
        _, state = perform_history_action(state.add_transition,
                                          from_state_id=state5.state_id, from_outcome=outcome_state5,
                                          to_state_id=state.state_id, to_outcome=-1)

        ###################
        # remove transition
        _, state = perform_history_action(state.remove_transition, new_transition_id1)

        #############
        # remove state
        _, state = perform_history_action(state.remove_state, state5.state_id)


        #############
        # add input_data_port
        state4 = get_state_by_name('state4', state_path_dict)
        input_state4_id, state4 = perform_history_action(state4.add_input_data_port, "input", "str", "zero")

        #############
        # remove input_data_port
        _, state4 = perform_history_action(state4.remove_input_data_port, input_state4_id)


        #############
        # add output_data_port
        output_state4_id, state4 = perform_history_action(state4.add_output_data_port, "output_"+state4.state_id, "int")

        #############
        # remove output_data_port
        _, state4 = perform_history_action(state4.remove_output_data_port, output_state4_id)


        # prepare again state4
        state4.add_output_data_port("output", "int")
        state4.add_input_data_port("input_new", "str", "zero")
        assert len(sm_history.modifications.single_trail_history()) == 17
        output_state4_id = state4.add_output_data_port("output_new", "int")
        assert len(sm_history.modifications.single_trail_history()) == 18

        state5 = ExecutionState('State5', 'STATE5')
        state = get_state_by_name(state_name, state_path_dict)
        state.add_state(state5)
        assert state_path_dict['state5'] == state5.get_path()
        assert len(sm_history.modifications.single_trail_history()) == 19
        input_par_state5 = state5.add_input_data_port("par", "int", 0)
        assert len(sm_history.modifications.single_trail_history()) == 20
        output_res_state5 = state5.add_output_data_port("res", "int")
        assert len(sm_history.modifications.single_trail_history()) == 21

        #####################
        # add scoped_variable
        scoped_buffer_nested, state = perform_history_action(state.add_scoped_variable, "buffer", "int")

        #####################
        # remove scoped_variable
        _, state = perform_history_action(state.remove_scoped_variable, scoped_buffer_nested)

        #############
        # add data_flow
        new_df_id, state = perform_history_action(state.add_data_flow,
                                                  from_state_id=state4.state_id, from_data_port_id=output_state4_id,
                                                  to_state_id=state5.state_id, to_data_port_id=input_par_state5)

        ################
        # remove data_flow
        perform_history_action(state.remove_data_flow, new_df_id)

    do_check_for_state(state_name='Nested')
    do_check_for_state(state_name='Container')

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
    sm_model, state_dict = create_state_machine_m()

    state1 = ExecutionState('State1', state_id="STATE1")
    state1.add_input_data_port("input", "str", "zero")
    state1.add_output_data_port("output", "int")
    state1.add_output_data_port("count", "int")

    state2 = ExecutionState('State2')
    state2.add_input_data_port("par", "int", 0)
    state2.add_input_data_port("number", "int", 5)
    state2.add_output_data_port("res", "int")

    nested_state = state_dict['Nested']
    nested_state_path = nested_state.get_path()
    nested_state.add_state(state1)
    nested_state.add_state(state2)
    state2_path = state2.get_path()
    nested_state.add_output_data_port("res", "int")

    state1.add_outcome("again")
    state1.add_outcome("counted")

    assert len(sm_model.history.modifications.single_trail_history()) == 6

    state2.add_outcome("done")
    state2.add_outcome("best")
    state2.add_outcome("full")
    assert len(sm_model.history.modifications.single_trail_history()) == 9

    nested_state.add_outcome("great")
    assert len(sm_model.history.modifications.single_trail_history()) == 10

    #######################################
    ######## Properties of State ##########

    # name(self, name)
    _, nested_state = perform_history_action(nested_state.__setattr__, "name", "nested")

    # TODO: The following commented operations are not correctly supported by the history!
    # input_data_ports(self, input_data_ports) None or dict
    # _, nested_state = perform_history_action(nested_state.__setattr__, "input_data_ports", {})
    # _, nested_state = perform_history_action(nested_state.__setattr__, "output_data_ports", {})

    # outcomes(self, outcomes) None or dict
    # _, nested_state = perform_history_action(nested_state.__setattr__, "outcomes", nested_state.outcomes)
    # _, nested_state = perform_history_action(nested_state.__setattr__, "outcomes", {})

    script_text = '\ndef execute(self, inputs, outputs, gvm):\n\tself.logger.debug("Hello World")\n\treturn 0\n'
    script_text1 = '\ndef execute(self, inputs, outputs, gvm):\n\tself.logger.debug("Hello NERD")\n\treturn 0\n'

    # script(self, script) Script -> no script setter any more only script_text !!!
    nested2_state = state_dict['Nested2']
    _, nested2_state = perform_history_action(nested2_state.__setattr__, "script_text", script_text)

    # script_text(self, script_text)
    _, nested2_state = perform_history_action(nested2_state.__setattr__, "script_text", script_text1)

    # description(self, description) str
    _, nested_state = perform_history_action(nested_state.__setattr__, "description", "awesome")

    ############################################
    ###### Properties of ContainerState ########

    # set_start_state(self, state) State or state_id
    _, nested_state = perform_history_action(nested_state.set_start_state, "STATE1")

    # set_start_state(self, start_state)
    state2 = sm_model.state_machine.get_state_by_path(state2_path)
    _, nested_state = perform_history_action(nested_state.set_start_state, state2, additional_operations=1)

    # transitions(self, transitions) None or dict
    _, nested_state = perform_history_action(nested_state.__setattr__, "transitions", {})

    # data_flows(self, data_flows) None or dict
    _, nested_state = perform_history_action(nested_state.__setattr__, "data_flows", {})

    # scoped_variables(self, scoped_variables) None or dict
    _, nested_state = perform_history_action(nested_state.__setattr__, "scoped_variables", {})

    # states(self, states) None or dict
    _, nested_state = perform_history_action(nested_state.__setattr__, "states", {})

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_outcome_property_modifications_history(caplog):
    ##################
    # outcome properties

    # change name

    # create testbed

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    sm_model, state_dict = create_state_machine_m()

    ####################################################
    # modify outcome and generate in previous a observer
    for state_name in ["Nested", "Nested2"]:
        state_path = state_dict[state_name].get_path()
        outcome_ids = list(state_dict[state_name].outcomes.keys())
        for outcome_id in outcome_ids:
            state = sm_model.state_machine.get_state_by_path(state_path)
            if outcome_id >= 0:
                outcome = state.outcomes[outcome_id]
                _, _ = perform_history_action(outcome.__setattr__, "name", "new_name_" + str(outcome_id))

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


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

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    sm_model, state_dict = create_state_machine_m()

    state1 = ExecutionState('State1')
    outcome_again_state1 = state1.add_outcome("again")
    state2 = ExecutionState('State2')
    oc_done_state2 = state2.add_outcome("done")
    state2.add_outcome("best")
    nested_state = state_dict['Nested']
    nested_state.add_state(state1)
    nested_state.add_state(state2)
    nested_state.add_outcome("great")
    state1.add_outcome("counted")
    oc_full_state2 = state2.add_outcome("full")

    new_trans_id, nested_state = perform_history_action(nested_state.add_transition,
                                                        from_state_id=state1.state_id, from_outcome=outcome_again_state1,
                                                        to_state_id=state1.state_id, to_outcome=None)

    # modify_origin(self, from_state, from_outcome)
    _, nested_state = perform_history_action(nested_state.transitions[new_trans_id].modify_origin,
                                             from_state=state2.state_id, from_outcome=oc_full_state2)

    # from_outcome(self, from_outcome)
    _, nested_state = perform_history_action(nested_state.transitions[new_trans_id].__setattr__, "from_outcome",
                                             oc_done_state2)

    # to_state(self, to_state)
    _, nested_state = perform_history_action(nested_state.transitions[new_trans_id].__setattr__, "to_state",
                                             state2.state_id)

    # reset observer and testbed
    _, nested_state = perform_history_action(nested_state.remove_transition, new_trans_id)
    new_df_id, nested_state = perform_history_action(nested_state.add_transition,
                                                     from_state_id=state1.state_id, from_outcome=outcome_again_state1,
                                                     to_state_id=state1.state_id, to_outcome=None)

    _, nested_state = perform_history_action(nested_state.transitions[new_df_id].modify_origin,
                                             state2.state_id, oc_full_state2)

    # modify_transition_from_outcome(self, transition_id, from_outcome)
    _, nested_state = perform_history_action(nested_state.transitions[new_df_id].__setattr__, "from_outcome",
                                             oc_done_state2)

    # modify_transition_to_state(self, transition_id, to_state, to_outcome)
    _, nested_state = perform_history_action(nested_state.transitions[new_df_id].__setattr__, "to_state",
                                             state1.state_id)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_input_port_modify(caplog):
    ##################
    # input_data_port properties

    # change name
    # change data_type
    # change default_value
    # change datatype

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False,
                                                     'HISTORY_ENABLED': True}, gui_already_started=False)
    sm_model, state_dict = create_state_machine_m()
    nested_state = state_dict['Nested2']

    new_input_data_port_id, nested_state = perform_history_action(nested_state.add_input_data_port,
                                                                  name='new_input', data_type='str')

    ################################
    # check for modification of name
    _, nested_state = perform_history_action(nested_state.input_data_ports[new_input_data_port_id].__setattr__, "name",
                                             "changed_new_input_name")

    #####################################
    # check for modification of data_type
    _, nested_state = perform_history_action(nested_state.input_data_ports[new_input_data_port_id].__setattr__,
                                             "data_type", "int")

    #########################################
    # check for modification of default_value
    _, nested_state = perform_history_action(nested_state.input_data_ports[new_input_data_port_id].__setattr__,
                                             "default_value", 5)

    ###########################################
    # check for modification of change_datatype
    _, nested_state = perform_history_action(nested_state.input_data_ports[new_input_data_port_id].change_data_type,
                                             data_type='str', default_value='awesome_tool')

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_output_port_modify(caplog):

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
    sm_model, state_dict = create_state_machine_m()
    nested_state = state_dict['Nested2']

    new_output_data_port_id, nested_state = perform_history_action(nested_state.add_output_data_port,
                                                                   name='new_output', data_type='str')

    ################################
    # check for modification of name
    _, nested_state = perform_history_action(nested_state.output_data_ports[new_output_data_port_id].__setattr__, "name",
                                             "changed_new_output_name")

    #####################################
    # check for modification of data_type
    _, nested_state = perform_history_action(nested_state.output_data_ports[new_output_data_port_id].__setattr__,
                                             "data_type", "int")

    #########################################
    # check for modification of default_value
    _, nested_state = perform_history_action(nested_state.output_data_ports[new_output_data_port_id].__setattr__,
                                             "default_value", 5)

    ###########################################
    # check for modification of change_datatype
    _, nested_state = perform_history_action(nested_state.output_data_ports[new_output_data_port_id].change_data_type,
                                             data_type='str', default_value='awesome_tool')


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
    sm_model, state_dict = create_state_machine_m()

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
    sm_model, state_dict = create_state_machine_m()

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


@pytest.mark.timeout(60)
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
        print("start thread")
        trigger_state_type_change_tests(with_gui)
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

    print("FINISH test_state_machine_modifications_with_gui", with_gui)


@pytest.mark.parametrize("with_gui", [True])
def test_state_type_change_bugs_with_gui(with_gui, caplog):

    testing_utils.dummy_gui(None)

    if with_gui:
        testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True})
        e = None
        try:
            trigger_state_type_change_typical_bug_tests(with_gui=True)
        except:
            raise  # required, otherwise the exception cannot be accessed within finally
        finally:
            testing_utils.close_gui()
            testing_utils.shutdown_environment(caplog=caplog)
    else:
        testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': True},
                                             gui_already_started=False)
        trigger_state_type_change_typical_bug_tests(with_gui)
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

    print("FINISH", test_state_type_change_bugs_with_gui, "with_gui", with_gui)


def test_multiple_undo_redo_bug_with_gui(caplog):

    testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': True, 'HISTORY_ENABLED': True})
    try:
        trigger_multiple_undo_redo_bug_tests(with_gui=True)
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


def do_type_change(target_sm_m, target_state_m, drop_down_combo_label_of_new_target_state_type, logger=None):
    import rafcon.gui.singleton
    main_window_controller = rafcon.gui.singleton.main_window_controller
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(target_sm_m, target_state_m, main_window_controller, 5., logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict[drop_down_combo_label_of_new_target_state_type]
    state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active(state_type_row_id)


def trigger_state_type_change_tests(with_gui):
    import rafcon.gui.singleton
    import rafcon.gui.helpers.state as gui_helper_state
    main_window_controller = rafcon.gui.singleton.main_window_controller

    sm_m, state_dict = create_state_machine_m(with_gui)
    sleep_time_max = 5  # 0.5

    check_elements_ignores.append("internal_transitions")

    # General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    parent_of_type_change = 'Container'
    path_of_state_of_type_change = state_dict[state_of_type_change].get_path()
    path_of_parent_of_type_change = state_dict[parent_of_type_change].get_path()

    print("start 1")
    # do state_type_change with gui
    if with_gui:
        call_gui_callback(sm_m.history.modifications.reset)
    else:
        sm_m.history.modifications.reset()
    print("start 2")

    state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    print("\n\n %s \n\n" % state_m.state.name)
    if with_gui:
        call_gui_callback(sm_m.selection.set, [state_m])
    else:
        sm_m.selection.set([state_m])

    list_store_id_from_state_type_dict = {}
    state_editor_ctrl = None
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    state_machine_path = TEST_PATH + "_state_type_change_{0}".format("with_gui" if with_gui else "without_gui")
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller') if with_gui else None
    save_state_machine(sm_m, state_machine_path, logger, with_gui, menubar_ctrl)
    print("CHECK state_of_type_change path", state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, state_m, logger)

    # HS -> BCS
    save_state_machine(sm_m, state_machine_path + '_before1', logger, with_gui, menubar_ctrl)
    # - do state type change
    logger.info("HS -> BCS")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, state_m, BarrierConcurrencyState.__name__, logger)
    else:
        gui_helper_state.change_state_type(state_m, BarrierConcurrencyState)
        # state_dict[parent_of_type_change].change_state_type(state_m.state, BarrierConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)
    print("CHECK state_of_type_change path", state_dict[state_of_type_change].get_path())
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
    print("CHECK state_of_type_change path", state_dict[state_of_type_change].get_path())
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    save_state_machine(sm_m, state_machine_path + '_undo1', logger, with_gui, menubar_ctrl)
    print("CHECK state_of_type_change path", state_dict[state_of_type_change].get_path())
    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)
    print("CHECK state_of_type_change path", state_dict[state_of_type_change].get_path())
    logger.info("HS -> BCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    save_state_machine(sm_m, state_machine_path + '_redo1', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_BCS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # BCS -> HS
    save_state_machine(sm_m, state_machine_path + '_before2', logger, with_gui, menubar_ctrl)
    logger.info("BCS -> HS 2")
    if with_gui:
        call_gui_callback(sm_m.state_machine.__setattr__, "file_system_path", state_machine_path)
        call_gui_callback(do_type_change, sm_m, new_state_m, HierarchyState.__name__, logger)
    else:
        sm_m.state_machine.file_system_path = state_machine_path
        gui_helper_state.change_state_type(new_state_m, HierarchyState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    print("\n\n ###### State: %s" % state_dict[state_of_type_change])
    from .test_storage import check_that_all_files_are_there
    check_that_all_files_are_there(sm_m.state_machine, state_machine_path + '_before2', False, True)
    check_that_all_files_are_there(sm_m.state_machine, state_machine_path + '_after2', False, True)

    save_state_machine(sm_m, state_machine_path + '_after2', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 3
    logger.info("HS -> BCS 2 (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo2', logger, with_gui, menubar_ctrl)

    logger.info("BCS -> HS 2 (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    save_state_machine(sm_m, state_machine_path + '_redo2', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_m, state_machine_path + '_before3', logger, with_gui, menubar_ctrl)
    logger.info("HS -> PCS")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, PreemptiveConcurrencyState.__name__, logger)
    else:
        gui_helper_state.change_state_type(new_state_m, PreemptiveConcurrencyState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, PreemptiveConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_m, state_machine_path + '_after3', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 4
    logger.info("PCS -> HS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo3', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    logger.info("HS -> PCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

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
        call_gui_callback(do_type_change, sm_m, new_state_m, ExecutionState.__name__, logger)
    else:
        gui_helper_state.change_state_type(new_state_m, ExecutionState)
        # state_dict[parent_of_type_change].change_state_type(new_state_m.state, ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_m, state_machine_path + '_after4', logger, with_gui, menubar_ctrl)

    assert len(sm_m.history.modifications.single_trail_history()) == 5
    logger.info("ES -> PCS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    save_state_machine(sm_m, state_machine_path + '_undo4', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    logger.info("PCS -> ES (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_parent_of_type_change)

    save_state_machine(sm_m, state_machine_path + '_redo4', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_ES, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    path_of_state_of_type_change = state_dict[state_of_type_change].get_path()
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, state_m, logger)

    # HS -> BCS
    logger.info("HS -> BCS root")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, state_m, BarrierConcurrencyState.__name__, logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, BarrierConcurrencyState)
        # sm_m.state_machine.change_root_state_type(BarrierConcurrencyState)
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    # import time
    # time.sleep(100)

    assert len(sm_m.history.modifications.single_trail_history()) == 6
    logger.info("BCS -> HS root (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("HS -> BCS root (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_BCS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # BCS -> HS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("BCS -> HS root 2")
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, HierarchyState.__name__, logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, HierarchyState)
        # sm_m.state_machine.change_root_state_type(HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    assert len(sm_m.history.modifications.single_trail_history()) == 7
    logger.info("HS -> BCS root 2 (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_BCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("BCS -> HS root 2 (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("HS -> PCS")
    # - do state type change
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, PreemptiveConcurrencyState.__name__, logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, PreemptiveConcurrencyState)
        # sm_m.state_machine.change_root_state_type(PreemptiveConcurrencyState)
        state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    assert len(sm_m.history.modifications.single_trail_history()) == 8
    logger.info("PCS -> HS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("HS -> PCS (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    # PCS -> ES
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    logger.info("PCS -> ES")
    # - do state type change
    if with_gui:
        call_gui_callback(do_type_change, sm_m, new_state_m, ExecutionState.__name__, logger)
    else:
        gui_helper_state.change_state_type(sm_m.root_state, ExecutionState)
        # sm_m.state_machine.change_root_state_type(ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    assert len(sm_m.history.modifications.single_trail_history()) == 9
    logger.info("ES -> PCS (undo)")
    if with_gui:
        call_gui_callback(sm_m.history.undo)
    else:
        sm_m.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    # print "path: ", state_dict[state_of_type_change].get_path(), " state ", state_dict[state_of_type_change]
    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        call_gui_callback(check_state_editor_models, sm_m, new_state_m, logger)

    logger.info("PCS -> ES (redo)")
    if with_gui:
        call_gui_callback(sm_m.history.redo)
    else:
        sm_m.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)

    new_state = sm_m.state_machine.get_state_by_path(path_of_state_of_type_change)
    new_state_m = sm_m.get_state_model_by_path(path_of_state_of_type_change)
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
    print(check_elements_ignores)


def trigger_state_type_change_typical_bug_tests(with_gui):
    import rafcon.gui.singleton
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller

    sm_m, state_dict = create_state_machine_m(with_gui)

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
    print("\n\n %s \n\n" % state_m.state.name)
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
        print(h_state1.get_path())
        h_state1_m = sm_m.get_state_model_by_path(h_state1.get_path())
        call_gui_callback(do_type_change, sm_m, h_state1_m, ExecutionState.__name__, logger)
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
    print(check_elements_ignores)


def trigger_multiple_undo_redo_bug_tests(with_gui=False):
    import rafcon.gui.singleton
    sm = StateMachine(HierarchyState())
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, sm)
    sm_m = list(rafcon.gui.singleton.state_machine_manager_model.state_machines.values())[-1]
    call_gui_callback(sm_m.selection.set, [sm_m.root_state])

    try:
        from keyboard_utils import press_key, keyboard

        sm_id = sm_m.state_machine.state_machine_id
        state_machines_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')
        state_machines_editor_ctrl.get_controller(sm_id).view.get_top_widget().grab_focus()
        state_machines_editor_ctrl.get_controller(sm_id).view.editor.grab_focus()
        press_key([keyboard.control_l_key, 'a'], duration=0.6)
        call_gui_callback(testing_utils.wait_for_gui)  # wait that last add is fully done
        assert sm_m.history.modifications.single_trail_history()
        press_key([keyboard.control_l_key, 'z'], duration=1.2)
        call_gui_callback(testing_utils.wait_for_gui)  # wait that last undo is fully done
        press_key([keyboard.control_l_key, keyboard.shift_l_key, 'z'], duration=1.2)
        call_gui_callback(testing_utils.wait_for_gui)  # wait that last redo is fully done
    except ImportError as e:
        print("ERROR: ", e)
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

    # test_state_machine_modifications_with_gui(with_gui=True, caplog=None)
    # test_state_type_change_bugs_with_gui(with_gui=False, caplog=None)
    # test_state_type_change_bugs_with_gui(with_gui=True, caplog=None)
    test_multiple_undo_redo_bug_with_gui(None)
    # pytest.main(['-xs', __file__])
