import sys
import logging
import gtk
import threading
import time
import glib
import os
import signal

from rafcon.utils import log
from rafcon.mvc.models import ContainerStateModel, StateModel, GlobalVariableManagerModel
from rafcon.mvc.controllers import MainWindowController, StateDataPortEditorController,\
    SingleWidgetWindowController, SourceEditorController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.views import LoggingView, StateDataportEditorView, SingleWidgetWindowView, SourceEditorView
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
import rafcon.mvc.singleton
from rafcon.statemachine.state_machine import StateMachine
import variables_for_pytest

from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID

from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config


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
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)

    # remove existing state machines
        rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm_in.state_machine_id)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)
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


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def store_state_elements(state, state_m):
    """Stores all ids of elements in or outside of the actual state"""
    print "STORE state elements of %s, %s" % (state.name, state_m.state.name)

    state_elements = {}
    state_m_elements = {}
    state_elements['name'] = state.name
    state_elements['path'] = state.get_path()
    state_m_elements['name'] = state_m.state.name
    state_m_elements['path'] = state_m.state.get_path()
    # collect input_data_ports
    state_elements['input_data_ports'] = []
    for p_id, p in state.input_data_ports.iteritems():
        print "input ports", p_id, state.input_data_ports.keys()
        state_elements['input_data_ports'].append(p_id)
    # - check if the right models are there and only those
    model_id_store = []
    state_m_elements['input_data_ports_meta'] = {}
    for p_m in state_m.input_data_ports:
        assert p_m.data_port.data_port_id in state_elements['input_data_ports']
        model_id_store.append(p_m.data_port.data_port_id)
        # - store model meta data
        state_m_elements['input_data_ports_meta'][p_m.data_port.data_port_id] = p_m.meta
    for p_id, p in state.input_data_ports.iteritems():
        assert p_id in model_id_store
    # print state_elements['input_data_ports'], state.input_data_ports

    # collect output_data_ports
    state_elements['output_data_ports'] = []
    for p_id, p in state.output_data_ports.iteritems():
        state_elements['output_data_ports'].append(p_id)
    # - check if the right models are there and only those
    model_id_store = []
    state_m_elements['output_data_ports_meta'] = {}
    for p_m in state_m.output_data_ports:
        assert p_m.data_port.data_port_id in state_elements['output_data_ports']
        model_id_store.append(p_m.data_port.data_port_id)
        # - store model meta data
        state_m_elements['output_data_ports_meta'][p_m.data_port.data_port_id] = p_m.meta
    for p_id, p in state.output_data_ports.iteritems():
        assert p_id in model_id_store
    # print state_elements['output_data_ports'], state.output_data_ports
    # print state.name
    # exit(0)

    # collect outcomes
    state_elements['outcomes'] = []
    for oc_id, oc, in state.outcomes.iteritems():
        state_elements['outcomes'].append(oc_id)
    # - check if the right models are there and only those
    model_id_store = []
    state_m_elements['outcomes_meta'] = {}
    for oc_m in state_m.outcomes:
        assert oc_m.outcome.outcome_id in state_elements['outcomes']
        model_id_store.append(oc_m.outcome.outcome_id)
        # - store model meta data
        state_m_elements['outcomes_meta'][oc_m.outcome.outcome_id] = oc_m.meta
    for oc_id, oc in state.outcomes.iteritems():
        assert oc_id in model_id_store
    # print state_elements['outcomes'], state.outcomes

    # collect scoped_variables
    if hasattr(state, 'scoped_variables'):
        state_elements['scoped_variables'] = []
        for sv_id, sv, in state.scoped_variables.iteritems():
            state_elements['scoped_variables'].append(sv_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['scoped_variables_meta'] = {}
        for sv_m in state_m.scoped_variables:
            assert sv_m.scoped_variable.data_port_id in state_elements['scoped_variables']
            model_id_store.append(sv_m.scoped_variable.data_port_id)
            # - store model meta data
            state_m_elements['scoped_variables_meta'][sv_m.scoped_variable.data_port_id] = sv_m.meta
        for sv_id, sv in state.scoped_variables.iteritems():
            assert sv_id in model_id_store

    # collect states
    if hasattr(state, 'states'):
        state_elements['states'] = []
        for s_id, s in state.states.iteritems():
            state_elements['states'].append(s_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['states_meta'] = {}
        for s_m_id, s_m in state_m.states.iteritems():
            if not hasattr(s_m, "state"):
                print s_m
            assert s_m_id == s_m.state.state_id
            assert s_m_id in state_elements['states']
            assert s_m.state.state_id in state_elements['states']
            model_id_store.append(s_m.state.state_id)
            # - store model meta data
            state_m_elements['states_meta'][s_m.state.state_id] = s_m.meta
        for s_id, s in state.states.iteritems():
            assert s_id in model_id_store

    # collect data_flows
    if hasattr(state, 'data_flows'):
        state_elements['data_flows'] = []
        for df_id, df in state.data_flows.iteritems():
            state_elements['data_flows'].append(df_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['data_flows_meta'] = {}
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in state_elements['data_flows']
            model_id_store.append(df_m.data_flow.data_flow_id)
            # - store model meta data
            state_m_elements['data_flows_meta'][df_m.data_flow.data_flow_id] = df_m.meta
        for df_id, df in state.data_flows.iteritems():
            assert df_id in model_id_store

    # collect transitions
    if hasattr(state, 'transitions'):
        state_elements['transitions'] = []
        for t_id, t in state.transitions.iteritems():
            state_elements['transitions'].append(t_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['transitions_meta'] = {}
        for t_m in state_m.transitions:
            assert t_m.transition.transition_id in state_elements['transitions']
            model_id_store.append(t_m.transition.transition_id)
            # - store model meta data
            state_m_elements['transitions_meta'][t_m.transition.transition_id] = t_m.meta
        for t_id, t in state.transitions.iteritems():
            if not UNIQUE_DECIDER_STATE_ID in [t.to_state, t.from_state]:  # TODO test needs to be improved to cover BarrierState, too
                assert t_id in model_id_store

    def is_related_transition(parent, state_id, t):
        return t.from_state == state_id or t.to_state == state_id

    def is_related_data_flow(parent, state_id, df):
        return df.from_state == state_id or df.to_state == state_id

    # LOOKOUT: root states have their statemachine as parent
    if hasattr(state, 'parent') and state.parent is not None and isinstance(state.parent, State):
        # collect transitions of parent related and not related to me
        state_elements['transitions_external'] = []
        state_elements['transitions_external_not_related'] = []
        for t_id, t in state.parent.transitions.iteritems():
            if is_related_transition(state.parent, state.state_id, t):
                state_elements['transitions_external'].append(t_id)
            else:
                state_elements['transitions_external_not_related'].append(t_id)
        state_m_elements['transitions_external'] = []
        state_m_elements['transitions_external_not_related'] = []
        state_m_elements['transitions_external_meta'] = {}
        state_m_elements['transitions_external_not_related_meta'] = {}
        for t_m in state_m.parent.transitions:
            t_id = t_m.transition.transition_id
            t = t_m.transition
            if is_related_transition(state.parent, state.state_id, t):
                state_m_elements['transitions_external'].append(t_id)
                assert t_m.transition.transition_id in state_elements['transitions_external']
                state_m_elements['transitions_external_meta'][t_id] = t_m.meta
            else:
                state_m_elements['transitions_external_not_related'].append(t_id)
                state_m_elements['transitions_external_not_related_meta'][t_id] = t_m.meta

        # collect data flows of parent related and not related to me
        state_elements['data_flows_external'] = []
        state_elements['data_flows_external_not_related'] = []
        for df_id, df in state.parent.data_flows.iteritems():
            if is_related_data_flow(state.parent, state.state_id, df):
                state_elements['data_flows_external'].append(df_id)
            else:
                state_elements['data_flows_external_not_related'].append(df_id)

        state_m_elements['data_flows_external'] = []
        state_m_elements['data_flows_external_not_related'] = []
        state_m_elements['data_flows_external_meta'] = {}
        state_m_elements['data_flows_external_not_related_meta'] = {}
        for df_m in state_m.parent.data_flows:
            df_id = df_m.data_flow.data_flow_id
            df = df_m.data_flow
            if is_related_data_flow(state.parent, state.state_id, df):
                state_m_elements['data_flows_external'].append(df_id)
                assert df_m.data_flow.data_flow_id in state_elements['data_flows_external']
                state_m_elements['data_flows_external_meta'][df_id] = t_m.meta
            else:
                state_m_elements['data_flows_external_not_related'].append(df_id)
                state_m_elements['data_flows_external_not_related_meta'][df_id] = t_m.meta
    else:
        print "STATE is a root_state"

    return state_elements, state_m_elements


def check_state_elements(check_list, state, state_m, stored_state_elements, stored_state_m_elements):
    print "CHECK state elements of %s, %s " % (state.name, state_m.state.name)
    print "AGAINST stored elements of %s, %s " % (stored_state_elements['name'],
                                                  stored_state_m_elements['name'])
    # check ports
    if 'ports' in check_list:
        # collect input_data_ports
        for p_id, p in state.input_data_ports.iteritems():
            print state.state_id, p_id, stored_state_elements['input_data_ports']
            assert p_id in stored_state_elements['input_data_ports']
        # - check if the right models are there and only those
        model_id_store = []
        for p_m in state_m.input_data_ports:
            assert p_m.data_port.data_port_id in stored_state_elements['input_data_ports']
            model_id_store.append(p_m.data_port.data_port_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['input_data_ports_meta'][p_m.data_port.data_port_id] == p_m.meta
        for p_id in stored_state_elements['input_data_ports']:
            assert p_id in model_id_store

        # collect output_data_ports
        for p_id, p in state.output_data_ports.iteritems():
            assert p_id in stored_state_elements['output_data_ports']
        # - check if the right models are there and only those
        model_id_store = []
        for p_m in state_m.output_data_ports:
            assert p_m.data_port.data_port_id in stored_state_elements['output_data_ports']
            model_id_store.append(p_m.data_port.data_port_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['output_data_ports_meta'][p_m.data_port.data_port_id] == p_m.meta
        for p_id in stored_state_elements['output_data_ports']:
            assert p_id in model_id_store

    # check outcomes
    if 'outcomes' in check_list:
        # collect outcomes
        for oc_id, oc, in state.outcomes.iteritems():
            # print oc_id, stored_state_elements['outcomes']
            assert oc_id in stored_state_elements['outcomes']
        # - check if the right models are there and only those
        model_id_store = []
        for oc_m in state_m.outcomes:
            assert oc_m.outcome.outcome_id in stored_state_elements['outcomes']
            model_id_store.append(oc_m.outcome.outcome_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['outcomes_meta'][oc_m.outcome.outcome_id] == oc_m.meta
        for oc_id in stored_state_elements['outcomes']:
            assert oc_id in model_id_store

    # check states
    if 'states' in check_list and hasattr(state, "states"):  # TODO last element of condition has to be deleted again
        for s_id, s in state.states.iteritems():
            if not s_id == UNIQUE_DECIDER_STATE_ID:
                assert s_id in stored_state_elements['states']
        # - check if the right models are there and only those
        model_id_store = []
        for s_m_id, s_m in state_m.states.iteritems():
            if not hasattr(s_m, "state"):
                print s_m
            assert s_m_id == s_m.state.state_id
            if not s_m_id == UNIQUE_DECIDER_STATE_ID:
                if not s_m_id in stored_state_elements['states']:
                    print "missing state: ", s_m_id, stored_state_elements['states']
                assert s_m_id in stored_state_elements['states']

                assert s_m.state.state_id in stored_state_elements['states']
                model_id_store.append(s_m.state.state_id)
                # - check if meta data is still the same
                print stored_state_m_elements['states_meta'][s_m.state.state_id], s_m.meta
                assert stored_state_m_elements['states_meta'][s_m.state.state_id] == s_m.meta
        for s_id in stored_state_elements['states']:
            if not s_id == UNIQUE_DECIDER_STATE_ID:
                assert s_id in model_id_store
    else:
        if hasattr(state, 'states'):
            print state, state.states
        assert not hasattr(state, 'states')
    # exit(0)

    # check scoped_variables
    if 'scoped_variables' in check_list and hasattr(state, 'scoped_variables'):  # TODO last element of condition has to be deleted again:
        for sv_id, sv, in state.scoped_variables.iteritems():
            assert sv_id in stored_state_elements['scoped_variables']
        # - check if the right models are there and only those
        model_id_store = []
        for sv_m in state_m.scoped_variables:
            assert sv_m.scoped_variable.data_port_id in stored_state_elements['scoped_variables']
            model_id_store.append(sv_m.scoped_variable.data_port_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['scoped_variables_meta'][sv_m.scoped_variable.data_port_id] == sv_m.meta
        for sv_id in stored_state_elements['scoped_variables']:
            assert sv_id in model_id_store
    else:
        assert not hasattr(state, 'scoped_variables')

    # # check transitions internal
    # if 'transitions_internal' in check_list:
    #     for t_id, t in state.transitions.iteritems():
    #         print state.name, t_id, stored_state_elements['transitions'], state.transitions.keys()
    #         assert t_id in stored_state_elements['transitions']
    #     # - check if the right models are there and only those
    #     model_id_store = []
    #     for t_m in state_m.transitions:
    #         assert t_m.transition.transition_id in stored_state_elements['transitions']
    #         model_id_store.append(t_m.transition.transition_id)
    #         # - check if meta data is still the same
    #         assert stored_state_m_elements['transitions_meta'][t_m.transition.transition_id] == t_m.meta
    #     for t_id in stored_state_elements['transitions']:
    #         assert t_id in model_id_store
    # else:
    #     assert not hasattr(state, 'transitions')
    #
    # def is_related_transition(parent, state_id, t):
    #     return t.from_state == state_id or t.to_state == state_id
    #
    # # check transitions external
    # if 'transitions_external' in check_list:
    #     for t_id, t in state.parent.transitions.iteritems():
    #         if is_related_transition(state.parent, state.state_id, t):
    #             assert t_id in stored_state_elements['transitions_external']
    #         else:
    #             assert stored_state_elements['transitions_external_not_related']
    #
    #     for t_m in state_m.parent.transitions:
    #         t_id = t_m.transition.transition_id
    #         t = t_m.transition
    #         if is_related_transition(state.parent, state.state_id, t):
    #             assert t_id in stored_state_m_elements['transitions_external']
    #             # - check if meta data is still the same
    #             assert stored_state_m_elements['transitions_external_meta'][t_id] == t_m.meta
    #         else:
    #             assert t_id in stored_state_m_elements['transitions_external_not_related']
    #             # - check if meta data is still the same
    #             assert stored_state_m_elements['transitions_external_not_related_meta'][t_id] == t_m.meta
    # else:
    #     assert state.parent is None

    # check data_flows internal
    if 'data_flows_internal' in check_list and hasattr(state, 'data_flows'):  # TODO last element of condition has to be deleted again::
        for df_id, df in state.data_flows.iteritems():
            assert df_id in stored_state_elements['data_flows']
        # - check if the right models are there and only those
        model_id_store = []
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in stored_state_elements['data_flows']
            model_id_store.append(df_m.data_flow.data_flow_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['data_flows_meta'][df_m.data_flow.data_flow_id] == df_m.meta
        for df_id in stored_state_elements['data_flows']:
            assert df_id in model_id_store
    else:
        assert not hasattr(state, 'data_flows')

    def is_related_data_flow(parent, state_id, df):
        return df.from_state == state_id or df.to_state == state_id

    # check data_flows external
    if 'data_flows_external' in check_list:
        for df_id, df in state.parent.data_flows.iteritems():
            if is_related_data_flow(state.parent, state.state_id, df):
                assert df_id in stored_state_elements['data_flows_external']
            else:
                assert df_id in stored_state_elements['data_flows_external_not_related']

        for df_m in state_m.parent.data_flows:
            df_id = df_m.data_flow.data_flow_id
            df = df_m.data_flow
            if is_related_data_flow(state.parent, state.state_id, df):
                assert df_id in stored_state_m_elements['data_flows_external']
                # - check if meta data is still the same
                assert stored_state_m_elements['data_flows_external_meta'][df_id] == df_m.meta
            else:
                assert df_id in stored_state_m_elements['data_flows_external_not_related']
                # - check if meta data is still the same
                assert stored_state_m_elements['data_flows_external_not_related_meta'][df_id] == df_m.meta

    # else:
    #     assert state.parent is None  # root state now has a parent

    print "\n check state type"
    # check state type and check source script
    if isinstance(state, ExecutionState):
        print "\n\nEXECUTION_STATE\n\n"
    elif isinstance(state, HierarchyState):
        print "\n\nHIERARCHY_STATE\n\n"
    elif isinstance(state, BarrierConcurrencyState):
        print "\n\nBARRIER_STATE\n\n"
    elif isinstance(state, PreemptiveConcurrencyState):
        print "\n\nPREEMPTIVE_STATE\n\n"
    else:
        print "\n\nNO EXECUTABLE STATE TYPE", state
        assert state in [ExecutionState, HierarchyState, PreemptiveConcurrencyState, BarrierConcurrencyState]


def list_store_id_dict(store):
    id = 0
    list_store_id = {}
    for row in store:
        # Print values of all columns
        list_store_id[row[0]] = id
        id += 1
    return list_store_id

check_list_ES = ['ports', 'outcomes', 'transitions_external', 'data_flows_external']
check_list_HS = ['ports', 'outcomes', 'states', 'scoped_variables',
                  'transitions_internal', 'transitions_external',
                  'data_flows_internal', 'data_flows_external']
check_list_PCS = ['ports', 'outcomes', 'states', 'scoped_variables',
                  'transitions_internal', 'transitions_external',
                  'data_flows_internal', 'data_flows_external']
check_list_BCS = ['ports', 'outcomes', 'states', 'scoped_variables',
                  'transitions_internal', 'transitions_external',
                  'data_flows_internal', 'data_flows_external']

check_list_root_ES = ['ports', 'outcomes']
check_list_root_HS = ['ports', 'outcomes', 'states', 'scoped_variables',
                      'transitions_internal',
                      'data_flows_internal']
check_list_root_PCS = ['ports', 'outcomes', 'states', 'scoped_variables',
                       'transitions_internal',
                       'data_flows_internal']
check_list_root_BCS = ['ports', 'outcomes', 'states', 'scoped_variables',
                       'transitions_internal',
                       'data_flows_internal']


def trigger_state_type_change_tests(*args):
    print "Wait for the gui to initialize"
    time.sleep(1.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    sm_m = args[2]
    state_dict = args[3]
    with_gui = args[4]
    sleep_time = 2

    time.sleep(sleep_time)

    ####### General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'

    # HS -> ES
    # state_m = sm_m.get_state_model_by_path(state_dict['Nested'].get_path())
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    print "\n\n %s \n\n" % state_m.state.name
    sm_m.selection.set([state_m])
    time.sleep(sleep_time)
    # state_dict['Container'].change_state_type(state_m, ExecutionState)
    # state_dict['Container'].change_state_type(state_m, PreemptiveConcurrencyState)

    # do state_type_change with gui
    # - find state machine id

    my_sm_id = None
    for sm_id, state_machine in sm_manager_model.state_machine_manager.state_machines.iteritems():
        if state_machine is sm_m.state_machine:
            my_sm_id = sm_id
    assert my_sm_id is not None

    list_store_id_from_state_type_dict = {}
    state_editor_ctrl = None
    if with_gui:
        # - get states-editor controller
        state_identifier = str(my_sm_id) + '|' + state_dict[state_of_type_change].get_path()
        assert state_identifier in main_window_controller.get_controller('states_editor_ctrl').tabs
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[state_identifier]['controller']

        # - find right row in combo box
        store = state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].get_model()
        list_store_id_from_state_type_dict = list_store_id_dict(store)

        print "+++++++++++++++++++++++++++++++++++++" + sm_m.state_machine.root_state.state_id

    # HS -> BCS
        state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
        glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
        time.sleep(sleep_time)
    else:
        state_dict[state_of_type_change].change_state_type(state_m, BarrierConcurrencyState)

    print "+++++++++++++++++++++++++++++++++++++" + sm_m.state_machine.root_state.state_id

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_BCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # BCS -> HS
    if with_gui:
        state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
        glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
        time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # HS -> PCS
    state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # PCS -> ES
    state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_ES, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # TODO all test that are not root_state-test have to be performed with Preemptive and Barrier Concurrency States as parents too

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'

    # HS -> ES
    # state_m = sm_m.get_state_model_by_path(state_dict['Nested'].get_path())
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    print "\n\n %s \n\n" % state_m.state.name
    sm_m.selection.set([state_m])
    time.sleep(sleep_time)

    # do state_type_change with gui
    states_editor_controller = main_window_controller.get_controller('states_editor_ctrl')
    state_identifier = states_editor_controller.get_state_identifier(state_m)
        # str(my_sm_id) + '|' + state_dict[state_of_type_change].get_path()
    # states_editor_controller.tabs[state_identifier]
    # states_editor_controller.tabs[state_identifier]['controller']
    state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[state_identifier]['controller']
    print state_editor_ctrl.get_controller('properties_ctrl')
    print state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].get_model()

    # - find right row in combo box
    store = state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].get_model()
    list_store_id_from_state_type_dict = list_store_id_dict(store)

    # HS -> BCS
    state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_BCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # BCS -> HS
    state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    ###################################
    # Test Preemptive Concurrency State
    # RULES
    # - no start states

    # HS -> PCS
    state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # PCS -> ES
    state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
    glib.idle_add(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    time.sleep(sleep_time)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_ES, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    # simple type change of root_state

    # state_m = sm_m.root_state
    # sm_m.state_machine.change_root_state_type(state_m, ExecutionState)
    #
    # time.sleep(sleep_time)
    #
    # sm_m.history.undo()

    time.sleep(sleep_time)

    if with_gui:
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        glib.idle_add(menubar_ctrl.on_stop_activate, None)
        menubar_ctrl.model.get_selected_state_machine_model().state_machine.file_system_path = '/tmp/dfc_test_state_type_change'
        glib.idle_add(menubar_ctrl.on_save_activate, None)
        glib.idle_add(menubar_ctrl.on_quit_activate, None)


def test_state_type_change_with_gui():
    state_type_change_test(with_gui=True)


def _test_state_type_change_without_gui():
    state_type_change_test(with_gui=False)


def state_type_change_test(with_gui=False):

    variables_for_pytest.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(rafcon.__path__[0] + "/mvc")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    global_config.load()  # load the default config
    global_gui_config.load()  # load the default config
    logging_view = LoggingView()
    setup_logger(logging_view)

    logger, state, gvm_model, sm_m, state_dict = create_models()

    rafcon.statemachine.singleton.library_manager.initialize()

    if variables_for_pytest.sm_manager_model is None:
            variables_for_pytest.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = None
    if with_gui:
        main_window_view = MainWindowView(logging_view)

        # load the meta data for the state machine
        variables_for_pytest.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

        main_window_controller = MainWindowController(variables_for_pytest.sm_manager_model, main_window_view,
                                                      editor_type='LogicDataGrouped')
    else:
        # load the meta data for the state machine
        variables_for_pytest.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    thread = threading.Thread(target=trigger_state_type_change_tests,
                              args=[variables_for_pytest.sm_manager_model, main_window_controller,
                                    sm_m, state_dict, with_gui])
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
        os.chdir(rafcon.__path__[0] + "/../test")
        variables_for_pytest.test_multithrading_lock.release()
    else:
        os.chdir(rafcon.__path__[0] + "/../test")
        thread.join()


if __name__ == '__main__':
    # _test_state_type_change_without_gui()

    test_state_type_change_with_gui()