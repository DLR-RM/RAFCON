import time

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
import pytest

store_elements_ignores = []
check_elements_ignores = []

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

    tmp_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4,
                  'Nested2': state5}
    sm = StateMachine(ctr_state)

    return tmp_dict, sm


def store_state_elements(state, state_m, return_list=None):
    from rafcon.core.states.state import State
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID

    """Stores all ids of elements in or outside of the actual state"""
    print "STORE state elements of %s, %s" % (state.name, state_m.state.name)
    global store_elements_ignores
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
    if isinstance(state, ContainerState):
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
    if isinstance(state, ContainerState):
        state_elements['states'] = []
        for s_id, s in state.states.iteritems():
            state_elements['states'].append(s_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['states_meta'] = {}
        print state_m.states.keys()
        for s_m_id, s_m in state_m.states.iteritems():
            # if not hasattr(s_m, "state"):
            #     print s_m
            assert s_m_id == s_m.state.state_id
            assert s_m_id in state_elements['states']
            assert s_m.state.state_id in state_elements['states']
            model_id_store.append(s_m.state.state_id)
            # - store model meta data
            state_m_elements['states_meta'][s_m.state.state_id] = s_m.meta
        # -check if all states have a model otherwise check after change has to fail
        for s_id, s in state.states.iteritems():
            # print s_id, model_id_store, s_id == UNIQUE_DECIDER_STATE_ID, s_id in model_id_store,
            # "missing_decider_state_models" in store_elements_ignores
            if not s_id == UNIQUE_DECIDER_STATE_ID or \
                    s_id == UNIQUE_DECIDER_STATE_ID and (s_id not in model_id_store and
                                                         "missing_decider_state_models" not in store_elements_ignores):
                assert s_id in model_id_store
            else:
                print "skip unique_state_id for model check"

    # collect data_flows
    if isinstance(state, ContainerState):
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
    if isinstance(state, ContainerState):
        state_elements['transitions'] = []
        for t_id, t in state.transitions.iteritems():
            state_elements['transitions'].append(t_id)
        # - check if the right models are there and only those
        model_id_store = []
        state_m_elements['transitions_meta'] = {}
        # print [t_m.transition.transition_id for t_m in state_m.transitions]
        # print state_elements['transitions']
        for t_m in state_m.transitions:
            if UNIQUE_DECIDER_STATE_ID not in [t_m.transition.to_state, t_m.transition.from_state]:
                assert t_m.transition.transition_id in state_elements['transitions']
            model_id_store.append(t_m.transition.transition_id)
            # - store model meta data
            state_m_elements['transitions_meta'][t_m.transition.transition_id] = t_m.meta
        for t_id, t in state.transitions.iteritems():
            # TODO test needs to be improved to cover BarrierState, too
            if UNIQUE_DECIDER_STATE_ID not in [t.to_state, t.from_state]:
                assert t_id in model_id_store

    def is_related_transition(parent, state_id, t):
        return t.from_state == state_id or t.to_state == state_id

    def is_related_data_flow(parent, state_id, df):
        return df.from_state == state_id or df.to_state == state_id

    # LOOKOUT: root states have their state machine as parent
    if isinstance(state, State) and state.parent is not None and isinstance(state.parent, State):
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

    if return_list is not None:
        return_list.append(state_elements)
        return_list.append(state_m_elements)
    else:
        return state_elements, state_m_elements

def check_state_elements(check_list, state, state_m, stored_state_elements, stored_state_m_elements):
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
    from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID

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
    if not type(state_m.state) is type(state):
        print "given model is not linked with given state"
        print "State-Model State-Type: ", state_m.state
        print "State-Type: ", state
    # TODO last element of condition has to be deleted again
    if 'states' in check_list and isinstance(state, ContainerState):
        for s_id, s in state.states.iteritems():
            if not s_id == UNIQUE_DECIDER_STATE_ID:
                assert s_id in stored_state_elements['states']
        # - check if the right models are there and only those
        model_id_store = []
        for s_m_id, s_m in state_m.states.iteritems():
            # if not hasattr(s_m, "state"):
            #     print s_m
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
        # if isinstance(state, ContainerState):
        #     print state, state.states
        assert not isinstance(state, ContainerState)
    # exit(0)

    # check scoped_variables
    # TODO last element of condition has to be deleted again:
    if 'scoped_variables' in check_list and isinstance(state, ContainerState):
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
        assert not isinstance(state, ContainerState)

    print "ignore internal_transitions in check: ", "internal_transitions" in check_elements_ignores
    if "internal_transitions" not in check_elements_ignores:
        # check transitions internal
        if 'transitions_internal' in check_list:
            for t_id, t in state.transitions.iteritems():
                print state.name, t_id, stored_state_elements['transitions'], state.transitions.keys()
                assert t_id in stored_state_elements['transitions']
            # - check if the right models are there and only those
            model_id_store = []
            for t_m in state_m.transitions:
                assert t_m.transition.transition_id in stored_state_elements['transitions']
                model_id_store.append(t_m.transition.transition_id)
                # - check if meta data is still the same
                assert stored_state_m_elements['transitions_meta'][t_m.transition.transition_id] == t_m.meta
            for t_id in stored_state_elements['transitions']:
                assert t_id in model_id_store
        else:
            assert not isinstance(state, ContainerState)

        def is_related_transition(parent, state_id, t):
            return t.from_state == state_id or t.to_state == state_id

        # check transitions external
        if 'transitions_external' in check_list:
            for t_id, t in state.parent.transitions.iteritems():
                if is_related_transition(state.parent, state.state_id, t):
                    assert t_id in stored_state_elements['transitions_external']
                else:
                    assert stored_state_elements['transitions_external_not_related']

            for t_m in state_m.parent.transitions:
                t_id = t_m.transition.transition_id
                t = t_m.transition
                if is_related_transition(state.parent, state.state_id, t):
                    assert t_id in stored_state_m_elements['transitions_external']
                    # - check if meta data is still the same
                    assert stored_state_m_elements['transitions_external_meta'][t_id] == t_m.meta
                else:
                    assert t_id in stored_state_m_elements['transitions_external_not_related']
                    # - check if meta data is still the same
                    assert stored_state_m_elements['transitions_external_not_related_meta'][t_id] == t_m.meta
        else:
            assert state.parent is None

    # check data_flows internal
    # TODO last element of condition has to be deleted again
    if 'data_flows_internal' in check_list and isinstance(state, ContainerState):
        # - all data_flows in the actual state should be in the stored_state_elements, too
        for df_id, df in state.data_flows.iteritems():
            assert df_id in stored_state_elements['data_flows']
        # - check if the right models are there (1) and only those (2)
        model_id_store = []
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in stored_state_elements['data_flows']
            model_id_store.append(df_m.data_flow.data_flow_id)
            # - check if meta data is still the same
            assert stored_state_m_elements['data_flows_meta'][df_m.data_flow.data_flow_id] == df_m.meta
        print stored_state_elements['data_flows']
        print model_id_store
        for df_id in stored_state_elements['data_flows']:
            assert df_id in model_id_store
    else:
        assert not isinstance(state, ContainerState)

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
                # assert stored_state_m_elements['data_flows_external_meta'][df_id] == df_m.meta
            else:
                assert df_id in stored_state_m_elements['data_flows_external_not_related']
                # - check if meta data is still the same
                # assert stored_state_m_elements['data_flows_external_not_related_meta'][df_id] == df_m.meta

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


def wait_for_states_editor(main_window_controller, tab_key, max_time=5.0):
    assert tab_key in main_window_controller.get_controller('states_editor_ctrl').tabs
    time_waited = 0.0
    time_start = time.time()
    testing_utils.wait_for_gui()
    time_waited += time.time() - time_start
    state_editor_ctrl = None
    while state_editor_ctrl is None:
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[tab_key]['controller']
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


check_list_ES = ['ports', 'outcomes', 'transitions_external', 'data_flows_external']
check_list_HS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'transitions_external',
                 'data_flows_internal', 'data_flows_external']
check_list_PCS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'transitions_external',
                  'data_flows_internal', 'data_flows_external']
check_list_BCS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'transitions_external',
                  'data_flows_internal', 'data_flows_external']

check_list_root_ES = ['ports', 'outcomes']
check_list_root_HS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'data_flows_internal']
check_list_root_PCS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'data_flows_internal']
check_list_root_BCS = ['ports', 'outcomes', 'states', 'scoped_variables', 'transitions_internal', 'data_flows_internal']


def debug_logger_print(s, logger):
    if logger:
        logger.debug(s)
    else:
        print s


def get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger=None):
    states_editor_controller = main_window_controller.get_controller('states_editor_ctrl')
    # - do state selection to generate state editor widget
    sm_m.selection.set(state_m)
    # - get states-editor controller
    state_identifier = states_editor_controller.get_state_identifier(state_m)
    [state_editor_ctrl, time_waited] = wait_for_states_editor(main_window_controller, state_identifier, sleep_time_max)
    debug_logger_print("wait for state's state editor %s" % time_waited, logger)
    assert state_editor_ctrl.model == state_m
    # - find right row in combo box
    store = state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].get_model()
    list_store_id_from_state_type_dict = list_store_id_dict(store)
    return state_editor_ctrl, list_store_id_from_state_type_dict


def change_state_type(input_and_return_list, new_state_type, state_of_type_change, checklist,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements):
    from rafcon.gui.singleton import main_window_controller
    state_m = input_and_return_list.pop()
    sleep_time_max = 5.0
    # - get state-editor controller and find right row in combo box
    [state_editor_ctrl, list_store_id_from_state_type_dict] = \
        get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger)
    # - do state type change
    state_type_row_id = list_store_id_from_state_type_dict[new_state_type]
    state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active(state_type_row_id)
    # - do checks
    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(checklist, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    input_and_return_list.append(new_state_m)


@log.log_exceptions(None, gtk_quit=True)
def trigger_state_type_change_tests(with_gui):
    """Only works with gui at the moment.

    :param args:
    """
    import rafcon.core.singleton
    import rafcon.gui.singleton
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model

    state_dict, sm = create_state_machine()
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, sm)
    call_gui_callback(testing_utils.wait_for_gui)

    first_sm_id = sm.state_machine_id
    sm_m = sm_manager_model.state_machines[first_sm_id]
    call_gui_callback(sm_manager_model.__setattr__, 'selected_state_machine_id', first_sm_id)
    # General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    check_elements_ignores.append("internal_transitions")
    # first storage
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())

    input_and_return_list = list()
    call_gui_callback(store_state_elements, state_dict[state_of_type_change], state_m, input_and_return_list)
    call_gui_callback(testing_utils.wait_for_gui)
    stored_state_elements = input_and_return_list[0]
    stored_state_m_elements = input_and_return_list[1]

    print "\n\n %s \n\n" % state_m.state.name

    # HS -> BCS
    input_and_return_list = [state_m]
    call_gui_callback(sm_m.selection.set, input_and_return_list)
    call_gui_callback(change_state_type, input_and_return_list, 'BARRIER_CONCURRENCY', 'State3', check_list_BCS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # BCS -> HS
    call_gui_callback(change_state_type, input_and_return_list, 'HIERARCHY', 'State3', check_list_HS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # HS -> PCS
    call_gui_callback(change_state_type, input_and_return_list, 'PREEMPTION_CONCURRENCY', 'State3', check_list_PCS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # PCS -> ES
    call_gui_callback(change_state_type, input_and_return_list, 'EXECUTION', 'State3', check_list_ES,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # TODO all test that are not root_state-test have to be performed with Preemptive and Barrier Concurrency States as parents too

    # General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    # get first storage
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    input_and_return_list = []
    call_gui_callback(store_state_elements, state_dict[state_of_type_change], state_m, input_and_return_list)
    call_gui_callback(testing_utils.wait_for_gui)
    stored_state_elements = input_and_return_list[0]
    stored_state_m_elements = input_and_return_list[1]
    print "\n\n %s \n\n" % state_m.state.name


    # HS -> BCS
    print "Test: change root state type: HS -> BCS"
    input_and_return_list = [state_m]
    call_gui_callback(sm_m.selection.set, input_and_return_list)
    call_gui_callback(change_state_type, input_and_return_list, 'BARRIER_CONCURRENCY', 'Container', check_list_root_BCS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # BCS -> HS
    print "Test: change root state type: BCS -> HS"
    call_gui_callback(change_state_type, input_and_return_list, 'HIERARCHY', 'Container', check_list_root_HS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # HS -> PCS
    print "Test: change root state type: HS -> PCS"
    call_gui_callback(change_state_type, input_and_return_list, 'PREEMPTION_CONCURRENCY', 'Container', check_list_root_PCS,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # PCS -> ES
    print "Test: change root state type: PCS -> ES"
    call_gui_callback(change_state_type, input_and_return_list, 'EXECUTION', 'Container', check_list_root_ES,
                      sm_m, state_dict, stored_state_elements, stored_state_m_elements)

    # simple type change of root_state -> still could be extended
    check_elements_ignores.remove("internal_transitions")


def test_state_type_change_test(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        trigger_state_type_change_tests(with_gui=True)
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    # test_state_type_change_test(None)
    pytest.main(['-s', __file__])
