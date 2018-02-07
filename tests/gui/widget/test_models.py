from gtkmvc.observer import Observer

import pytest
import testing_utils

with_print = False


class NotificationLogObserver(Observer):
    """ This observer is a abstract class to count and store notification
    """

    def __init__(self, model, with_print=False):

        self.log = {"before": {}, "after": {}}
        self.reset()
        self.observed_model = model
        self.with_print = with_print
        self.no_failure = True

        Observer.__init__(self, model)

    def reset(self):
        """ Initiate and reset the log dictionary """

    def get_number_of_notifications(self):
        nr = 0
        for key, l in self.log['before'].iteritems():
            nr += len(l)
        for key, l in self.log['after'].iteritems():
            nr += len(l)
        return nr


class StateNotificationLogObserver(NotificationLogObserver):
    """ This observer counts and stores notification of StateModel-Class
    """

    def __init__(self, model, with_print=False):
        NotificationLogObserver.__init__(self, model, with_print)

    def reset(self):
        self.log = {"before": {'states': [], 'state': [],
                               'outcomes': [], 'input_data_ports': [], 'output_data_ports': [], 'scoped_variables': [],
                               'transitions': [], 'data_flows': [], 'is_start': [],
                               'meta_signal': [], 'state_type_changed_signal': []},
                    "after": {'states': [], 'state': [],
                              'outcomes': [], 'input_data_ports': [], 'output_data_ports': [], 'scoped_variables': [],
                              'transitions': [], 'data_flows': [], 'is_start': [],
                              'meta_signal': [], 'state_type_changed_signal': []}}
        self.no_failure = True

    @Observer.observe('states', before=True)
    @Observer.observe("state", before=True)
    @Observer.observe("outcomes", before=True)
    @Observer.observe("input_data_ports", before=True)
    @Observer.observe("output_data_ports", before=True)
    @Observer.observe("scoped_variables", before=True)
    @Observer.observe("transitions", before=True)
    @Observer.observe("data_flows", before=True)
    @Observer.observe("is_start", before=True)
    def notification_before(self, model, prop_name, info):
        # print "parent call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #       (prop_name, info.instance, info.method_name, info.result)
        #if info.method_name in self.method_list:
        if prop_name in self.log['before']:
            self.log['before'][prop_name].append({'model': model, 'prop_name': prop_name, 'info': info})
            # if self.with_print:
            #     print "++++++++ log BEFORE instance '%s' and property '%s' in state %s" % \
            #           (info.instance, prop_name, self.observed_model.state.name)
            #     print "observer: ", self
        else:
            if self.with_print:
                print "!!!! NOT a prop_name '%s' to be observed in BEFORE %s %s" % (prop_name, model, info)
            self.no_failure = False

        self.parent_state_of_notification_source(model, prop_name, info, before_after='before')

    @Observer.observe('states', after=True)
    @Observer.observe("state", after=True)
    @Observer.observe("outcomes", after=True)
    @Observer.observe("input_data_ports", after=True)
    @Observer.observe("output_data_ports", after=True)
    @Observer.observe("scoped_variables", after=True)
    @Observer.observe("transitions", after=True)
    @Observer.observe("data_flows", after=True)
    @Observer.observe("is_start", after=True)
    @Observer.observe("meta_signal", signal=True)
    @Observer.observe("state_type_changed_signal", signal=True)
    def notification_after(self, model, prop_name, info):
        if prop_name in self.log['after']:
            self.log['after'][prop_name].append({'model': model, 'prop_name': prop_name, 'info': info})
            # if self.with_print:
            #     print "++++++++ log AFTER instance '%s' and property '%s' in state %s" % \
            #           (info.instance, prop_name, self.observed_model.state.name)
            #     print "observer: ", self
        else:
            if self.with_print:
                print "!!!! NOT a prop_name '%s' to be observed in AFTER %s %s" % (prop_name, model, info)
            self.no_failure = False

        self.parent_state_of_notification_source(model, prop_name, info, before_after='after')

    def parent_state_of_notification_source(self, model, prop_name, info, before_after):
        if self.with_print:
            print "----- xxxxxxx %s \n%s\n%s\n%s\n" % (before_after, model, prop_name, info)

        def set_dict(info, d):
            d['model'].append(info['model'])
            d['prop_name'].append(info['prop_name'])
            d['instance'].append(info['instance'])
            d['method_name'].append(info['method_name'])
            if self.with_print:
                print "set"

        def find_parent(info, elem):
            elem['info'].append(info)
            if 'kwargs' in info and info['kwargs']:
                if self.with_print:
                    print 'kwargs'
                elem['level'].append('kwargs')
                set_dict(info, elem)
                if 'method_name' in info['kwargs']:
                    find_parent(info['kwargs'], elem)
            elif 'info' in info and info['info']:
                if self.with_print:
                    print 'info'
                elem['level'].append('info')
                set_dict(info, elem)
                find_parent(info['info'], elem)
            elif 'info' in info or 'kwargs' in info:
                set_dict(info, elem)
            else:
                print info
                from rafcon.gui.utils.notification_overview import NotificationOverview
                print 'NotificationLogger ---> assert !!! Type of notification not known'#\n{0}'.format(NotificationOverview(info))
                assert True
            return elem

        overview = find_parent(info, {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'level': [],
                                      'info': []})
        # # info_print = ''
        # # for elem in overview['info']:
        # #     info_print += "\n" + str(elem)
        # # print info_print
        # if self.with_print:
        #     print overview['model']
        #     print overview['prop_name']
        #     print overview['instance']
        #     print overview['method_name']
        #     print overview['level']
        #     print overview['prop_name'][-1]
        # if overview['prop_name'][-1] == 'state':
        #     print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
        #     assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
        # else:
        #     if overview['model'][-1].parent is None:  # is root_state
        #         overview['model'][-1].state.get_path()
        #         print "Path_root: ", overview['model'][-1].state.get_path()
        #     else:
        #         overview['model'][-1].parent.state.get_path()
        #         print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
        #             overview['model'][-1].parent.state.get_path()
        #         assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.state.get_path().split('/')[0]
        return overview


def create_models(*args, **kargs):
    import rafcon.core.singleton
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id="NESTED")
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id="NESTED2")
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3')
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

    ctr_state = HierarchyState(name="Container")
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
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4, 'Nested2': state5}
    sm = StateMachine(ctr_state)

    import rafcon.gui.models.state_machine
    # for sm_in in rafcon.core.singleton.state_machine_manager.state_machines.values():
    #     rafcon.core.singleton.state_machine_manager.remove_state_machine(sm_in.state_machine_id)
    sm_m = rafcon.gui.models.state_machine.StateMachineModel(sm)

    # sm_m.history.fake = True

    return ctr_state, sm_m, state_dict


def store_state_elements(state, state_m):
    """Stores all ids of elements in or outside of the actual state"""
    from rafcon.core.states.state import State
    from rafcon.core.states.container_state import ContainerState

    state_elements = {}
    state_m_elements = {}
    # collect input_data_ports
    state_elements['input_data_ports'] = []
    for p_id, p in state.input_data_ports.iteritems():
        state_elements['input_data_ports'].append(p_id)
    # - check if the right models are there and only those
    model_id_store = []
    # state_m_elements['input_data_ports_meta'] = {}
    for p_m in state_m.input_data_ports:
        assert p_m.data_port.data_port_id in state_elements['input_data_ports']
        model_id_store.append(p_m.data_port.data_port_id)
        # - store model meta data
        # state_m_elements['input_data_ports_meta'][p_m.data_port.data_port_id] = p_m.meta
    for p_id, p in state.input_data_ports.iteritems():
        assert p_id in model_id_store
    # print state_elements['input_data_ports'], state.input_data_ports

    # collect output_data_ports
    state_elements['output_data_ports'] = []
    for p_id, p in state.output_data_ports.iteritems():
        state_elements['output_data_ports'].append(p_id)
    # - check if the right models are there and only those
    model_id_store = []
    # state_m_elements['output_data_ports_meta'] = {}
    for p_m in state_m.output_data_ports:
        assert p_m.data_port.data_port_id in state_elements['output_data_ports']
        model_id_store.append(p_m.data_port.data_port_id)
        # - store model meta data
        # state_m_elements['output_data_ports_meta'][p_m.data_port.data_port_id] = p_m.meta
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
    # state_m_elements['outcomes_meta'] = {}
    for oc_m in state_m.outcomes:
        assert oc_m.outcome.outcome_id in state_elements['outcomes']
        model_id_store.append(oc_m.outcome.outcome_id)
        # - store model meta data
        # state_m_elements['outcomes_meta'][oc_m.outcome.outcome_id] = oc_m.meta
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
        # state_m_elements['scoped_variables_meta'] = {}
        for sv_m in state_m.scoped_variables:
            assert sv_m.scoped_variable.data_port_id in state_elements['scoped_variables']
            model_id_store.append(sv_m.scoped_variable.data_port_id)
            # - store model meta data
            # state_m_elements['scoped_variables_meta'][sv_m.scoped_variable.data_port_id] = sv_m.meta
        for sv_id, sv in state.scoped_variables.iteritems():
            assert sv_id in model_id_store

    # collect states
    if isinstance(state, ContainerState):
        state_elements['states'] = []
        for s_id, s in state.states.iteritems():
            state_elements['states'].append(s_id)
        # - check if the right models are there and only those
        model_id_store = []
        # state_m_elements['states_meta'] = {}
        for s_m_id, s_m in state_m.states.iteritems():
            # if not hasattr(s_m, "state"):
            #     print s_m
            assert s_m_id == s_m.state.state_id
            assert s_m_id in state_elements['states']
            assert s_m.state.state_id in state_elements['states']
            model_id_store.append(s_m.state.state_id)
            # - store model meta data
            # state_m_elements['states_meta'][s_m.state.state_id] = s_m.meta
        for s_id, s in state.states.iteritems():
            assert s_id in model_id_store

    # collect data_flows
    if isinstance(state, ContainerState):
        state_elements['data_flows'] = []
        for df_id, df in state.data_flows.iteritems():
            state_elements['data_flows'].append(df_id)
        # - check if the right models are there and only those
        model_id_store = []
        # state_m_elements['data_flows_meta'] = {}
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in state_elements['data_flows']
            model_id_store.append(df_m.data_flow.data_flow_id)
            # - store model meta data
            # state_m_elements['data_flows_meta'][df_m.data_flow.data_flow_id] = df_m.meta
        for df_id, df in state.data_flows.iteritems():
            assert df_id in model_id_store

    # collect transitions
    if isinstance(state, ContainerState):
        state_elements['transitions'] = []
        for t_id, t in state.transitions.iteritems():
            state_elements['transitions'].append(t_id)
        # - check if the right models are there and only those
        model_id_store = []
        # state_m_elements['transitions_meta'] = {}
        for t_m in state_m.transitions:
            assert t_m.transition.transition_id in state_elements['transitions']
            model_id_store.append(t_m.transition.transition_id)
            # - store model meta data
            # state_m_elements['transitions_meta'][t_m.transition.transition_id] = t_m.meta
        for t_id, t in state.transitions.iteritems():
            assert t_id in model_id_store

    def is_related_transition(parent, state_id, t):
        return t.from_state == state_id or t.to_state == state_id

    def is_related_data_flow(parent, state_id, df):
        return df.from_state == state_id or df.to_state == state_id

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
        # state_m_elements['transitions_external_meta'] = {}
        # state_m_elements['transitions_external_not_related_meta'] = {}
        for t_m in state_m.parent.transitions:
            t_id = t_m.transition.transition_id
            t = t_m.transition
            if is_related_transition(state.parent, state.state_id, t):
                state_m_elements['transitions_external'].append(t_id)
                assert t_m.transition.transition_id in state_elements['transitions_external']
                # state_m_elements['transitions_external_meta'][t_id] = t_m.meta
            else:
                state_m_elements['transitions_external_not_related'].append(t_id)
                # state_m_elements['transitions_external_not_related_meta'][t_id] = t_m.meta

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
        # state_m_elements['data_flows_external_meta'] = {}
        # state_m_elements['data_flows_external_not_related_meta'] = {}
        for df_m in state_m.parent.data_flows:
            df_id = df_m.data_flow.data_flow_id
            df = df_m.data_flow
            if is_related_data_flow(state.parent, state.state_id, df):
                state_m_elements['data_flows_external'].append(df_id)
                assert df_m.data_flow.data_flow_id in state_elements['data_flows_external']
                # state_m_elements['data_flows_external_meta'][df_id] = t_m.meta
            else:
                state_m_elements['data_flows_external_not_related'].append(df_id)
                # state_m_elements['data_flows_external_not_related_meta'][df_id] = t_m.meta
    else:
        print "STATE is a root_state"

    return state_elements, state_m_elements


def check_state_elements(check_list, state, state_m, stored_state_elements, stored_state_m_elements):
    from rafcon.core.states.container_state import ContainerState
    
    # check ports
    if 'ports' in check_list:
        # collect input_data_ports
        for p_id, p in state.input_data_ports.iteritems():
            # print p_id, stored_state_elements['input_data_ports']
            assert p_id in stored_state_elements['input_data_ports']
        # - check if the right models are there and only those
        model_id_store = []
        for p_m in state_m.input_data_ports:
            assert p_m.data_port.data_port_id in stored_state_elements['input_data_ports']
            model_id_store.append(p_m.data_port.data_port_id)
            # - check if meta data is still the same
            # assert stored_state_m_elements['input_data_ports_meta'][p_m.data_port.data_port_id] == p_m.meta
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
            # assert stored_state_m_elements['output_data_ports_meta'][p_m.data_port.data_port_id] == p_m.meta
        for p_id in stored_state_elements['output_data_ports']:
            assert p_id in model_id_store

    # check outcomes
    if 'outcomes' in check_list:
        # collect outcomes
        for oc_id, oc, in state.outcomes.iteritems():
            print oc_id, stored_state_elements['outcomes']
            assert oc_id in stored_state_elements['outcomes']
        # - check if the right models are there and only those
        model_id_store = []
        for oc_m in state_m.outcomes:
            assert oc_m.outcome.outcome_id in stored_state_elements['outcomes']
            model_id_store.append(oc_m.outcome.outcome_id)
            # - check if meta data is still the same
            # assert stored_state_m_elements['outcomes_meta'][oc_m.outcome.outcome_id] == oc_m.meta
        for oc_id in stored_state_elements['outcomes']:
            assert oc_id in model_id_store

    # check states
    if 'states' in check_list:
        for s_id, s in state.states.iteritems():
            assert s_id in stored_state_elements['states']
        # - check if the right models are there and only those
        model_id_store = []
        for s_m_id, s_m in state_m.states.iteritems():
            # if not hasattr(s_m, "state"):
            #     print s_m
            assert s_m_id == s_m.state.state_id
            assert s_m_id in stored_state_elements['states']
            assert s_m.state.state_id in stored_state_elements['states']
            model_id_store.append(s_m.state.state_id)
            # - check if meta data is still the same
            # print stored_state_m_elements['states_meta'][s_m.state.state_id], s_m.meta
            # assert stored_state_m_elements['states_meta'][s_m.state.state_id] == s_m.meta
        for s_id in stored_state_elements['states']:
            assert s_id in model_id_store
    else:
        assert not isinstance(state, ContainerState)
    # exit(0)

    # check scoped_variables
    if 'scoped_variables' in check_list:
        for sv_id, sv, in state.scoped_variables.iteritems():
            assert sv_id in stored_state_elements['scoped_variables']
        # - check if the right models are there and only those
        model_id_store = []
        for sv_m in state_m.scoped_variables:
            assert sv_m.scoped_variable.data_port_id in stored_state_elements['scoped_variables']
            model_id_store.append(sv_m.scoped_variable.data_port_id)
            # - check if meta data is still the same
            # assert stored_state_m_elements['scoped_variables_meta'][sv_m.scoped_variable.data_port_id] == sv_m.meta
        for sv_id in stored_state_elements['scoped_variables']:
            assert sv_id in model_id_store
    else:
        assert not isinstance(state, ContainerState)

    # check transitions internal
    if 'transitions_internal' in check_list:
        for t_id, t in state.transitions.iteritems():
            assert t_id in stored_state_elements['transitions']
        # - check if the right models are there and only those
        model_id_store = []
        for t_m in state_m.transitions:
            assert t_m.transition.transition_id in stored_state_elements['transitions']
            model_id_store.append(t_m.transition.transition_id)
            # - check if meta data is still the same
            # assert stored_state_m_elements['transitions_meta'][t_m.transition.transition_id] == t_m.meta
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
                # assert stored_state_m_elements['transitions_external_meta'][t_id] == t_m.meta
            else:
                assert t_id in stored_state_m_elements['transitions_external_not_related']
                # - check if meta data is still the same
                # assert stored_state_m_elements['transitions_external_not_related_meta'][t_id] == t_m.meta
    else:
        assert state.parent is None

    # check data_flows internal
    if 'data_flows_internal' in check_list:
        for df_id, df in state.data_flows.iteritems():
            assert df_id in stored_state_elements['data_flows']
        # - check if the right models are there and only those
        model_id_store = []
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in stored_state_elements['data_flows']
            model_id_store.append(df_m.data_flow.data_flow_id)
            # - check if meta data is still the same
            # assert stored_state_m_elements['data_flows_meta'][df_m.data_flow.data_flow_id] == df_m.meta
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
                assert stored_state_m_elements['data_flows_external_meta'][df_id] == df_m.meta
            else:
                assert df_id in stored_state_m_elements['data_flows_external_not_related']
                # - check if meta data is still the same
                assert stored_state_m_elements['data_flows_external_not_related_meta'][df_id] == df_m.meta

    else:
        assert state.parent is None


def check_state_for_all_models(state, state_m):
    from rafcon.core.states.state import State
    from rafcon.core.states.container_state import ContainerState
    
    # check ports
    # - collect input_data_ports
    core_id_store = []
    for p_id, p in state.input_data_ports.iteritems():
        core_id_store.append(p_id)

    # - check if the right models are there and only those
    for p_m in state_m.input_data_ports:
        assert p_m.data_port.data_port_id in core_id_store

    # - collect output_data_ports
    core_id_store = []
    for p_id, p in state.output_data_ports.iteritems():
        core_id_store.append(p_id)
    # - check if the right models are there and only those
    for p_m in state_m.output_data_ports:
        assert p_m.data_port.data_port_id in core_id_store

    # check outcomes
    # - collect outcomes
    core_id_store = []
    for oc_id, oc, in state.outcomes.iteritems():
        core_id_store.append(oc_id)
    # - check if the right models are there and only those
    for oc_m in state_m.outcomes:
        assert oc_m.outcome.outcome_id in core_id_store

    # check states
    if isinstance(state, ContainerState):
        core_id_store = []
        for s_id, s in state.states.iteritems():
            core_id_store.append(s_id)
        # - check if the right models are there and only those
        for s_m_id, s_m in state_m.states.iteritems():
            # if not hasattr(s_m, "state"):
            #     print s_m
            assert s_m_id == s_m.state.state_id
            assert s_m_id in core_id_store
            assert s_m.state.state_id in core_id_store

    # check scoped_variables
    if isinstance(state, ContainerState):
        core_id_store = []
        for sv_id, sv, in state.scoped_variables.iteritems():
            core_id_store.append(sv_id)
        # - check if the right models are there and only those
        for sv_m in state_m.scoped_variables:
            assert sv_m.scoped_variable.data_port_id in core_id_store

    if isinstance(state, ContainerState):
        # check transitions internal
        core_id_store = []
        for t_id, t in state.transitions.iteritems():
            core_id_store.append(t_id)
        # - check if the right models are there and only those
        for t_m in state_m.transitions:
            assert t_m.transition.transition_id in core_id_store

        def is_related_transition(parent, state_id, t):
            return t.from_state == state_id or t.to_state == state_id

        # check transitions external
        if isinstance(state.parent, State):
            core_id_store = []
            for t_id, t in state.parent.transitions.iteritems():
                if is_related_transition(state.parent, state.state_id, t):
                    core_id_store.append(t_id)
                # else:
                #     assert stored_state_elements['transitions_external_not_related']

            for t_m in state_m.parent.transitions:
                t_id = t_m.transition.transition_id
                t = t_m.transition
                if is_related_transition(state.parent, state.state_id, t):
                    assert t_id in core_id_store
                # else:
                #     assert t_id in stored_state_m_elements['transitions_external_not_related']
        else:
            pass  # paren should be state_machine

    if isinstance(state, ContainerState):
        # check data_flows internal
        core_id_store = []
        for df_id, df in state.data_flows.iteritems():
            core_id_store.append(df_id)
        # - check if the right models are there and only those
        for df_m in state_m.data_flows:
            assert df_m.data_flow.data_flow_id in core_id_store

        def is_related_data_flow(parent, state_id, df):
            return df.from_state == state_id or df.to_state == state_id

        # check data_flows external
        if isinstance(state.parent, State):
            core_id_store = []
            for df_id, df in state.parent.data_flows.iteritems():
                if is_related_data_flow(state.parent, state.state_id, df):
                    core_id_store.append(df_id)
                # else:
                #     assert df_id in stored_state_elements['data_flows_external_not_related']

            for df_m in state_m.parent.data_flows:
                df_id = df_m.data_flow.data_flow_id
                df = df_m.data_flow
                if is_related_data_flow(state.parent, state.state_id, df):
                    assert df_id in core_id_store
                # else:
                #     assert df_id in stored_state_m_elements['data_flows_external_not_related']
        else:
            pass  # parent should be a state machine


def test_add_remove_models(caplog):
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
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config
    from rafcon.core.script import Script
    from rafcon.core.states.state import State
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.storage import storage

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()

    def store_state_machine(sm_model, path):
        storage.save_state_machine_to_path(sm_model.state_machine, path, delete_old_state_machine=True)
        sm_model.root_state.store_meta_data()


    state1 = HierarchyState('state1')
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    output_state1 = state1.add_output_data_port("output", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")

    state2 = ExecutionState('state2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    input_number_state2 = state2.add_input_data_port("number", "int", 5)
    output_res_state2 = state2.add_output_data_port("res", "int")

    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    state_dict['state1'] = state1
    state_dict['state2'] = state2

    StateNotificationLogObserver(sm_model.root_state, with_print=False)

    def print_all_states_with_path_and_name(state):
        print state.get_path(), state.name, type(state)
        if isinstance(state.parent, State):
            print "parent is: ", state.parent.state_id, state.parent.name

        from rafcon.core.states.container_state import ContainerState
        if isinstance(state, ContainerState):
            script = Script(parent=None)
        else:
            script = state.script
        state_dict = {'states': {}, 'data_flows': {}, 'transitions': {},
                      'input_data_ports': {},
                      'output_data_ports': {},
                      'scoped_variables': {},
                      'outcomes': {},
                      'path': state.get_path(),
                      'script': script, 'name': state.name, 'description': state.description}

        if isinstance(state, ContainerState):
            for s_id, child_state in state.states.iteritems():
                state_dict['states'][s_id] = print_all_states_with_path_and_name(child_state)
            for dp_id, dp in state.input_data_ports.iteritems():
                state_dict['input_data_ports'][dp_id] = dp
            for dp_id, dp in state.output_data_ports.iteritems():
                state_dict['output_data_ports'][dp_id] = dp
            for dp_id, dp in state.scoped_variables.iteritems():
                state_dict['scoped_variables'][dp_id] = dp
            for oc_id, oc in state.outcomes.iteritems():
                state_dict['outcomes'][oc_id] = oc
            for t_id, t in state.transitions.iteritems():
                state_dict['transitions'][t_id] = t
            for df_id, df in state.data_flows.iteritems():
                state_dict['transitions'][df_id] = df
        return state_dict

    def check_if_all_states_there(state, state_dict):
        everything_right = True
        everything_right = everything_right and state.get_path() == state_dict['path']
        if not state.get_path() == state_dict['path']:
            print "path is inconsistent", state.state_id, state.name, state.get_path(), state_dict['path']
        for s_id, s_dict in state_dict['states'].iteritems():
            if s_id in state.states:
                everything_right = everything_right and check_if_all_states_there(state.states[s_id], s_dict)
            else:
                everything_right = False
        if isinstance(state, ContainerState):
            for s_id, s in state.states.iteritems():
                everything_right = everything_right and s_id in state_dict['states']
                if not s_id in state_dict['states']:
                    print "state '%s' not found in '%s %s' list of states %s" % (s_id, state.state_id, state.name, state_dict['states'])
        return everything_right

    def do_check_for_state(state_dict, state_name):
        #############
        # add outcome
        outcome_super = state_dict[state_name].add_outcome('super')
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        # [stored_s_elements, stored_s_m_elements] = store_state_elements(state, state_m)
        check_state_for_all_models(state, state_m)

        ################
        # remove outcome
        state_dict[state_name].remove_outcome(outcome_super)  # new outcome should be the third one
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        state4 = ExecutionState('State4')
        state5 = ExecutionState('State5')

        #############
        # add state
        state_dict[state_name].add_state(state4)
        state_dict[state_name].add_state(state5)
        print state_dict[state_name].states

        state4_path = state4.get_path()
        state5_path = state5.get_path()
        print state_dict[state_name].get_path(), state_dict[state_name].state_id, state_dict[state_name].name
        print state4_path, state4.state_id
        print state5_path, state5.state_id
        # store_state_machine(sm_model, test_history_path1)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)
        # store_state_machine(sm_model, test_history_path2)

        print state4.state_id
        if isinstance(state4.parent.parent, State):
            pstate = sm_model.get_state_model_by_path(state4.parent.parent.get_path())
            print pstate.states.keys(), "\n\n"
            # print pstate.state_id, pstate.name, pstate.get_path()

        # print_all_states_with_path_and_name(state_dict['Container'])
        # exit(1)
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        outcome_state4 = state4.add_outcome('UsedHere')
        outcome_state5 = state5.add_outcome('UsedHere')

        ################
        # add transition from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id
        new_transition_id1 = state_dict[state_name].add_transition(from_state_id=state4.state_id,
                                                                   from_outcome=outcome_state4,
                                                                   to_state_id=state5.state_id,
                                                                   to_outcome=None)
        check_state_for_all_models(state, state_m)
        state_dict[state_name].add_transition(from_state_id=state5.state_id, from_outcome=outcome_state5,
                                              to_state_id=state_dict[state_name].state_id, to_outcome=-1)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        ###################
        # remove transition
        state_dict[state_name].remove_transition(new_transition_id1)  # new outcome should be the third one
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # remove state
        state_dict[state_name].remove_state(state5.state_id)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # add input_data_port
        input_state4 = state4.add_input_data_port("input", "str", "zero")
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #############
        # remove input_data_port
        state4.remove_input_data_port(input_state4)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #############
        # add output_data_port
        output_state4 = state4.add_output_data_port("output", "int")
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #############
        # remove output_data_port
        state4.remove_output_data_port(output_state4)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        input_state4 = state4.add_input_data_port("input", "str", "zero")
        check_state_for_all_models(state, state_m)
        output_state4 = state4.add_output_data_port("output", "int")
        check_state_for_all_models(state, state_m)

        state5 = ExecutionState('State5')
        state_dict[state_name].add_state(state5)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)
        input_par_state5 = state5.add_input_data_port("par", "int", 0)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)
        output_res_state5 = state5.add_output_data_port("res", "int")
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #####################
        # add scoped_variable
        scoped_buffer_nested = state_dict[state_name].add_scoped_variable("buffer", "int")
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #####################
        # remove scoped_variable
        state_dict[state_name].remove_scoped_variable(scoped_buffer_nested)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        #############
        # add data_flow
        new_df_id = state_dict[state_name].add_data_flow(from_state_id=state4.state_id, from_data_port_id=output_state4,
                                                         to_state_id=state5.state_id, to_data_port_id=input_par_state5)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

        ################
        # remove data_flow
        state_dict[state_name].remove_data_flow(new_df_id)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)

    # state_check_dict1 = print_all_states_with_path_and_name(state_dict['Container'])
    do_check_for_state(state_dict, state_name='state1')
    # # do_check_for_state(state_dict, history_ctrl, state_name='state2')
    # # do_check_for_state(state_dict, history_ctrl, state_name='Nested')
    # sm_history.modifications.reset()
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # state_check_dict2 = print_all_states_with_path_and_name(state_dict['Container'])
    do_check_for_state(state_dict, state_name='Container')
    # # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # # assert check_if_all_states_there(state_dict['Container'], state_check_dict2)

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_state_property_models_consistency(caplog):
    ##################
    # state properties
    # TODO the LibraryStateModel and StateModel has to be checked separately like mentioned in the notification-test

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

    # change description

    # change active

    # set_start_state

    # change start_state_id

    # change child_execution
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config
    from rafcon.core.script import Script
    from rafcon.core.states.execution_state import ExecutionState

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()

    state1 = ExecutionState('State1')
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    output_state1 = state1.add_output_data_port("output", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")

    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    input_number_state2 = state2.add_input_data_port("number", "int", 5)
    output_res_state2 = state2.add_output_data_port("res", "int")

    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")

    oc_again_state1 = state1.add_outcome("again")
    oc_counted_state1 = state1.add_outcome("counted")

    oc_done_state2 = state2.add_outcome("done")
    oc_best_state2 = state2.add_outcome("best")
    oc_full_state2 = state2.add_outcome("full")

    oc_great_nested = state_dict['Nested'].add_outcome("great")

    #######################################
    # Properties of State #################
    state_name = 'Nested'

    # name(self, name)
    state_dict['Nested'].name = 'nested'

    # parent(self, parent) State
    state_dict['Nested'].parent = state_dict['State3']

    # input_data_ports(self, input_data_ports) None or dict
    state_dict['Nested'].input_data_ports = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    print "CHECK INPUTS_LIST_ASSIGNMENT"
    check_state_for_all_models(state, state_m)

    # output_data_ports(self, output_data_ports) None or dict
    print "DO OUTPUTS_LIST_ASSIGNMENT"
    state_dict['Nested'].output_data_ports = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    print "CHECK OUTPUTS_LIST_ASSIGNMENT"
    check_state_for_all_models(state, state_m)

    # outcomes(self, outcomes) None or dict
    print "DO OUTCOMES_LIST_ASSIGNMENT"
    state_dict['Nested'].outcomes = state2.outcomes
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    print "CHECK OUTCOMES_LIST_ASSIGNMENT"
    check_state_for_all_models(state, state_m)

    state_dict['Nested'].outcomes = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    check_state_for_all_models(state, state_m)

    # script(self, script) Script
    state_dict['Nested2'].script = Script(parent=state_dict['Nested2'])
    state_dict['Nested2'].script = Script(parent=state_dict['Nested2'])

    # description(self, description) str
    state_dict['Nested'].description = "awesome"

    # # active(self, active) bool
    # state_dict['Nested'].active = True

    # # child_execution(self, child_execution) bool
    # state_dict['Nested'].child_execution = True
    # TODO introduce state.execution_status and so on

    ############################################
    # Properties of ContainerState #############

    # TODO check where all this shit comes from may a observed capsuled set_start_state function in ContainerState will help
    # set_start_state(self, state) State or state_id
    state1_m = sm_model.get_state_model_by_path(state1.get_path())
    state_dict['Nested'].set_start_state(state1_m.state.state_id)
    # set_start_state(self, start_state)
    state2_m = sm_model.get_state_model_by_path(state2.get_path())
    state_dict['Nested'].set_start_state(state2_m.state)

    # states(self, states) None or dict
    state_dict['Nested'].states = {}
    # print "\n\n\n", state_dict['Nested'].states
    # state_nested_m = sm_model.get_state_model_by_path(state_dict['Nested'].get_path())
    # print "\n\n\n", state_nested_m.state.states
    # print "\n\n\n", state_nested_m.states
    # exit(1)
    # assert state_nested_m.states
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    check_state_for_all_models(state, state_m)

    # transitions(self, transitions) None or dict
    state_dict['Nested'].transitions = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    check_state_for_all_models(state, state_m)

    # data_flows(self, data_flows) None or dict
    state_dict['Nested'].data_flows = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    check_state_for_all_models(state, state_m)

    # scoped_variables(self, scoped_variables) None or dict
    state_dict['Nested'].scoped_variables = {}
    state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
    state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
    check_state_for_all_models(state, state_m)

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_outcome_property__models_consistency(caplog):
    ##################
    # outcome properties

    # change name
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()

    def do_check_for_state(state_dict, state_name='Nested'):
        ####################################################
        # modify outcome and generate in previous a observer
        outcome_models_observer_dict = {}
        for outcome_id, outcome in state_dict['Nested2'].outcomes.iteritems():
            if not outcome_id < 0:
                outcome.name = "new_name_" + str(outcome_id)
                # resolve reference
                state_dict['Nested2'] = sm_model.get_state_model_by_path(state_dict['Nested2'].get_path()).state

        ##########################
        # check for ContainerState -> should be unnecessary
        state_model = sm_model.get_state_model_by_path(state_dict['Nested'].get_path())

        ####################################################
        # modify outcome
        outcome_models_observer_dict = {}
        for outcome_id, outcome in state_dict['Nested'].outcomes.iteritems():
            outcome.name = "new_name_" + str(outcome_id)
            # resolve reference
            state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

        # outcome_id(self, outcome_id) -> no data_fow_id setter anymore
        # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
        # state_dict['Nested'].outcomes.values()[0].outcome_id += 10

    # do_check_for_state(state_dict, history_ctrl, state_name='Nested')
    do_check_for_state(state_dict, state_name='Container')

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_transition_property_models_consistency(caplog):
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

    from rafcon.gui.config import global_gui_config
    from rafcon.core.states.execution_state import ExecutionState

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()

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

    new_trans_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id,
                                                       from_outcome=outcome_again_state1,
                                                       to_state_id=state1.state_id,
                                                       to_outcome=None)

    # modify_origin(self, from_state, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].modify_origin(from_state=state2.state_id,
                                                                 from_outcome=oc_full_state2)

    # from_outcome(self, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].from_outcome = oc_done_state2

    # to_state(self, to_state)
    state_dict['Nested'].transitions[new_trans_id].to_state = state2.state_id

    # to_outcome(self, to_outcome)
    # Invalid change of outcome! to_outcome must be None as transition goes to child state
    # state_dict['Nested'].transitions[new_trans_id].to_outcome = oc_great_nested

    # transition_id(self, transition_id) -> no transition_id setter anymore
    # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    # state_dict['Nested'].transitions[new_trans_id].transition_id += 1

    # reset observer and testbed
    state_dict['Nested'].remove_transition(new_trans_id)
    new_df_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id,
                                                    from_outcome=outcome_again_state1,
                                                    to_state_id=state1.state_id,
                                                    to_outcome=None)

    ##### modify from parent state #######
    # modify_transition_from_state(self, transition_id, from_state, from_outcome)
    # state_dict['Nested'].modify_transition_from_state(new_df_id, from_state=state2.state_id,
    #                                                   from_outcome=oc_full_state2)
    state_dict['Nested'].transitions[new_df_id].modify_origin(state2.state_id, oc_full_state2)

    # modify_transition_from_outcome(self, transition_id, from_outcome)
    # state_dict['Nested'].modify_transition_from_outcome(new_df_id, from_outcome=oc_done_state2)
    state_dict['Nested'].transitions[new_df_id].from_outcome = oc_done_state2

    # modify_transition_to_state(self, transition_id, to_state, to_outcome)
    # state_dict['Nested'].modify_transition_to_state(new_df_id, to_state=state1.state_id)
    state_dict['Nested'].transitions[new_df_id].to_state = state1.state_id

    # modify_transition_to_outcome(self, transition_id, to_outcome)
    state_dict['Nested'].transitions[new_df_id].modify_target(to_state=state_dict['Nested'].state_id,
                                                              to_outcome=0)
    state_dict['Nested'].transitions[new_df_id].to_outcome = oc_great_nested

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_input_port_modify_notification(caplog):
    ##################
    # input_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()
    new_input_data_port_id = state_dict['Nested2'].add_input_data_port(name='new_input', data_type='str')

    ################################
    # check for modification of name
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].name = 'changed_new_input_name'

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].data_type = 'int'

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].default_value = 5

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].change_data_type(data_type='str',
                                                                                    default_value='awesome_tool')

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_output_port_modify_notification(caplog):

    ##################
    # output_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()
    new_output_data_port_id = state_dict['Nested2'].add_output_data_port(name='new_output', data_type='str')

    ################################
    # check for modification of name
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].name = 'changed_new_output_name'

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].data_type = 'int'

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].default_value = 5

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].change_data_type(data_type='str',
                                                                                      default_value='awesome_tool')

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_scoped_variable_modify_notification(caplog):
    ##################
    # scoped_variable properties

    # change name

    # change data_type

    # change default_value

    # change datatype
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()
    new_scoped_variable_id = state_dict['Nested'].add_scoped_variable(name='new_output', data_type='str')

    ################################
    # check for modification of name
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].name = 'changed_new_scoped_var_name'
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #####################################
    # check for modification of data_type
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].data_type = 'int'
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #########################################
    # check for modification of default_value
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].default_value = 5
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].change_data_type(data_type='str',
                                                                                   default_value='awesome_tool')

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_data_flow_property_modifications_history(caplog):
    ##################
    # data_flow properties

    # change modify_origin

    # change from_key

    # change modify_target

    # change to_key

    # change data_flow_id

    # modify_transition_from_state

    # modify_transition_from_key

    # modify_transition_to_key

    # modify_transition_to_state
    testing_utils.dummy_gui(None)

    from rafcon.gui.config import global_gui_config
    from rafcon.core.states.execution_state import ExecutionState

    # create testbed
    global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    [state, sm_model, state_dict] = create_models()

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id="STATE2")
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")
    input_number_state2 = state2.add_input_data_port("number", "int", 5)

    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id,
                                                   from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)

    ##### modify from data_flow #######
    # modify_origin(self, from_state, from_key)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(from_state=state1.state_id, from_key=output_state1)
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # from_key(self, from_key)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_target(self, to_state, to_key)
    state_dict['Nested'].data_flows[new_df_id].modify_target(to_state=state2.state_id, to_key=input_par_state2)
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # to_key(self, to_key)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # data_flow_id(self, data_flow_id)  -> no data_fow_id setter anymore
    # state_dict['Nested'].data_flows[new_df_id].data_flow_id += 1
    #
    # # resolve reference
    # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    # new_df_id = state_dict['Nested'].data_flows[new_df_id].data_flow_id  # only if histroy.redo was not run

    # reset observer and testbed
    state_dict['Nested'].remove_data_flow(new_df_id)
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id,
                                                   from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ##### modify from parent state #######
    # modify_data_flow_from_state(self, data_flow_id, from_state, from_key)
    # state_dict['Nested'].modify_data_flow_from_state(new_df_id, from_state=state1.state_id, from_key=output_state1)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(state1.state_id, output_state1)
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_from_key(self, data_flow_id, from_key)
    # state_dict['Nested'].modify_data_flow_from_key(new_df_id, from_key=output_count_state1)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_state(self, data_flow_id, to_state, to_key)
    # state_dict['Nested'].modify_data_flow_to_state(new_df_id, to_state=state2.state_id, to_key=input_par_state2)
    state_dict['Nested'].data_flows[new_df_id].modify_target(state2.state_id, input_par_state2)
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_key(self, data_flow_id, to_key)
    # state_dict['Nested'].modify_data_flow_to_key(new_df_id, to_key=input_number_state2)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2

    sm_model.destroy()
    global_gui_config.load()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main(['-s', __file__])
