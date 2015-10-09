import copy
import sys
import traceback

from gtkmvc import ModelMT, Observable
import yaml

from rafcon.utils import log

from rafcon.statemachine.scope import ScopedVariable
from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.data_flow import DataFlow
from rafcon.statemachine.transition import Transition
from rafcon.statemachine.script import Script
from rafcon.statemachine.states.state import State
from rafcon.statemachine.data_port import DataPort
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState

from rafcon.mvc.models.container_state import ContainerState
import rafcon.mvc.statemachine_helper

from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID
logger = log.get_logger(__name__)


def get_state_tuple(state, state_m=None):
    """ Generates a tuple that holds the state as yaml-strings and its meta data in a dictionary.
    The tuple consists of:
    [0] yaml_str for state,
    [1] dict of child_state tuples,
    [2] script of state,
    [3] dict of model_meta-data of self and elements
    [4] path of state in state machine
    [5] script_text
    #   states-meta - [state-, transitions-, data_flows-, outcomes-, inputs-, outputs-, scopes, states-meta]

    :param rafcon.statemachine.states.state.State state: The state that should be stored
    :return: state_tuple tuple
    """
    state_str = yaml.dump(state)

    # print "++++++++++", state
    state_tuples_dict = {}
    if hasattr(state, "states"):
        # print state.states, "\n"
        for child_state_id, child_state in state.states.iteritems():
            # print "child_state: %s" % child_state_id, child_state, "\n"
            state_tuples_dict[child_state_id] = get_state_tuple(child_state)

    if state_m is not None:
        state_meta_dict = get_state_element_meta(state_m)
    else:
        state_meta_dict = {}

    if not isinstance(state, ExecutionState):
        script_content = "Dummy Script"
        script = Script(state=state)
    else:
        script_content = state.script.script
        script = state.script
    state_tuple = (state_str, state_tuples_dict, script, state_meta_dict, state.get_path(), script_content)

    return state_tuple


# def do_storage_test(state):
#     import os
#     # logger.debug(state.get_file_system_path() + "         " + str(state.get_file_system_path().split('/')))
#     #if child_state.get_file_system_path().split('/')[1] == "tmp" and not os.path.exists(state.get_file_system_path()):
#     if not os.path.exists(state.get_file_system_path()):
#     #     # logger.debug("is tmp")
#     os.makedirs(state.get_file_system_path())
#     script_file = open(os.path.join(state.get_file_system_path(), state.script.filename), "w")
#     #     script_file.write(state.script.script)
#     #     script_file.close()


def get_state_from_state_tuple(state_tuple):

    # print "++++ new state", state_tuple

    # Transitions and data flows are not added, as also states are not added
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    state_info = yaml.load(state_tuple[0])
    if not isinstance(state_info, tuple):
        state = state_info
    else:
        state = state_info[0]
        transitions = state_info[1]
        data_flows = state_info[2]

    if isinstance(state, BarrierConcurrencyState):
        # logger.debug("\n\ninsert decider_state\n\n")
        child_state = get_state_from_state_tuple(state_tuple[1][UNIQUE_DECIDER_STATE_ID])
        # do_storage_test(child_state)
        for t in state.transitions.values():
            if UNIQUE_DECIDER_STATE_ID in [t.from_state, t.to_state]:
                state.remove_transition(t.transition_id)
        try:
            state.add_state(child_state)
        except:
            if not UNIQUE_DECIDER_STATE_ID in state.states:
                logger.error("Could not insert DeciderState!!! while it is in NOT in already!!! {0} {1}".format(
                    UNIQUE_DECIDER_STATE_ID in state.states, child_state.state_id == UNIQUE_DECIDER_STATE_ID))

    state.script = state_tuple[2]
    state.script.script = state_tuple[5]
    #print "------------- ", state
    for child_state_id, child_state_tuple in state_tuple[1].iteritems():
        child_state = get_state_from_state_tuple(child_state_tuple)
        # do_storage_test(child_state)

        # print "++++ new cild", child_state  # child_state_tuple, child_state
        if not child_state.state_id == UNIQUE_DECIDER_STATE_ID:
            try:
                state.add_state(child_state)
            except Exception as e:
                logger.debug(str(e))
                logger.error("try to add state %s to state %s with states %s" % (child_state, state, state.states.keys()))

        def print_states(state):
            if hasattr(state, "states"):
                for state_id, child_state in state.states.iteritems():
                    print child_state.get_path()
                    print_states(child_state)
        # print "got from tuple:"
        # print_states(state)

    # Child states were added, now we can add transitions and data flows
    if isinstance(state_info, tuple):
        state.transitions = transitions
        state.data_flows = data_flows

    return state



def get_state_element_meta(state_model, with_parent_linkage=True, with_prints=False):
    meta_dict = {'state': copy.deepcopy(state_model.meta), 'is_start': False, 'data_flows': {}, 'transitions': {},
                 'outcomes': {}, 'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}, 'states':  {},
                 'related_parent_transitions': {}, 'related_parent_data_flows': {}}
    if with_parent_linkage:
        with_parent_linkage = False
        if not state_model.state.is_root_state:
            state_id = state_model.state.state_id
            for transition_m in state_model.parent.transitions:
                transition = transition_m.transition
                if transition.from_state == state_id or transition.to_state == state_id:
                    meta_dict['related_parent_transitions'][transition.transition_id] = copy.deepcopy(transition_m.meta)
            for data_flow_m in state_model.parent.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    meta_dict['related_parent_data_flows'][data_flow.data_flow_id] = copy.deepcopy(data_flow_m.meta)

    if with_prints:
        print "STORE META for STATE: ", state_model.state.state_id, state_model.state.name
    meta_dict['is_start'] = state_model.is_start
    for elem in state_model.outcomes:
        meta_dict['outcomes'][elem.outcome.outcome_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "outcome: ", elem.outcome.outcome_id, elem.parent.state.outcomes.keys(), meta_dict['outcomes'].keys()
    for elem in state_model.input_data_ports:
        meta_dict['input_data_ports'][elem.data_port.data_port_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "input: ", elem.data_port.data_port_id, elem.parent.state.input_data_ports.keys(), meta_dict['input_data_ports'].keys()
    for elem in state_model.output_data_ports:
        meta_dict['output_data_ports'][elem.data_port.data_port_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "output: ", elem.data_port.data_port_id, elem.parent.state.output_data_ports.keys(), meta_dict['output_data_ports'].keys()

    meta_dict['state'] = copy.deepcopy(state_model.meta)
    if hasattr(state_model, 'states'):
        for state_id, state_m in state_model.states.iteritems():
            meta_dict['states'][state_m.state.state_id] = get_state_element_meta(state_m, with_parent_linkage)
            if with_prints:
                print "FINISHED STORE META for STATE: ", state_id, meta_dict['states'].keys(), state_model.state.state_id
        for elem in state_model.transitions:
            meta_dict['transitions'][elem.transition.transition_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "transition: ", elem.transition.transition_id, elem.parent.state.transitions.keys(), meta_dict['transitions'].keys(), elem.parent.state.state_id
        for elem in state_model.data_flows:
            meta_dict['data_flows'][elem.data_flow.data_flow_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "data_flow: ", elem.data_flow.data_flow_id, elem.parent.state.data_flows.keys(), meta_dict['data_flows'].keys()
        for elem in state_model.scoped_variables:
            meta_dict['scoped_variables'][elem.scoped_variable.data_port_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "scoped_variable: ", elem.scoped_variable.data_port_id, elem.parent.state.scoped_variables.keys(), meta_dict['scoped_variables'].keys()
    return meta_dict


def insert_state_meta_data(meta_dict, state_model, with_parent_linkage=True, with_prints=False):

    # meta_dict = {'state': state_model.meta, 'data_flows': {}, 'transitions': {}, 'outcomes': {},
    #              'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}}

    # if with_parent_linkage:
    #     with_parent_linkage = False
    #     if state_model.parent is not None:  # not root_state
    #         state_id = state_model.state.state_id
    #         for t_id in meta_dict['related_parent_transitions'].keys():
    #             transition_m = state_model.parent.transitions[t_id]
    #             if transition_m.transition.from_state == state_id or transition_m.transition.to_state == state_id:
    #                 transition_m.meta = meta_dict['related_parent_transitions'][t_id]
    #         for df_id in meta_dict['related_parent_data_flows'].keys():
    #             data_flow_m = state_model.parent.data_flows[df_id]
    #             if data_flow_m.data_flow.from_state == state_id or data_flow_m.data_flow.to_state == state_id:
    #                 data_flow_m.meta = meta_dict['related_parent_data_flows'][df_id]

    state_model.meta = copy.deepcopy(meta_dict['state'])
    if with_prints:
        print "INSERT META for STATE: ", state_model.state.state_id, state_model.state.name

    for elem in state_model.outcomes:
        if with_prints:
            print "outcome: ", elem.outcome.outcome_id, meta_dict['outcomes'].keys()
        assert elem.outcome.outcome_id in meta_dict['outcomes']
        elem.meta = copy.deepcopy(meta_dict['outcomes'][elem.outcome.outcome_id])
    for elem in state_model.input_data_ports:
        if with_prints:
            print "input: ", elem.data_port.data_port_id, meta_dict['input_data_ports'].keys()
        assert elem.data_port.data_port_id in meta_dict['input_data_ports']
        elem.meta = copy.deepcopy(meta_dict['input_data_ports'][elem.data_port.data_port_id])
    for elem in state_model.output_data_ports:
        if with_prints:
            print "output: ", elem.data_port.data_port_id, meta_dict['output_data_ports'].keys()
        assert elem.data_port.data_port_id in meta_dict['output_data_ports']
        elem.meta = copy.deepcopy(meta_dict['output_data_ports'][elem.data_port.data_port_id])
    if hasattr(state_model, 'states'):
        for state_id, state_m in state_model.states.iteritems():
            if with_prints:
                print "FIN: ", state_id, state_m.state.state_id, meta_dict['states'].keys(), state_model.state.state_id
            if state_m.state.state_id is not UNIQUE_DECIDER_STATE_ID:
                insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_parent_linkage)
            if with_prints:
                print "FINISHED META for STATE: ", state_m.state.state_id
        for elem in state_model.transitions:
            if with_prints:
                print "transition: ", elem.transition.transition_id, meta_dict['transitions'].keys(), elem.parent.state.transitions.keys(), elem.parent.state.state_id
            # assert elem.transition.transition_id in meta_dict['transitions']
            if elem.transition.transition_id in meta_dict['transitions']:
                elem.meta = copy.deepcopy(meta_dict['transitions'][elem.transition.transition_id])
            else:
                logger.info("Storage Dict seems to miss Meta-Data of Transition in State: %s %s for transition: %s" %
                            (state_model.state.state_id, state_model.state.name, elem.transition))
        for elem in state_model.data_flows:
            if with_prints:
                print "data_flow: ", elem.data_flow.data_flow_id, meta_dict['data_flows'].keys()
            assert elem.data_flow.data_flow_id in meta_dict['data_flows']
            elem.meta = copy.deepcopy(meta_dict['data_flows'][elem.data_flow.data_flow_id])
        for elem in state_model.scoped_variables:
            if with_prints:
                print "scoped: ", elem.scoped_variable.data_port_id, meta_dict['scoped_variables'].keys()
            assert elem.scoped_variable.data_port_id in meta_dict['scoped_variables']
            elem.meta = copy.deepcopy(meta_dict['scoped_variables'][elem.scoped_variable.data_port_id])
    state_model.is_start = copy.deepcopy(meta_dict['is_start'])
    if state_model.is_start and not state_model.state.is_root_state:  # TODO not nice that model stuff does things in core
        if not (isinstance(state_model.parent.state, BarrierConcurrencyState) or
                isinstance(state_model.parent.state, PreemptiveConcurrencyState)):
            # logger.debug("set start_state_id %s" % state_model.parent.state)
            state_model.parent.state.start_state_id = state_model.state.state_id
        else:
            state_model.is_start = False


class ActionDummy:
    def __init__(self):
        pass

    def set_after(self, model, prop_name, info):
        pass

    def undo(self):
        pass

    def redo(self):
        pass


class Action:

    def __init__(self, action_type, parent_path, model, prop_name, info, state_machine_model):

        self.type = action_type
        self.state_machine = state_machine_model.state_machine
        self.state_machine_model = state_machine_model
        self.parent_path = parent_path

        self.before_model = model
        self.before_prop_name = prop_name
        self.before_info = info
        self.before_storage = self.get_storage(model)  # tuple of state and states-list of storage tuple

        self.after_model = None
        self.after_prop_name = None
        self.after_info = None
        self.after_storage = None  # tuple of state and states-list of storage tuple

        self.__version_id = None

    @property
    def version_id(self):
        return self.__version_id

    @version_id.setter
    def version_id(self, value):
        if self.__version_id is None:
            self.__version_id = value
        else:
            logger.warning("The version_id of an action is not allowed to be modify after first assignment")

    def set_after(self, model, prop_name, info):
        self.after_model = model
        self.after_prop_name = prop_name
        self.after_info = info
        self.after_storage = self.get_storage(self.after_model)  # tuple of state and states-list of storage tuple

    def get_storage(self, model):

        if not self.state_machine.get_state_by_path(self.parent_path):
            logger.warning("statemachine could not get state by path -> take root_state")
            state_tuple = get_state_tuple(self.state_machine.root_state)
            state_model = model  # root_state
        else:
            state_tuple = get_state_tuple(self.state_machine.get_state_by_path(self.parent_path))
            state_model = self.state_machine_model.get_state_model_by_path(self.parent_path)
        state_tuple[3].update(get_state_element_meta(state_model))
        return state_tuple

    def redo(self):

        if not self.state_machine.get_state_by_path(self.parent_path) or \
                not self.state_machine.get_state_by_path(self.parent_path).parent:
            # if self.state_machine.get_state_by_path(self.parent_path).parent is None:
            #     logger.info("state is root_state -> take root_state for redo")
            # else:
            #     logger.warning("statemachine could not get state by path -> take root_state for redo")
            state = self.state_machine.root_state
        else:
            state = self.state_machine.get_state_by_path(self.parent_path)

        self.set_state_to_version(state, self.after_storage)

    def undo(self):
        """ General Undo, that takes all elements in the parent and
        :return:
        """

        if not self.state_machine.get_state_by_path(self.parent_path) or \
                not self.state_machine.get_state_by_path(self.parent_path).parent:
            # if self.state_machine.get_state_by_path(self.parent_path).parent is None:
            #     logger.info("state is root_state -> take root_state for undo")
            # else:
            #     logger.warning("statemachine could not get state by path -> take root_state for undo")
            state = self.state_machine.root_state
        else:
            state = self.state_machine.get_state_by_path(self.parent_path)
        self.set_state_to_version(state, self.before_storage)

    def set_state_to_version(self, state, storage_version):
        # print state.get_path(), '\n', storage_version[4]
        assert state.get_path() == storage_version[4]
        # print self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/'))
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        assert storage_version_of_state
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s %s || %s || Action\n\n\n\n\n\n\n" % (path_of_state, state, storage_version_of_state))
        self.update_state(state, storage_version_of_state)
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

    def update_state(self, state, stored_state):

        assert type(stored_state) is type(state)

        is_root = state.is_root_state

        if isinstance(state, ContainerState):

            for data_flow_id in state.data_flows.keys():
                state.remove_data_flow(data_flow_id)

            if not isinstance(state, BarrierConcurrencyState):
                for t_id in state.transitions.keys():
                    # if not UNIQUE_DECIDER_STATE_ID in [state.transitions[t_id].from_state, state.transitions[t_id].to_state]: # funst nicht
                    state.remove_transition(t_id)

            for old_state_id in state.states.keys():
                try:
                    state.remove_state(old_state_id, force=True)
                except Exception as e:
                    print old_state_id, UNIQUE_DECIDER_STATE_ID, state
                    raise e

        if is_root:
            for outcome_id in state.outcomes.keys():
                if not outcome_id < 0:
                    state.remove_outcome(outcome_id)

            for dp_id in state.input_data_ports.keys():
                state.remove_input_data_port(dp_id)

            # print " \n\n\n ########### start removing output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id in state.output_data_ports.keys():
                state.remove_output_data_port(dp_id)

        if hasattr(state, 'states'):
            for dp_id in state.scoped_variables.keys():
                # print "scoped_variable ", dp_id
                state.remove_scoped_variable(dp_id)

        state.name = stored_state.name
        state.script = stored_state.script
        # # logger.debug("script0: " + stored_state.script.script)
        # state.set_script_text(stored_state.script.script)

        if is_root:
            for dp_id, dp in stored_state.input_data_ports.iteritems():
                # print "generate input data port", dp_id
                state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print "got input data ports", dp_id, state.input_data_ports.keys()
                assert dp_id in state.input_data_ports.keys()

            # print " \n\n\n ########### start adding output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id, dp in stored_state.output_data_ports.iteritems():
                scoped_str = str([])
                if hasattr(state, "scoped_variables"):
                    scoped_str = str(state.scoped_variables.keys())
                # print "\n\n\n ------- ############ generate output data port", dp_id, state.input_data_ports.keys(), \
                #     state.output_data_ports.keys(), scoped_str, "\n\n\n"
                state.add_output_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print "\n\n\n ------- ############ got output data ports", dp_id, state.output_data_ports.keys(), "\n\n\n"
                assert dp_id in state.output_data_ports.keys()

            for oc_id, oc in stored_state.outcomes.iteritems():
                # print oc_id, state.outcomes, type(oc_id), oc_id < 0, oc_id == 0, oc_id == -1, oc_id == -2
                if not oc_id < 0:
                    # print "add_outcome", oc_id
                    state.add_outcome(oc.name, oc_id)
            # print "\n\n\n++++++++++++++++ ", stored_state.outcomes, state.outcomes, "\n\n\n++++++++++++++++ "
            for oc_id, oc in stored_state.outcomes.iteritems():
                # print oc_id, state.outcomes
                assert oc_id in state.outcomes

        if hasattr(state, 'states'):
            # logger.debug("UPDATE STATES")
            for dp_id, sv in stored_state.scoped_variables.iteritems():
                state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)

            if UNIQUE_DECIDER_STATE_ID in stored_state.states:
                state.add_state(stored_state.states[UNIQUE_DECIDER_STATE_ID], storage_load=True)

            for new_state in stored_state.states.values():
                # print "++++ new child", new_state
                if not new_state.state_id == UNIQUE_DECIDER_STATE_ID:
                    state.add_state(new_state)
                    state.states[new_state.state_id].script = new_state.script
                    # # logger.debug("script1: " + new_state.script.script)
                    # state.states[new_state.state_id].set_script_text(new_state.script.script)
                    s_path = state.states[new_state.state_id].get_file_system_path()
                    sm_id = self.state_machine.state_machine_id
                    rafcon.statemachine.singleton.global_storage.unmark_path_for_removal_for_sm_id(sm_id, s_path)
                    # print "unmark from removal: ", s_path
                    if hasattr(new_state, 'states'):
                        def unmark_state(state_, sm_id_):
                            spath = state_.get_file_system_path()
                            rafcon.statemachine.singleton.global_storage.unmark_path_for_removal_for_sm_id(sm_id_, spath)
                            # print "unmark from removal: ", spath
                            if hasattr(state_, 'states'):
                                for child_state in state_.states.values():
                                    unmark_state(child_state, sm_id_)
                            # do_storage_test(state_)

                        unmark_state(new_state, sm_id)
                    # check if tmp folder otherwise everthing is Ok

                    # if is -> do check if exists and write the script if not!!!! TODO

            if not (isinstance(state, BarrierConcurrencyState) or
                    isinstance(state, PreemptiveConcurrencyState)):
                for t_id, t in stored_state.transitions.iteritems():
                    # print "\n\n\n++++++++++++++++ ", stored_state.outcomes, state.outcomes, "\n\n\n++++++++++++++++ "
                    # print "### transitions to delete ", [t.from_state, t.to_state], t
                    if not UNIQUE_DECIDER_STATE_ID in [t.from_state, t.to_state]:
                        state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)
            # logger.debug("CHECK TRANSITIONS %s" % state.transitions.keys())
            for t in state.transitions.values():
                # logger.debug(str([t.from_state, t.from_outcome, t.to_state, t.to_outcome]))
                if UNIQUE_DECIDER_STATE_ID == t.from_state and UNIQUE_DECIDER_STATE_ID == t.to_state:
                    state.remove_transition(t.transition_id)

            for df_id, df in stored_state.data_flows.iteritems():
                state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

        # self.before_model.transitions._notify_method_after(state, 'data_flow_change', None, (self.before_model,), {})


class StateMachineAction(Action):

    def __init__(self, action_type, parent_path, model, prop_name, info, state_machine_model):
        Action.__init__(self, action_type, parent_path, model, prop_name, info, state_machine_model)

        self.type = action_type
        self.state_machine = state_machine_model.state_machine
        self.state_machine_model = state_machine_model
        self.parent_path = parent_path

        self.before_model = model
        self.before_prop_name = prop_name
        self.before_info = info
        self.before_storage = self.get_storage(model)  # tuple of state and states-list of storage tuple

        self.after_model = None
        self.after_prop_name = None
        self.after_info = None
        self.after_storage = None  # tuple of state and states-list of storage tuple

    def set_after(self, model, prop_name, info):
        self.after_model = model
        self.after_prop_name = prop_name
        self.after_info = info
        self.after_storage = self.get_storage(self.after_model)  # tuple of state and states-list of storage tuple

    def get_storage(self, model):

        state_tuple = get_state_tuple(self.state_machine.root_state)
        state_tuple[3].update(get_state_element_meta(model))
        return state_tuple

    def set_root_state_to_version(self, state, storage_version):
        import rafcon.mvc.statemachine_helper as statemachine_helper
        # print self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/'))
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s  || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state))
        # self.state_machine.root_state = get_state_from_state_tuple(storage_version)
        root_state_version_fom_storage = get_state_from_state_tuple(storage_version)
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s || %s || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state, root_state_version_fom_storage))
        # actual_state_model = self.state_machine_model.get_state_model_by_path(state.get_path())

        # print "\n#H# TRY STATE_HELPER ", type(root_state_version_fom_storage), \
        #     isinstance(root_state_version_fom_storage, statemachine_helper.HierarchyState), "\n"
        if isinstance(root_state_version_fom_storage, statemachine_helper.HierarchyState):
            new_state_class = statemachine_helper.HierarchyState
        elif isinstance(root_state_version_fom_storage, statemachine_helper.BarrierConcurrencyState):
            new_state_class = statemachine_helper.BarrierConcurrencyState
        elif isinstance(root_state_version_fom_storage, statemachine_helper.PreemptiveConcurrencyState):
            new_state_class = statemachine_helper.PreemptiveConcurrencyState
        else:
            new_state_class = statemachine_helper.ExecutionState
        # logger.debug("DO root version change")
        new_state = statemachine_helper.StateMachineHelper.duplicate_state_with_other_state_type(state, new_state_class)
        self.update_state(new_state, root_state_version_fom_storage)

        # if isinstance(root_state_version_fom_storage, ContainerState):
        #     new_state_model = ContainerStateModel(new_state)
        # else:
        #     new_state_model = StateModel(new_state)

        # insert_state_meta_data(meta_dict=storage_version[3], state_model=new_state_model)
        # self.state_machine_model.root_state = new_state_model
        # self.state_machine.root_state = new_state  # root_state_version_fom_storage

        self.state_machine.root_state = new_state  # root_state_version_fom_storage
        self.state_machine.root_state.script = storage_version[2]
        insert_state_meta_data(meta_dict=storage_version[3], state_model=self.state_machine_model.root_state)

    def redo(self):
        # print "#H# STATE_MACHINE_REDO STARTED"
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.after_storage)
        # print "#H# STATE_MACHINE_REDO FINISHED"

    def undo(self):
        """ General Undo, that takes all elements in the parent and
        :return:
        """
        # print "#H# STATE_MACHINE_UNDO STARTED"
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.before_storage)
        # print "#H# STATE_MACHINE_UNDO FINISHED"


class AddObject(Action):
    def __init__(self, *args):
        Action.__init__(self, *args)

    def redo(self):
        pass

    def undo(self):
        pass


class RemoveObject(Action):
    def __init__(self, *args):
        Action.__init__(self, *args)

    def redo(self):
        pass

    def undo(self):
        pass


class ModifyAttribute(Action):
    def __init__(self, *args):
        Action.__init__(self, *args)

    def redo(self):
        pass

    def undo(self):
        pass


class Group(Action):
    def __init__(self, *args):
        Action.__init__(self, *args)

    def redo(self):
        pass

    def undo(self):
        pass


class UnGroup(Action):
    def __init__(self, *args):
        Action.__init__(self, *args)

    def redo(self):
        pass

    def undo(self):
        pass


class HistoryTreeElement:

    def __init__(self, prev_id, action, next_id, old_next_ids):
        self.prev_id = None
        if prev_id is not None:
            self.prev_id = int(prev_id)
        self.action = action
        self.next_id = next_id
        self.old_next_ids = old_next_ids

    def __str__(self):
        return "prev_id: %s next_id: %s and other next_ids: %s" % (self.prev_id, self.next_id, self.old_next_ids)


class History(ModelMT):

    state_machine_model = None
    changes = None

    __observables__ = ("changes", )

    def __init__(self, state_machine_model):
        ModelMT.__init__(self)

        # assert isinstance(state_machine_model, StateMachineModel)
        self.state_machine_model = state_machine_model
        self.__state_machine_id = state_machine_model.state_machine.state_machine_id
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

        self.observe_model(self.state_machine_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

        self.actual_action = None
        self.actual_root_state_action = None
        self.locked = False
        self.busy = False
        self.count_before = 0

        self.changes = ChangeHistory()

        self.fake = False

        self.with_prints = False

    def get_state_element_meta_from_tmp_storage(self, state_path):
        path_elements = state_path.split('/')
        path_elements.pop(0)
        print path_elements
        act_state_elements_meta = self.tmp_meta_storage
        for path_elem in path_elements:
            act_state_elements_meta = act_state_elements_meta['states'][path_elem]
        print act_state_elements_meta

    def recover_specific_version(self, pointer_on_version_to_recover):
        """ Recovers a specific version of the all_time_history element by doing several undos and redos.

        :param pointer_on_version_to_recover: the id of the list element which is to recover
        :return:
        """
        logger.info("recover version %s of trail state machine edit history" % pointer_on_version_to_recover)
        # search for traceable path -> list of action to undo and list of action to redo
        actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer].version_id
        # logger.debug("actual version_id %s and goal version_id %s" %
        #              (self.changes.all_time_history[actual_version_pointer].action.version_id, pointer_on_version_to_recover))
        undo_redo_list = []
        #backward
        # logger.debug("actual version id %s " % self.changes.trail_history[self.changes.trail_pointer].version_id)
        while not self.changes.trail_pointer == -1 and not int(pointer_on_version_to_recover) == int(actual_version_pointer):
            undo_redo_list.append((actual_version_pointer, 'undo'))
            # logger.info("%s" % self.changes.all_time_history[actual_version_pointer])
            # logger.info(str(self.changes.all_time_history[actual_version_pointer].prev_id))
            actual_version_pointer = self.changes.all_time_history[actual_version_pointer].prev_id

            # logger.info("%s %s %s " % (type(pointer_on_version_to_recover), type(actual_version_pointer), pointer_on_version_to_recover == actual_version_pointer))
            if actual_version_pointer is None:
                # logger.warning("version could not be found 'backward'")
                undo_redo_list = []
                break

        # forward
        # logger.debug("actual version id %s " % self.changes.trail_history[self.changes.trail_pointer].version_id)
        if self.changes.trail_pointer == -1:
            actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer + 1].version_id
            undo_redo_list.append((actual_version_pointer, 'redo'))
        elif self.changes.trail_pointer is None:
            actual_version_pointer = self.changes.trail_history[0].version_id
        else:
            actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer].version_id
        if not undo_redo_list or self.changes.trail_pointer == -1 \
                and self.changes.all_time_history[actual_version_pointer].next_id is not None:
            actual_version_pointer = self.changes.all_time_history[actual_version_pointer].next_id
            undo_redo_list.append((actual_version_pointer, 'redo'))
            while not int(pointer_on_version_to_recover) == int(actual_version_pointer):
                # logger.info("%s" % self.changes.all_time_history[actual_version_pointer])
                # logger.info(str(self.changes.all_time_history[actual_version_pointer].next_id))
                actual_version_pointer = self.changes.all_time_history[actual_version_pointer].next_id

                # logger.info("%s %s %s " % (type(pointer_on_version_to_recover), type(actual_version_pointer), pointer_on_version_to_recover == actual_version_pointer))
                if actual_version_pointer is None:
                    logger.warning("version could not be found 'forward' and 'backward'")
                    undo_redo_list = []
                    break
                undo_redo_list.append((actual_version_pointer, 'redo'))


        # logger.info("found steps to perform %s to reach version_id %s" % (undo_redo_list, pointer_on_version_to_recover))
        for elem in undo_redo_list:
            if elem[1] == 'undo':
                # do undo
                self._undo(elem[0])
            else:
                # do redo
                self._redo(elem[0])
                #self.changes.all_time_history[elem[0]].action.redo()

    def _undo(self, version_id):
        self.busy = True
        self.changes.all_time_history[version_id].action.undo()
        self.changes.trail_pointer -= 1
        self.changes.all_time_pointer -= 1
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer+1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()

    def undo(self):
        if not self.changes.trail_history or self.changes.trail_pointer < 0 \
                or not self.changes.trail_pointer < len(self.changes.trail_history):
            logger.debug("There is no more TrailEditionHistory element to Undo")
            return
        # else:
        #     logger.debug("do Undo %s %s %s" % (bool(self.changes.trail_history), self.changes.trail_history, (self.changes.trail_pointer, len(self.changes.trail_history))))
        self.busy = True
        self.changes.undo()
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer+1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()

    def _redo(self, version_id):
        self.busy = True
        self.changes.all_time_history[version_id].action.redo()
        self.changes.trail_pointer += 1
        self.changes.all_time_pointer += 1
        self.busy = False
        if self.changes.trail_history is not None \
                and self.changes.trail_pointer < len(self.changes.trail_history) \
                and isinstance(self.changes.trail_history[self.changes.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()

    def redo(self):
        if not self.changes.trail_history or self.changes.trail_history and not self.changes.trail_pointer+1 < len(self.changes.trail_history):
            logger.debug("There is no more TrailHistory element to Redo")
            return
        # else:
        #     logger.debug("do Redo %s %s %s" % (bool(self.changes.trail_history), self.changes.trail_history, (self.changes.trail_pointer, len(self.changes.trail_history))))
        self.busy = True
        self.changes.redo()
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()

    def _interrupt_actual_action(self):
        # self.busy = True
        # self.actual_action.undo()
        # self.busy = False
        self.locked = False
        self.count_before = 0

    def _re_initiate_observation(self):
        # logger.info("re initiate root_state observation")
        self.relieve_model(self.__buffered_root_state_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

    def start_new_action(self, overview):
        """

        :param overview:
        :return:
        """
        if self.fake:
            self.actual_action = ActionDummy()
            return True

        if self.with_prints:
            print overview['model']
            print overview['prop_name']
            print overview['instance']
            print overview['method_name']
            print overview['level']
            print overview['prop_name'][-1]
        # exit(1)
        # logger.debug("History stores BEFORE")
        result = True
        cause = overview['method_name'][-1]
        if isinstance(overview['instance'][-1], DataFlow) or \
                isinstance(overview['instance'][-1], Transition) or \
                isinstance(overview['instance'][-1], ScopedVariable):

            # print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
            #     overview['model'][-1].parent.state.get_path()
            assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.state.get_path().split('/')[0]
            overview['model'][-1].parent.state.get_path()
            # print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
            #     overview['model'][-1].parent.state.get_path()
            assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.state.get_path().split('/')[0]
            # the model should be StateModel or ContainerStateModel and "info" from those model notification
            # logger.debug("State-Element changed %s in State %s" % (overview['instance'][-1],
            #                                                        overview['model'][-1].parent.state.get_path()))
            # self.actual_action = Action(info.method_name, model.state.get_path(),
            #                             model, prop_name, info, state_machine=self._selected_sm_model.state_machine)
            self.actual_action = Action(cause, overview['model'][-1].parent.state.get_path(),  # instance path of parent
                                        overview['model'][0], overview['prop_name'][0], overview['info'][-1],
                                        state_machine_model=self.state_machine_model)

        elif isinstance(overview['instance'][-1], DataPort) or \
                isinstance(overview['instance'][-1], Outcome) or \
                overview['method_name'][-1] in ['add_outcome', 'remove_outcome',
                                                'add_output_data_port', 'remove_output_data_port',
                                                'add_input_data_port', 'remove_input_data_port']:
            if self.with_prints:
                print overview['model']
                print overview['prop_name']
                print overview['instance']
                print overview['method_name']
                print overview['level']
                print overview['prop_name'][-1]

            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):
                    if self.with_prints:
                        print "Path_root: ", overview['model'][-1].state.get_path()
                    # exit(1)
                    # logger.debug("State-Element changed %s in State %s" % (overview['instance'][-1],
                    #                                                        overview['model'][-1].state.get_path()))
                    # self.actual_action = Action(info.method_name, model.state.get_path(),
                    #                             model, prop_name, info, state_machine=self._selected_sm_model.state_machine)
                    self.actual_action = Action(cause, overview['model'][-1].state.get_path(),  # instance path of parent
                                                overview['model'][0], overview['prop_name'][0], overview['info'][-1],
                                                state_machine_model=self.state_machine_model)
                elif not isinstance(overview['model'][-1].parent.state.parent, State):  # is root_state
                    if self.with_prints:
                        print "Path_root: ", overview['model'][-1].parent.state.get_path()
                    # exit(1)
                    # logger.debug("State-Element changed %s in State %s" % (overview['instance'][-1],
                    #                                                        overview['model'][-1].parent.state.get_path()))
                    # self.actual_action = Action(info.method_name, model.state.get_path(),
                    #                             model, prop_name, info, state_machine=self._selected_sm_model.state_machine)
                    self.actual_action = Action(cause, overview['model'][-1].parent.state.get_path(),  # instance path of parent
                                                overview['model'][0], overview['prop_name'][0], overview['info'][-1],
                                                state_machine_model=self.state_machine_model)
                else:
                    if self.with_prints:
                        print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                            overview['model'][-1].parent.state.get_path()
                    assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.parent.state.get_path().split('/')[0]
                    overview['model'][-1].parent.state.get_path()
                    if self.with_prints:
                        print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                            overview['model'][-1].parent.state.get_path()
                    assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.parent.state.get_path().split('/')[0]
                    # the model should be StateModel or ContainerStateModel and "info" from those model notification
                    # logger.debug("State-Element changed %s in State %s" % (overview['instance'][-1],
                    #                                                        overview['model'][-1].parent.state.get_path()))
                    # self.actual_action = Action(info.method_name, model.state.get_path(),
                    #                             model, prop_name, info, state_machine=self._selected_sm_model.state_machine)
                    self.actual_action = Action(cause, overview['model'][-1].parent.parent.state.get_path(),  # instance path of parent
                                                overview['model'][0], overview['prop_name'][0], overview['info'][-1],
                                                state_machine_model=self.state_machine_model)
                    # exit(1)

        elif overview['prop_name'][-1] == 'state':
            if self.with_prints:
                print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
            # logger.debug("State-Element changed %s in State %s" % (overview['instance'][-1],
            #                                                        overview['model'][-1].state.get_path()))
            self.actual_action = Action(cause, overview['model'][-1].state.get_path(),  # instance path of parent
                                        overview['model'][0], overview['prop_name'][0], overview['info'][-1],
                                        state_machine_model=self.state_machine_model)

        else:  # FAILURE  # is root_state
            # self.actual_action = Action(info.method_name, '/',
            #                             model, prop_name, info, state_machine=self._selected_sm_model.state_machine)
            logger.warn("History may need update, tried to start observation of new action that is not classifiable "
                        "\n%s \n%s \n%s \n%s",
                         overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
            return False

        return result

    def finish_new_action(self, model, prop_name, info):
        # logger.debug("History stores AFTER")
        try:
            self.actual_action.set_after(model, prop_name, info)
            self.state_machine_model.history.changes.insert_action(self.actual_action)
            # logger.debug("history is now: %s" % self.state_machine_model.history.changes.single_trail_history())
            self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)
        except:
            logger.debug("Failure occurred while finishing action")
            traceback.print_exc(file=sys.stdout)

    def meta_changed_notify_after(self, changed_parent_model, changed_model, recursive_changes):
        self.manual_changed_notify_after("gui_meta_data_changed", changed_parent_model, changed_model, recursive_changes)

    def manual_changed_notify_before(self, change_type, changed_parent_model, changed_model, recursive_changes):
        pass

    def manual_changed_notify_after(self, change_type, changed_parent_model, changed_model, recursive_changes):
        """
        :param changed_parent_model rafcon.mvc.models.container_state.ContainerStateModel: model that holds the changed model
        :param changed_model gtkmvc.Model: inherent class of gtkmvc.Model like TransitionModel, StateModel and so on
        :param recursive_changes bool: indicates if the changes are recursive and touch multiple or all recursive childs
        :return:
        """
        if change_type == 'gui_meta_data_changed':
            # store meta data

            logger.debug("state %s '%s' history got notification that Meta data has changed" %
                         (changed_model.state.state_id, changed_model.state.name))
            # -> in case of undo/redo overwrite Model.meta-dict

            # self.actual_action = Action('meta_data_changed', changed_parent_model.state.get_path(),  # instance path of parent
            #                             changed_model, 'meta_data_changed', {},
            #                             state_machine_model=self.state_machine_model)
            # b_tuple = self.actual_action.before_storage
            # meta_dict = self.get_state_element_meta_from_tmp_storage(changed_model.state.get_path())
            # mod_tuple = (b_tuple[0], b_tuple[1], b_tuple[2], meta_dict, b_tuple[4], b_tuple[5])
            # self.actual_action.before_storage = mod_tuple
            # self.finish_new_action(changed_model, 'meta_data_changed', {})

    @ModelMT.observe("state_machine", before=True)
    def assign_notification_change_type_root_state_before(self, model, prop_name, info):
        # print model, prop_name, info
        if self.busy:  # if doing undo and redos
            return
        if info.method_name == "change_root_state_type":
            if self.with_prints:
                print "ROOT_STATE is NEW", model, prop_name, info
            self.actual_action = StateMachineAction("change_root_state_type", model.state_machine.root_state.get_path(),  # instance path of parent
                                                    model.root_state, prop_name, info,
                                                    state_machine_model=self.state_machine_model)
            self.count_before += 1
            if self.with_prints:
                print "LOCKED count up state_machine", self.count_before
            self.locked = True

    @ModelMT.observe("state_machine", after=True)
    def assign_notification_change_type_root_state_after(self, model, prop_name, info):
        # print model, prop_name, info
        if info.result == "CRASH in FUNCTION":
            if self.with_prints:
                logger.warning("function crash detected sm_after")
            return self._interrupt_actual_action()

        if self.busy:  # if doing undo and redos
            return

        if info.method_name == "change_root_state_type":
            # logger.debug("History state_machine_AFTER")
            if self.with_prints:
                print "ROOT_STATE is NEW", model, prop_name, info
            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down state_machine", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info
                    if prop_name == "states" and info.kwargs.result == "CRASH in FUNCTION":
                        if self.with_prints:
                            print "HISTORY COUNT WAS 0 AND RESET FAILURE to the previous version of the state machine"
                        self.actual_action.undo()
                    else:
                        self.finish_new_action(model.root_state, prop_name, info)
                        self._re_initiate_observation()
                        if self.with_prints:
                            print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"

    @ModelMT.observe("states", before=True)
    def assign_notification_states_before(self, model, prop_name, info):
        if self.with_prints:
            print "states_before: ", model, prop_name, info
        if self.busy:  # if doing undo and redo's
            return
        else:
            # logger.debug("History states_BEFORE")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = parent_state_of_notification_source(model, prop_name, info, before_after='before', with_prints=self.with_prints)

            # skipped state changes
            if overview['method_name'][0] == 'state_change' and \
                    overview['method_name'][-1] in ['active', 'child_execution', 'state_execution_status']:
                # print overview['method_name']
                return

            # lock changes
            if self.locked:
                self.count_before += 1
                if self.with_prints:
                    print "LOCKED count up", self.count_before
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT", info
                # if self.start_new_action(parent_info,
                #                          info['model'], info['prop_name'], info,
                #                          cause, root_cause_is_state):
                if self.start_new_action(overview):
                    self.count_before += 1
                    if self.with_prints:
                        print "LOCKED count up", self.count_before
                    self.locked = True
                else:
                    if self.with_prints:
                        print "FAILED to start NEW HISTORY ELEMENT"

    @ModelMT.observe("states", after=True)
    def assign_notification_states_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if self.with_prints:
            print "states_after: ", model, prop_name, info
        if hasattr(info, "kwargs") and info.kwargs and info.kwargs['result'] == "CRASH in FUNCTION":
            if self.with_prints:
                logger.warning("function crash detected states_after")
            return self._interrupt_actual_action()

        if self.busy or info.method_name == 'state_change' and \
                        info.kwargs.prop_name == 'state' and \
                        info.kwargs.method_name in ['active', 'state_execution_status']:
            return
        else:
            # logger.debug("History states_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = parent_state_of_notification_source(model, prop_name, info, before_after='after', with_prints=self.with_prints)
            if overview['method_name'][0] == 'state_change' and \
                    overview['method_name'][-1] in ['active', 'child_execution', 'state_execution_status']:
                if self.with_prints:
                    print overview['method_name']
                return

            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info
                    # SEG-FAULT
                    # if prop_name == "states" and info.kwargs.result == "CRASH in FUNCTION":
                    #     if self.with_prints:
                    #         print "HISTORY COUNT WAS 0 AND RESET FAILURE to the previous version of the state machine"
                    # else:
                    self.finish_new_action(model, prop_name, info)
                    if self.with_prints:
                        print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"

    @ModelMT.observe("state", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("output_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def assign_notification_root_state_before(self, model, prop_name, info):
        if self.with_prints:
            print "root_state_before: ", model, prop_name, info
        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        # first element should be prop_name="state_machine", instance=StateMachine and model=StateMachineModel
        # second element should be Prop_name="states" if root_state child elements are changed
        # --- for root_state elements it has to be prop_name in ["data_flows", "transitions", "input_data_ports",
        #                                                        "output_data_ports", "scoped_variables"]
        # third (and last element) should be prop_name in ["data_flow", "transition",
        else:
            # logger.debug("History BEFORE")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = parent_state_of_notification_source(model, prop_name, info, before_after='before', with_prints=self.with_prints)
            cause = overview['method_name'][-1]
            parent_info = overview['info'][0]
            parent_model = overview['model'][0]
            root_cause_is_state = parent_model.state.is_root_state

            # print "IN HISTORY", info
            #print "states changed ", info.prop_name, info.method_name
            if self.locked:
                self.count_before += 1
                if self.with_prints:
                    print "LOCKED count up", self.count_before
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT", info
                # if self.start_new_action(parent_info,
                #                          info['model'], info['prop_name'], info,
                #                          cause, root_cause_is_state):
                if self.start_new_action(overview):
                    self.count_before += 1
                    if self.with_prints:
                        print "LOCKED count up", self.count_before
                    self.locked = True
                else:
                    if self.with_prints:
                        print "FAILED to start NEW HISTORY ELEMENT"

    @ModelMT.observe("state", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def assign_notification_root_state_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if self.with_prints:
            print "root_state_after: ", model, prop_name, info

        if info.result == "CRASH in FUNCTION":
            if self.with_prints:
                logger.warning("function crash detected state_after")
            return self._interrupt_actual_action()

        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        else:
            # logger.debug("History state_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = parent_state_of_notification_source(model, prop_name, info, before_after='before', with_prints=self.with_prints)
            cause = overview['method_name'][-1]
            parent_info = overview['info'][0]
            parent_model = overview['model'][0]
            root_cause_is_state = parent_model.state.is_root_state

            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info
                    if prop_name == "states" and info.kwargs.result == "CRASH in FUNCTION":
                        if self.with_prints:
                            print "HISTORY COUNT WAS 0 AND RESET FAILURE to the previous version of the state machine"
                        self.actual_action.undo()
                    else:
                        self.finish_new_action(model, prop_name, info)
                        if self.with_prints:
                            print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"


def parent_state_of_notification_source(model, prop_name, info, before_after, with_prints):
    if with_prints:
        print "----- xxxxxxx %s \n%s\n%s\n%s\n" % (before_after, model, prop_name, info)

    def set_dict(info, d):
        d['model'].append(info['model'])
        d['prop_name'].append(info['prop_name'])
        d['instance'].append(info['instance'])
        d['method_name'].append(info['method_name'])
        if with_prints:
            print "set"

    def find_parent(info, elem):
        elem['info'].append(info)
        if 'kwargs' in info and info['kwargs']:
            if with_prints:
                print 'kwargs'
            elem['level'].append('kwargs')
            set_dict(info, elem)
            if 'method_name' in info['kwargs'] and 'instance' in info['kwargs']:
                find_parent(info['kwargs'], elem)
        elif 'info' in info and info['info']:
            if with_prints:
                print 'info'
            elem['level'].append('info')
            set_dict(info, elem)
            find_parent(info['info'], elem)
        elif 'info' in info:
            set_dict(info, elem)
        elif 'kwargs' in info:
            set_dict(info, elem)
        else:
            if with_prints:
                print 'assert'
            assert True
        return elem

    overview = find_parent(info, {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'level': [],
                                  'info': []})
    info_print = ''
    info_count = 0
    for elem in overview['info']:
        info_print += "\ninfo %s: %s" % (info_count, str(elem))
        info_count += 1
    if with_prints:
        print info_print
        print "model: ", overview['model']
        print "prop_: ", overview['prop_name']
        print "insta: ", overview['instance']
        print "metho: ", overview['method_name']
        print "level: ", overview['level']
        print "prop-: ", overview['prop_name'][-1]

    if overview['prop_name'][-1] == 'state':
        # print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
        assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
    else:
        if overview['model'][-1].parent:
            if not isinstance(overview['model'][-1].parent.state, State):  # is root_state
                overview['model'][-1].state.get_path()
                if with_prints:
                    print "Path_root: ", overview['model'][-1].state.get_path()
            else:
                overview['model'][-1].parent.state.get_path()
                if with_prints:
                    print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                        overview['model'][-1].parent.state.get_path()
                assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.state.get_path().split('/')[0]
    return overview


class ChangeHistory(Observable):
    """This Class should remember all historical changes insert. Basic functionalities provide typical store change,
    undo and redo functionalities. Additionally there will be implemented functionalities that never forget a single
    change that was insert for debugging reasons.
    - the pointer are pointing on the next undo ... so redo is pointer + 1
    - all_actions is a type of a tree # prev_id, action, next_id, old_next_ids
    """

    def __init__(self):
        Observable.__init__(self)
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None
        self.all_time_pointer = None
        self.counter = 0

        self.with_prints = False

    @Observable.observed
    def insert_action(self, action):
        # insert new element in
        action.version_id = self.counter
        self.all_time_history.append(HistoryTreeElement(prev_id=self.all_time_pointer,
                                                        action=action, next_id=None, old_next_ids=[]))
        self.counter += 1

        # set pointer of previous element
        if self.all_time_pointer is not None:
            prev_action = self.all_time_history[self.all_time_pointer]
            prev_action.old_next_ids.append(prev_action.next_id)
            prev_action.next_id = len(self.all_time_history) - 1

        # do single trail history
        if self.trail_pointer is not None:
            if self.trail_pointer > len(self.trail_history) - 1 or self.trail_pointer < 0:
                logger.error('History is broken may!!! %s' % self.trail_pointer)
            while not self.trail_pointer == len(self.trail_history) - 1:
                if self.with_prints:
                    print "pointer: %s %s" % (self.trail_pointer, len(self.trail_history))
                self.trail_history.pop()
        self.trail_history.append(action)

        # print '\n\n\n\n\n################ PUT pointer ON: Trail: %s All Time History: %s\n\n\n\n\n' % \
        #       (len(self.trail_history) - 1, len(self.all_time_history) - 1)
        self.trail_pointer = len(self.trail_history) - 1
        if self.trail_pointer == -1:
            self.trail_pointer = None
        self.all_time_pointer = len(self.all_time_history) - 1  # general should be equal to self.counter and version_id

    @Observable.observed
    def undo(self):
        # logger.debug("try undo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_pointer is not None and not self.trail_pointer < 0:
            self.trail_history[self.trail_pointer].undo()
            self.trail_pointer -= 1
            self.all_time_pointer -= 1
        elif self.trail_pointer is not None:
            logger.warning("No UNDO left over!!!")
        else:
            logger.error("History undo FAILURE")

    @Observable.observed
    def redo(self):
        # logger.debug("try redo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_history is not None and self.trail_pointer + 1 < len(self.trail_history):
            self.trail_history[self.trail_pointer+1].redo()
            self.trail_pointer += 1
            self.all_time_pointer += 1
        elif self.trail_history is not None:
            logger.warning("No REDO left over!!!")
        else:
            logger.error("History redo FAILURE")

    def single_trail_history(self):
        if self.is_end():
            return self.trail_history  # [:]
        else:
            if self.with_prints:
                print "pointer: ", str(self.trail_pointer)
            return self.trail_history  # [:self.trail_pointer + 1]

    def is_end(self):
        return len(self.trail_history)-1 == self.trail_pointer

    @Observable.observed
    def reset(self):
        logger.debug("################ RESET ChangeHistory PUT ALL TO INITIATION")
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None
        self.all_time_pointer = None
        self.counter = 0
