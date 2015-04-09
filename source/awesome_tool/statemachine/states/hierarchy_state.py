"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import yaml
from gtkmvc import Observable

from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.enums import StateType
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.enums import MethodName


class HierarchyState(ContainerState, yaml.YAMLObject):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states on their own
    """

    yaml_tag = u'!HierarchyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ContainerState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                transitions, data_flows, start_state_id, scoped_variables, v_checker, path, filename,
                                state_type=StateType.HIERARCHY, check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the hierarchy state is executed

        The input_data and output_data comes with a mapping from names to values,
        to transfer the data to the correct ports, the input_data.port_id has to be retrieved again
        :return:
        """
        self.setup_run()

        self.add_input_data_to_scoped_data(self.input_data, self)
        self.add_default_values_of_scoped_variables_to_scoped_data()

        try:
            logger.debug("Starting hierarchy state with id %s and name %s" % (self._state_id, self.name))

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.execution_history.add_call_history_item(self, MethodName.ENTRY)
            self.enter(scoped_variables_as_dict)
            self.execution_history.add_return_history_item(self, MethodName.ENTRY)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            transition = None

            state = self.get_start_state(set_final_outcome=True)

            self.child_execution = True
            while state is not self:
                if self.preempted:
                    if self.concurrency_queue:
                        self.concurrency_queue.put(self.state_id)
                    self.final_outcome = Outcome(-2, "preempted")
                    self.active = False
                    self.child_execution = False
                    logger.debug("Exit hierarchy state %s with outcome preempted, as the state itself "
                                 "was preempted!" % self.name)
                    return

                # depending on the execution mode pause execution
                execution_signal = awesome_tool.statemachine.singleton.state_machine_execution_engine.handle_execution_mode(self)
                if execution_signal == "stop":
                    # this will be caught at the end of the run method
                    raise RuntimeError("state stopped")

                logger.debug("Executing next state with id \"%s\", type \"%s\" and name \"%s\"" %
                             (state.state_id, str(state.state_type), state.name))
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                #execute the state
                # TODO: test as it was before: state.run()
                state.start(self.execution_history)
                state.join()
                state.active = False
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, state.final_outcome)

                if transition is None:
                    transition = self.handle_no_transition(state)
                # it the transition is still None, then the state was preempted or aborted, in this case return
                if transition is None:
                    self.child_execution = False
                    return

                state = self.get_state_for_transition(transition)

            self.child_execution = False
            #handle data for the exit script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.execution_history.add_call_history_item(self, MethodName.EXIT)
            self.exit(scoped_variables_as_dict)
            self.execution_history.add_return_history_item(self, MethodName.EXIT)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            self.write_output_data()

            self.check_output_data_type()

            # notify other threads that wait for this thread to finish
            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                return

            # At least one child state was executed (if no child state was executed, the income is connected to an
            # outcome and the final_outcome is set by the get_start_state method)
            if transition is not None:
                self.final_outcome = self.outcomes[transition.to_outcome]
            self.active = False
            logger.debug("Return from hierarchy state %s", self.name)
            return

        except Exception, e:
            if str(e) == "state stopped":
                logger.debug("State %s was stopped!" % self.name)
            else:
                logger.error("State %s had an internal error: %s" % (self.name, str(e)))
            # notify other threads that wait for this thread to finish
            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
            self.child_execution = False
            return

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        node = dumper.represent_mapping(u'!HierarchyState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        return HierarchyState(name=dict_representation['name'],
                              state_id=dict_representation['state_id'],
                              input_data_ports=dict_representation['input_data_ports'],
                              output_data_ports=dict_representation['output_data_ports'],
                              outcomes=dict_representation['outcomes'],
                              states=None,
                              transitions=dict_representation['transitions'],
                              data_flows=dict_representation['data_flows'],
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'],
                              check_path=False)

    @staticmethod
    def copy_state(source_state):
        state_copy = HierarchyState()
        # TODO: copy fields from source_state into the state_copy
        return state_copy