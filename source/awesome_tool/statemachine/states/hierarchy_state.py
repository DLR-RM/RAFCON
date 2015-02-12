"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import yaml

from statemachine.states.container_state import ContainerState
from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome
from statemachine.enums import StateType
import statemachine.singleton


class HierarchyState(ContainerState, yaml.YAMLObject):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states again
    """

    yaml_tag = u'!HierarchyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ContainerState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                transitions, data_flows, start_state, scoped_variables, v_checker, path, filename,
                                state_type=StateType.HIERARCHY, check_path=check_path)

        #the execution engine has to be set for some units tests specifically
        self.execution_engine = None

    # the input_data and output_data comes in with a mapping from names to values,
    # to transfer the data to the correct ports, the input_data.port_id has to be retrieved again
    def run(self):

        self.setup_run()

        self.add_input_data_to_scoped_data(self.input_data, self)
        self.add_default_values_of_scoped_variables_to_scoped_data()

        try:
            logger.debug("Starting hierarchy state with id %s and name %s" % (self._state_id, self.name))

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.enter(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            transition = None

            state = self.get_start_state()

            while not state is self:
                if self.preempted:
                    self.final_outcome = Outcome(-2, "preempted")
                    self.active = False
                    return
                # depending on the execution mode pause execution
                execution_signal = None
                if self.execution_engine:
                    execution_signal = self.execution_engine.handle_execution_mode(self)
                else:
                    execution_signal = statemachine.singleton.state_machine_execution_engine.handle_execution_mode(self)
                if execution_signal == "stop":
                    # this will be catched at the end of the run method
                    raise RuntimeError("state stopped")

                logger.debug("Executing next state state with id %s, type %s and name %s" %
                             (state.state_id, str(state.state_type), state.name))
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                #execute the state
                state.run()
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, state.final_outcome)

                while not transition:
                    self._transitions_cv.wait(3.0)
                    if self.preempted:
                        self.final_outcome = Outcome(-2, "preempted")
                        self.active = False
                        return
                    transition = self.get_transition_for_outcome(state, state.final_outcome)
                state = self.get_state_for_transition(transition)

            #handle data for the exit script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
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

            self.final_outcome = self.outcomes[transition.to_outcome]
            self.active = False
            return

        except RuntimeError, e:
            if str(e) == "state stopped":
                logger.debug("State %s was stopped!" % self.name)
            else:
                logger.error("State %s had an internal error: %s" % (self.name, str(e)))
            # notify other threads that wait for this thread to finish
            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
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
                              start_state=dict_representation['start_state'],
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'],
                              check_path=False)