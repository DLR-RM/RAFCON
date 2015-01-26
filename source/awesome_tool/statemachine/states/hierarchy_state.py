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
from statemachine.states.state import StateType
from statemachine.scope import ScopedData
from statemachine.states.state import DataPortType


class HierarchyState(ContainerState, yaml.YAMLObject):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states again
    """

    yaml_tag = u'!HierarchyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 sm_status=None, states=None, transitions=None, data_flows=None, start_state=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None):

        ContainerState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, sm_status, states,
                                transitions, data_flows, start_state, scoped_variables, v_checker, path, filename,
                                state_type = StateType.HIERARCHY)


    # the input_data and output_data comes in with a mapping from names to values,
    # to transfer the data to the correct ports, the input_data.port_id has to be retrieved again
    def run(self):

        #initialize data structures
        input_data = self.input_data
        output_data = self.output_data
        if not isinstance(input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(output_data, dict):
            raise TypeError("states must be of type dict")
        self.check_input_data_type(input_data)
        self.add_input_data_to_scoped_data(input_data, self)
        self.add_scoped_variables_to_scoped_data()

        try:
            logger.debug("Starting hierarchy state with id %s" % self._state_id)

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.enter(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            transition = None

            state = self.get_start_state()

            while not state is self:
                logger.debug("Executing next state state with id %s" % state.state_id)
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                #execute the state
                state.run()
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                #print "Final outcome of state is " + str(state.final_outcome)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, state.final_outcome)

                while not transition:
                    self._transitions_cv.wait(3.0)
                    if self.preempted:
                        self.final_outcome = Outcome(-2, "preempted")
                        return
                    transition = self.get_transition_for_outcome(state, state.final_outcome)
                state = self.get_state_for_transition(transition)

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            #write output data back to the dictionary
            for output_name, value in output_data.iteritems():
                output_port_key = self.get_io_data_port_id_from_name_and_type(output_name, DataPortType.OUTPUT)
                for data_flow_key, data_flow in self.data_flows.iteritems():
                    if data_flow.to_state is self.state_id:
                        if data_flow.to_key == output_port_key:
                            output_data[output_name] =\
                                self.scoped_results[str(data_flow.from_key)+data_flow.from_state].value()

            self.check_output_data_type(output_data)

            #for i in range(5):
            #    if self.preempted:
            #        print str(self.__class__) + " was preempted"
            #        return transition.to_outcome
            #    print i
            #    time.sleep(1.0)

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                return

            self.final_outcome = self.outcomes[transition.to_outcome]
            return

        except RuntimeError:
            self.final_outcome = Outcome(-1, "aborted")
            return

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        print dict_representation
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
                              sm_status=None,
                              states=None,
                              transitions=dict_representation['transitions'],
                              data_flows=dict_representation['data_flows'],
                              start_state=dict_representation['start_state'],
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'])