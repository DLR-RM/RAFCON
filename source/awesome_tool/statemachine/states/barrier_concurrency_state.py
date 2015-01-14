"""
.. module:: barrier_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a barrier concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import yaml

from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome
from concurrency_state import ConcurrencyState
from container_state import ContainerState
from statemachine.states.state import StateType


class BarrierConcurrencyState(ConcurrencyState, yaml.YAMLObject):

    yaml_tag = u'!BarrierConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 sm_status=None, states=None, transitions=None, data_flows=None, start_state=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, sm_status,
                                  states, transitions, data_flows, start_state, scoped_variables, v_checker, path,
                                  filename, state_type = StateType.BARRIER_CONCURRENCY)

    def run(self):

        #initialize data structures
        input_data = self.input_data
        output_data = self.output_data
        if not isinstance(input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(output_data, dict):
            raise TypeError("states must be of type dict")
        #print input_data
        self.check_input_data_type(input_data)
        self.add_dict_to_scoped_data(input_data)
        self.add_scoped_variables_to_scoped_data()
        #print self.scoped_variables

        try:
            logger.debug("Starting preemptive concurrency state with id %s" % self._state_id)
            self.enter()

            #start all threads
            for key, state in self.states.iteritems():
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start()

            #wait for all threads
            for key, state in self.states.iteritems():
                state.join()
                self.update_scoped_variables(state.output_data, state)

            #write output data back to the dictionary
            for output_key, value in output_data.iteritems():
                for data_flow_key, data_flow in self.data_flows.iteritems():
                    if data_flow.to_key == output_key:
                        # should be always the case, although used could specify something else
                        if data_flow.to_state == self.state_id:
                            if data_flow.from_state is self:
                                # get value from a scoped_variable
                                output_data[output_key] = self.scoped_variables[output_key].value()
                            else:
                                # get value from the output of the correct state
                                output_data[output_key] =\
                                    self.states[data_flow.from_state].output_data[data_flow.from_key]

            self.check_output_data_type(output_data)

            self.exit()

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                return

            self.final_outcome = Outcome(0, "success")
            return

        except RuntimeError:
            self.final_outcome = Outcome(-1, "aborted")
            return

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        print dict_representation
        node = dumper.represent_mapping(u'!BarrierConcurrencyState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        return BarrierConcurrencyState(name=dict_representation['name'],
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