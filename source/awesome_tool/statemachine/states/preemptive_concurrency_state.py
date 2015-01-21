"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import yaml
import Queue

from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome
from concurrency_state import ConcurrencyState
from container_state import ContainerState
from statemachine.states.state import StateType


class PreemptiveConcurrencyState(ConcurrencyState, yaml.YAMLObject):


    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None, sm_status=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, sm_status, states,
                                  transitions, data_flows, start_state, scoped_variables, v_checker, path, filename,
                                  state_type = StateType.PREEMPTION_CONCURRENCY)

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

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            #print "Printing Scoped Variables Dict:"
            #print scoped_variables_as_dict
            self.enter(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            #infinite Queue size
            concurrency_queue = Queue.Queue(maxsize=0)

            queue_ids = 0
            for key, state in self.states.iteritems():
                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = queue_ids
                queue_ids +=1

                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start()

            finished_thread_id = concurrency_queue.get()
            self.states[finished_thread_id].join()
            self.update_scoped_variables(self.states[finished_thread_id].output_data, self.states[finished_thread_id])

            for key, state in self.states.iteritems():
                state.preempted = True

            for key, state in self.states.iteritems():
                state.join()

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
                                # get value from the output of the state that finished first
                                output_data[output_key] =\
                                    self.states[finished_thread_id].output_data[data_flow.from_key]

            self.check_output_data_type(output_data)

            #reset concurrency queue and preempted flag for all child states
            for key, state in self.states.iteritems():
                state.concurrency_queue = None
                state.preempted = False

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

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
        node = dumper.represent_mapping(u'!PreemptiveConcurrencyState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        return PreemptiveConcurrencyState(name=dict_representation['name'],
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