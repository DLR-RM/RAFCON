"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import Queue

import yaml

from utils import log

logger = log.get_logger(__name__)
from statemachine.outcome import Outcome
from concurrency_state import ConcurrencyState
from container_state import ContainerState
from statemachine.enums import StateType


class PreemptiveConcurrencyState(ConcurrencyState, yaml.YAMLObject):

    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                  transitions, data_flows, start_state, scoped_variables, v_checker, path, filename,
                                  state_type=StateType.PREEMPTION_CONCURRENCY, check_path=check_path)

    def run(self):
        self.setup_run()

        self.add_input_data_to_scoped_data(self.input_data, self)
        self.add_default_values_of_scoped_variables_to_scoped_data()

        try:
            logger.debug("Starting preemptive concurrency state with id %s" % self._state_id)

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
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

            #print "++++++++ output of execution state: ", self.states[finished_thread_id].output_data

            self.add_state_execution_output_to_scoped_data(self.states[finished_thread_id].output_data,
                                                           self.states[finished_thread_id])
            self.update_scoped_variables_with_output_dictionary(self.states[finished_thread_id].output_data,
                                                                self.states[finished_thread_id])

            for key, state in self.states.iteritems():
                self.recursively_preempt_states(state)

            for key, state in self.states.iteritems():
                state.join()

            #handle data for the exit script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            #print "+++++++ scoped data of the preemptive container state %s: %s" % (self.name, str(self.scoped_data))
            # for key, value in self.scoped_data.iteritems():
            #     print "Scoped Data key %s: %s" % (key, str(value))

            self.write_output_data()

            #print "+++++ output of the preemptive container state: ", self.output_data

            self.check_output_data_type()

            # check the outcomes of the finished state for aborted or preempted
            # the output data has to be set before this check can be done
            if self.states[finished_thread_id].final_outcome.outcome_id == -1:
                self.final_outcome = Outcome(-1, "preempted")
                self.active = False
                return
            if self.states[finished_thread_id].final_outcome.outcome_id == -2:
                self.final_outcome = Outcome(-2, "aborted")
                self.active = False
                return

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                return

            transition = self.get_transition_for_outcome(self.states[finished_thread_id],
                                                         self.states[finished_thread_id].final_outcome)
            # wait until connection is created by user
            while not transition:
                self._transitions_cv.wait(3.0)
                if self.preempted:
                    self.final_outcome = Outcome(-2, "preempted")
                    self.active = False
                    return
                transition = self.get_transition_for_outcome(self.states[finished_thread_id],
                                                             self.states[finished_thread_id].final_outcome)

            self.final_outcome = self.outcomes[transition.to_outcome]
            self.active = False
            return

        except RuntimeError, e:
            logger.error("Runtime error %s" % e)
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
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
                              states=None,
                              transitions=dict_representation['transitions'],
                              data_flows=dict_representation['data_flows'],
                              start_state=dict_representation['start_state'],
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'],
                              check_path=False)