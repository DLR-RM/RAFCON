"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import Queue

import yaml

from awesome_tool.utils import log

logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.enums import StateType
from awesome_tool.statemachine.enums import MethodName


class PreemptiveConcurrencyState(ConcurrencyState, yaml.YAMLObject):
    """ The preemptive concurrency state has a set of substates which are started when the preemptive concurrency state
    executes. The execution of preemptive concurrency state waits for the first substate to return, preempts all other
    substates and finally returns self.

    """

    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                  transitions, data_flows, start_state_id, scoped_variables, v_checker, path, filename,
                                  state_type=StateType.PREEMPTION_CONCURRENCY, check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the preemptive concurrency state is executed

        :return:
        """
        self.setup_run()

        try:
            logger.debug("Starting preemptive concurrency state with id %s" % self._state_id)

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.execution_history.add_call_history_item(self, MethodName.ENTRY)
            self.enter(scoped_variables_as_dict)
            self.execution_history.add_return_history_item(self, MethodName.ENTRY)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            self.child_execution = True
            #infinite Queue size
            concurrency_queue = Queue.Queue(maxsize=0)

            history_item = self.execution_history.add_concurrency_history_item(self, len(self.states))
            queue_ids = 0
            history_index = 0
            for key, state in self.states.iteritems():
                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = queue_ids
                queue_ids += 1

                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start(history_item.execution_histories[history_index])
                history_index += 1

            finished_thread_id = concurrency_queue.get()
            self.states[finished_thread_id].join()

            self.add_state_execution_output_to_scoped_data(self.states[finished_thread_id].output_data,
                                                           self.states[finished_thread_id])
            self.update_scoped_variables_with_output_dictionary(self.states[finished_thread_id].output_data,
                                                                self.states[finished_thread_id])

            for key, state in self.states.iteritems():
                self.recursively_preempt_states(state)

            for key, state in self.states.iteritems():
                state.join()
                state.concurrency_queue = None

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

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            # check the outcomes of the finished state for aborted or preempted
            # the output data has to be set before this check can be done
            if self.states[finished_thread_id].final_outcome.outcome_id == -1:
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                return
            if self.states[finished_thread_id].final_outcome.outcome_id == -2:
                self.final_outcome = Outcome(-1, "aborted")
                self.active = False
                return

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                return

            transition = self.get_transition_for_outcome(self.states[finished_thread_id],
                                                         self.states[finished_thread_id].final_outcome)

            if transition is None:
                transition = self.handle_no_transition(self.states[finished_thread_id])
            # it the transition is still None, then the state was preempted or aborted, in this case return
            if transition is None:
                return

            self.final_outcome = self.outcomes[transition.to_outcome]
            self.active = False
            return

        except Exception, e:
            logger.error("Runtime error %s" % e)
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
            self.child_execution = False
            return


    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
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
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'],
                              check_path=False)

    @staticmethod
    def copy_state(source_state):
        state_copy = PreemptiveConcurrencyState()
        # TODO: copy fields from source_state into the state_copy
        return state_copy