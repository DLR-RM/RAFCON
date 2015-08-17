"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import Queue
import traceback

from awesome_tool.utils import log

logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.enums import StateExecutionState
from awesome_tool.statemachine.execution.execution_history import ConcurrencyItem


class PreemptiveConcurrencyState(ConcurrencyState):
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
                                  check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the preemptive concurrency state is executed

        :return:
        """
        self.setup_run()

        try:
            #######################################################
            # start threads
            #######################################################

            if self.backward_execution:
                logger.debug("Backward executing preemptive concurrency state with id %s and name %s" % (self._state_id, self.name))

                history_item = self.execution_history.get_last_history_item()
                assert isinstance(history_item, ConcurrencyItem)
                # history_item.state_reference must be "self" in this case

            else:  # forward_execution
                logger.debug("Executing preemptive concurrency state with id %s" % self._state_id)
                history_item = self.execution_history.add_concurrency_history_item(self, len(self.states))

            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size
            queue_ids = 0
            history_index = 0
            for key, state in self.states.iteritems():
                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = queue_ids
                queue_ids += 1

                state_input = self.get_inputs_for_state(state)
                state_output = self.create_output_dictionary_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start(history_item.execution_histories[history_index], self.backward_execution)
                history_index += 1

            #######################################################
            # wait for the first threads to finish
            #######################################################

            finished_thread_id = concurrency_queue.get()
            self.states[finished_thread_id].join()

            # in the backward executing case, only backward execute the entry function and return
            if self.states[finished_thread_id].backward_execution:
                # wait until all states finished their jobs in the backward execution case
                for key, state in self.states.iteritems():
                    state.join()
                    state.state_execution_status = StateExecutionState.INACTIVE

                # backward_execution needs to be True to signal the parent container state the backward execution
                self.backward_execution = True
                logger.debug("Backward-executing entry script of preemption concurrency state %s" % self.name)
                # pop the ConcurrencyItem as we are leaving the barrier concurrency state
                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ConcurrencyItem)

                # do not write the output of the entry script
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                return self.finalize()

            else:
                self.backward_execution = False

            # preempt all child states
            for state_id, state in self.states.iteritems():
                state.recursively_preempt_states()

            for key, state in self.states.iteritems():
                state.join()
                state.concurrency_queue = None
                # add the data of all child states to the scoped data and the scoped variables
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                state.state_execution_status = StateExecutionState.INACTIVE

            self.write_output_data()
            self.check_output_data_type()

            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.preempted:
                outcome = Outcome(-2, "preempted")

            else:
                transition = self.get_transition_for_outcome(self.states[finished_thread_id],
                                                             self.states[finished_thread_id].final_outcome)

                if transition is None:
                    transition = self.handle_no_transition(self.states[finished_thread_id])
                # it the transition is still None, then the state was preempted or aborted, in this case return
                if transition is None:
                    outcome = Outcome(-1, "aborted")
                    self.output_data["error"] = RuntimeError("state aborted")
                else:
                    outcome = self.outcomes[transition.to_outcome]

            return self.finalize(outcome)

        except Exception, e:
            logger.error("Runtime error: {0}\n{1}".format(e, str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state = PreemptiveConcurrencyState(name=dict_representation['name'],
                                           state_id=dict_representation['state_id'],
                                           input_data_ports=dict_representation['input_data_ports'],
                                           output_data_ports=dict_representation['output_data_ports'],
                                           outcomes=dict_representation['outcomes'],
                                           states=None,
                                           transitions=None,
                                           data_flows=None,
                                           scoped_variables=dict_representation['scoped_variables'],
                                           v_checker=None,
                                           check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state, dict_representation['transitions'], dict_representation['data_flows']

    def _check_transition_validity(self, check_transition):
        # Transition of BarrierConcurrencyStates must least fulfill the condition of a ContainerState
        # Start transitions are already forbidden in the ConcurrencyState
        valid, message = super(PreemptiveConcurrencyState, self)._check_transition_validity(check_transition)
        if not valid:
            return False, message

        # Only transitions to the parent state are allowed

        if check_transition.to_state != self.state_id:
            return False, "Only transitions to the parent state are allowed"

        return True, message
