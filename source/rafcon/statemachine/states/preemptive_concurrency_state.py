"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import Queue
import traceback

from rafcon.statemachine.state_elements.outcome import Outcome
from rafcon.statemachine.states.concurrency_state import ConcurrencyState
from rafcon.statemachine.enums import StateExecutionState
from rafcon.statemachine.enums import MethodName
from rafcon.statemachine.execution.execution_history import CallItem, ReturnItem, ConcurrencyItem
from rafcon.utils import log
logger = log.get_logger(__name__)


class PreemptiveConcurrencyState(ConcurrencyState):
    """ The preemptive concurrency state has a set of substates which are started when the preemptive concurrency state
    executes. The execution of preemptive concurrency state waits for the first substate to return, preempts all other
    substates and finally returns self.

    """

    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                  transitions, data_flows, start_state_id, scoped_variables, v_checker)

    def run(self):
        """ This defines the sequence of actions that are taken when the preemptive concurrency state is executed

        :return:
        """
        logger.debug("Starting execution of {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))
        self.setup_run()

        try:

            concurrency_history_item = self.setup_forward_or_backward_execution()

            #######################################################
            # start child threads
            #######################################################

            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size

            for index, state in enumerate(self.states.itervalues()):

                if not self.backward_execution:
                    # care for the history items; this item is only for execution visualization
                    concurrency_history_item.execution_histories[index].add_call_history_item(
                        state, MethodName.EXECUTE, self)
                else:  # backward execution
                    last_history_item = concurrency_history_item.execution_histories[index].pop_last_item()
                    assert isinstance(last_history_item, ReturnItem)

                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = index

                state_input = self.get_inputs_for_state(state)
                state_output = self.create_output_dictionary_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start(concurrency_history_item.execution_histories[index], self.backward_execution)

            #######################################################
            # wait for the first threads to finish
            #######################################################

            finished_thread_id = concurrency_queue.get()
            self.states[finished_thread_id].join()

            # preempt all child states
            for state_id, state in self.states.iteritems():
                state.recursively_preempt_states()

            # join all states
            for history_index, state in enumerate(self.states.itervalues()):
                state.join()
                state.state_execution_status = StateExecutionState.INACTIVE
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)

                # care for the history items
                if not self.backward_execution:
                    state.concurrency_queue = None
                    # add the data of all child states to the scoped data and the scoped variables
                    state.execution_history.add_return_history_item(state, MethodName.EXECUTE, self)
                else:
                    last_history_item = concurrency_history_item.execution_histories[history_index].pop_last_item()
                    assert isinstance(last_history_item, CallItem)

            #######################################################
            # handle backward execution case
            #######################################################

            if self.states[finished_thread_id].backward_execution:
                return self.finalize_backward_execution()

            else:
                self.backward_execution = False

            #######################################################
            # handle no transition
            #######################################################

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

            #######################################################
            # finalize
            #######################################################

            self.execution_history.add_return_history_item(self, MethodName.CALL_CONTAINER_STATE, self)
            self.write_output_data()
            self.check_output_data_type()
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.preempted:
                outcome = Outcome(-2, "preempted")
            return self.finalize(outcome)

        except Exception, e:
            logger.error("{0} had an internal error: {1}\n{2}".format(self, str(e), str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

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
