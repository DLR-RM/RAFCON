"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import traceback

from rafcon.statemachine.state_elements.outcome import Outcome
from rafcon.statemachine.states.concurrency_state import ConcurrencyState
from rafcon.statemachine.enums import StateExecutionState
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
            concurrency_queue = self.start_child_states(concurrency_history_item)

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
                self.join_state_and_process_its_data(state, history_index, concurrency_history_item)

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

            return self.finalize_concurrency_state(outcome)

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
