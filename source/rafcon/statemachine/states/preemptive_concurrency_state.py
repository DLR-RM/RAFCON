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
            finisher_state = self.states[finished_thread_id]
            finisher_state.join()
            self.add_state_execution_output_to_scoped_data(finisher_state.output_data, finisher_state)
            self.update_scoped_variables_with_output_dictionary(finisher_state.output_data, finisher_state)
            # preempt all child states
            if not self.backward_execution:
                for state_id, state in self.states.iteritems():
                    state.recursively_preempt_states()
            # join all states
            for history_index, state in enumerate(self.states.itervalues()):
                self.join_state(state, history_index, concurrency_history_item)

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
                # final outcome is set here
                transition = self.handle_no_transition(self.states[finished_thread_id])
            # it the transition is still None, then the state was preempted or aborted, in this case return
            if transition is None:
                self.output_data["error"] = RuntimeError("state aborted")
            else:
                if 'error' in self.states[finished_thread_id].output_data:
                    self.output_data["error"] = self.states[finished_thread_id].output_data['error']
                self.final_outcome = self.outcomes[transition.to_outcome]

            return self.finalize_concurrency_state(self.final_outcome)

        except Exception, e:
            logger.error("{0} had an internal error: {1}\n{2}".format(self, str(e), str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    def _check_transition_validity(self, check_transition):
        """ Transition of BarrierConcurrencyStates must least fulfill the condition of a ContainerState.
        Start transitions are forbidden in the ConcurrencyState

        :param check_transition: the transition to check for validity
        :return:
        """
        valid, message = super(PreemptiveConcurrencyState, self)._check_transition_validity(check_transition)
        if not valid:
            return False, message

        # Only transitions to the parent state are allowed

        if check_transition.to_state != self.state_id:
            return False, "Only transitions to the parent state are allowed"

        return True, message
