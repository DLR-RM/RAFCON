# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: preemptive_concurrency_state
   :synopsis: A module to represent a preemptive concurrency state for the state machine

"""

from builtins import str

from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.states.concurrency_state import ConcurrencyState
from rafcon.core.states.state import StateExecutionStatus
from rafcon.utils import log
logger = log.get_logger(__name__)


class PreemptiveConcurrencyState(ConcurrencyState):
    """ The preemptive concurrency state has a set of substates which are started when the preemptive concurrency state
    executes. The execution of preemptive concurrency state waits for the first substate to return, preempts all other
    substates and finally returns self.

    """

    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None,
                 income=None, outcomes=None, states=None, transitions=None, data_flows=None, start_state_id=None,
                 scoped_variables=None, safe_init=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, income, outcomes, states,
                                  transitions, data_flows, start_state_id, scoped_variables, safe_init=safe_init)

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

            # preempt all child states
            if not self.backward_execution:
                for state_id, state in self.states.items():
                    state.recursively_preempt_states()
            # join all states
            for history_index, state in enumerate(self.states.values()):
                self.join_state(state, history_index, concurrency_history_item)
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)

            # add the data of the first state now to overwrite data of the preempted states
            self.add_state_execution_output_to_scoped_data(finisher_state.output_data, finisher_state)
            self.update_scoped_variables_with_output_dictionary(finisher_state.output_data, finisher_state)
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

        except Exception as e:
            logger.exception("{0} had an internal error:".format(self))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
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
