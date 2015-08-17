"""
.. module:: concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
from gtkmvc import Observable

from awesome_tool.statemachine.states.container_state import ContainerState


class ConcurrencyState(ContainerState):

    """A class to represent a concurrency state for the state machine

    The concurrency state holds several child states, that can be container states again
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, states, transitions,
                                data_flows, start_state_id, scoped_variables, v_checker, path, filename,
                                check_path=check_path)

    def run(self, *args, **kwargs):
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def _check_start_transition(self, start_transition):
        return False, "No start transitions are allowed in concurrency state"

    # @Observable.observed
    # def add_transition(self, from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id=None):
    #     """Adds a transition to the container state
    #
    #     Note: Either the toState or the toOutcome needs to be "None"
    #
    #     :param from_state_id: The source state of the transition
    #     :param from_outcome: The outcome of the source state to connect the transition to
    #     :param to_state_id: The target state of the transition
    #     :param to_outcome: The target outcome of a container state
    #     :param transition_id: An optional transition id for the new transition
    #     """
    #
    #     transition_id = self.check_transition_id(transition_id)
    #
    #     self.basic_transition_checks(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
    #
    #     self.check_if_outcome_already_connected(from_state_id, from_outcome)
    #
    #     # in concurrency states only transitions to the parents are allowed
    #     if to_state_id is not None and to_state_id is not self.state_id:  # None means that the target state is the containing state
    #         raise AttributeError("In concurrency states the to_state must be the container state itself")
    #
    #     self.create_transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
    #
    #     return transition_id