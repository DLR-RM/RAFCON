"""
.. module:: concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.states.container_state import ContainerState


class ConcurrencyState(ContainerState):

    """A class tto represent a concurrency state for the state machine

    The concurrency state holds several child states, that can be container states again
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, state_type=None):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, states, transitions,
                                data_flows, start_state, scoped_variables, v_checker, path, filename,
                                state_type=state_type)

    def run(self, *args, **kwargs):
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")