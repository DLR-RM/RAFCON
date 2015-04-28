"""
.. module:: concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

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