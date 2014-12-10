"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from execution_history import ExecutionHistory
from statemachine_status import StateMachineStatus


class StatemachineExecutionEngine():

    """A class that cares for the execution of the statemachine

    :ivar _statemachine_status: holds the status of the statemachine
    :ivar _root_Thread: holds the root thread of the statemachine execution (i.e. the start state)
    :ivar _validity_checker: holds a class instance that assures the validity of the statemachine

    """

    def __init__(self, statemachine_status=None, root_thread=None, validity_checker=None):

        self._statemachine_status = statemachine_status
        self._root_Thread = root_thread
        self._validity_checker = validity_checker

    def start(self):
        #TODO: implement
        pass

    def stop(self):
        #TODO: implement
        pass

    def pause(self):
        #TODO: implement
        pass

    def toggle_step_mode(self):
        #TODO: implement
        pass

    def step(self):
        #TODO: implement
        pass

    def step_backward(self):
        #TODO: implement
        pass

    def start_from_selected_state(self, state):
        self._create_dependency_tree(state)
        self._start_tree()

    def _create_dependency_tree(self, state):
        #TODO: implement
        pass

    def _start_tree(self):
        #TODO: implement
        pass