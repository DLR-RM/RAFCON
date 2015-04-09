"""
.. module:: state_machine_status
   :platform: Unix, Windows
   :synopsis: A module to represent the state machine status

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
from enum import Enum
from threading import Condition

from execution_history import ExecutionHistory
from awesome_tool.utils import log
logger = log.get_logger(__name__)


ExecutionMode = Enum('EXECUTION_MODE', 'PAUSED RUNNING STOPPED STEPPING')


class StateMachineStatus(Observable):

    """A class for representing the state machine status

    It inherits from Observable to make a change of its fields observable.

    :ivar execution_mode: the execution mode of the state machine
                        (i.e. running, paused, stopped, stepping)
    :ivar _dependency_tree: when starting the state machine from an arbitrary child state, a dependency tree is
                            calculated and stored here
    :ivar _thread_histories: set of threads that are currently executing inside the state machine

    """

    def __init__(self, execution_mode=None):

        Observable.__init__(self)

        # these fields are not supposed to be written by the GUI directly, but via the methods of the
        # StateMachineExecutionEngine class
        self.execution_mode = execution_mode
        logger.debug("Statemachine status is set to %s" % str(execution_mode))
        self.execution_condition_variable = Condition()
        self._dependency_tree = None
        self._thread_histories = []

    def set_thread_history(self, history_id, execution_history):
        """ Sets the execution history for a certain history_id

        :param history_id: the id of the history
        :param execution_history: the execution history of a certain thread
        :return:
        """
        if not isinstance(execution_history, ExecutionHistory):
            raise TypeError("execution_history must be of type ExecutionHistory")
        self._thread_histories[history_id] = execution_history

    def set_execution_mode(self, execution_mode):
        """ Sets the current execution mode

        :param execution_mode: the new execution mode to set
        :return:
        """
        self.execution_mode = execution_mode


#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def dependency_tree(self):
        """Property for the _dependency_tree field

        """
        return self._dependency_tree

    @property
    def thread_histories(self):
        """Property for the _thread_histories field

        """
        return self._thread_histories
