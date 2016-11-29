"""
.. module:: state_machine_status
   :platform: Unix, Windows
   :synopsis: A module to represent the state machine status

.. moduleauthor:: Sebastian Brunner


"""
from enum import Enum
from gtkmvc import Observable
from threading import Condition

from execution_history import ExecutionHistory
from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionStatus(Observable):
    """A class for representing the state machine status

    It inherits from Observable to make a change of its fields observable.

    :ivar execution_mode: the execution mode of the state machine
                        (i.e. running, paused, stopped, stepping)

    """

    def __init__(self, execution_mode=None):

        Observable.__init__(self)

        # these fields are not supposed to be written by the GUI directly, but via the methods of the
        # StateMachineExecutionEngine class
        self._execution_mode = None
        self.execution_mode = execution_mode
        logger.debug("State machine status is set to %s" % str(execution_mode))
        self.execution_condition_variable = Condition()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def execution_mode(self):
        """Property for the _execution_mode field

        """
        return self._execution_mode

    @execution_mode.setter
    @Observable.observed
    def execution_mode(self, execution_mode):
        if execution_mode is not None:
            if not isinstance(execution_mode, StateMachineExecutionStatus):
                raise TypeError("execution_mode must be of type StateMachineExecutionStatus")

        self._execution_mode = execution_mode


StateMachineExecutionStatus = Enum('STATE_MACHINE_EXECUTION_STATUS', 'STARTED STOPPED PAUSED '
                                                                     'FORWARD_INTO FORWARD_OVER FORWARD_OUT '
                                                                     'BACKWARD RUN_TO_SELECTED_STATE')