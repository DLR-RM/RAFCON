"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from execution_history import ExecutionHistory
from statemachine_status import StateMachineStatus, ExecutionMode
from utils import log
logger = log.get_logger(__name__)


class StatemachineExecutionEngine():

    """A class that cares for the execution of the statemachine

    :ivar _statemachine_status: holds the status of the statemachine
    :ivar _validity_checker: holds a class instance that assures the validity of the statemachine

    """

    def __init__(self):

        self._status = StateMachineStatus(ExecutionMode.RUNNING)
        # TODO: write validity checker of the statemachine; Should this include the validity checkers of the state
        self._validity_checker = None
        self.execution_history = ExecutionHistory()
        logger.debug("Statemachine execution engine initialized")

    #TODO: pause all external modules
    def pause(self):
        self._status.execution_mode = ExecutionMode.PAUSED

    def start(self):
        logger.debug("Start statemachine and notify all threads waiting ")
        self._status.execution_mode = ExecutionMode.RUNNING
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    #TODO: stop all external modules
    def stop(self):
        self._status.execution_mode = ExecutionMode.STOPPED

    def step_mode(self):
        logger.debug("Activate step mode")
        self._status.execution_mode = ExecutionMode.STEPPING

    def backward_step_mode(self):
        self._status.execution_mode = ExecutionMode.BACKWARD_STEPPING

    def step(self):
        logger.debug("Notify all threads waiting for the the execution condition variable")
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    # depending on the execution state wait for the execution condition variable to be notified
    # list all execution modes to keep the overview
    def handle_execution_mode(self):
        if self._status.execution_mode is ExecutionMode.RUNNING:
            return

        elif self._status.execution_mode is ExecutionMode.STOPPED:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

        elif self._status.execution_mode is ExecutionMode.PAUSED:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

        elif self._status.execution_mode is ExecutionMode.STEPPING:
            logger.debug("Stepping mode: wait for next step")
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

        elif self._status.execution_mode is ExecutionMode.BACKWARD_STEPPING:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

    def start_from_selected_state(self, state):
        self._create_dependency_tree(state)
        self._start_tree()

    def _create_dependency_tree(self, state):
        #TODO: implement
        pass

    def _start_tree(self):
        #TODO: implement
        pass