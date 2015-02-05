"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
from gtkmvc import ModelMT
from execution_history import ExecutionHistory
from statemachine_status import StateMachineStatus, ExecutionMode
from statemachine.state_machine_manager import StateMachineManager
from utils import log
logger = log.get_logger(__name__)


class StatemachineExecutionEngine(ModelMT, Observable):

    """A class that cares for the execution of the statemachine

    :ivar _statemachine_status: holds the status of the statemachine
    :ivar _validity_checker: holds a class instance that assures the validity of the statemachine

    """

    execution_engine = None

    __observables__ = ("execution_engine",)

    def __init__(self, state_machine_manager):

        ModelMT.__init__(self)
        Observable.__init__(self)
        self.state_machine_manager = state_machine_manager
        self.execution_engine = self
        self._status = None
        self.status = StateMachineStatus(ExecutionMode.STOPPED)
        # TODO: write validity checker of the statemachine; Should this include the validity checkers of the state
        self._validity_checker = None
        self.execution_history = ExecutionHistory()
        logger.debug("Statemachine execution engine initialized")

    #TODO: pause all external modules
    @Observable.observed
    def pause(self):
        self._status.execution_mode = ExecutionMode.PAUSED

    @Observable.observed
    def start(self):
        if self._status.execution_mode is ExecutionMode.PAUSED:
            logger.debug("Start statemachine and notify all threads waiting ")
            self._status.execution_mode = ExecutionMode.RUNNING
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            logger.debug("Restart the state machine")
            self._status.execution_mode = ExecutionMode.RUNNING
            self.state_machine_manager.root_state.start()

    #TODO: stop all external modules
    @Observable.observed
    def stop(self):
        self._status.execution_mode = ExecutionMode.STOPPED

    @Observable.observed
    def step_mode(self):
        logger.debug("Activate step mode")
        self._status.execution_mode = ExecutionMode.STEPPING

    @Observable.observed
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
            # try:
            #     self._status.execution_condition_variable.acquire()
            #     self._status.execution_condition_variable.wait()
            # finally:
            #     self._status.execution_condition_variable.release()
            quit()

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

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def status(self):
        """Property for the _status field

        """
        return self._status

    @status.setter
    @Observable.observed
    def status(self, status):
        if not isinstance(status, StateMachineStatus):
            raise TypeError("status must be of type StateMachineStatus")
        self._status = status