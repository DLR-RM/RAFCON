"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
from gtkmvc import ModelMT
from awesome_tool.statemachine.execution.execution_history import ExecutionHistory
from awesome_tool.statemachine.execution.statemachine_status import StateMachineStatus, ExecutionMode
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class StatemachineExecutionEngine(ModelMT, Observable):

    """A class that cares for the execution of the statemachine

    :ivar state_machine_manager: holds the state machine manager of all states that can be executed
    :ivar status: holds the current execution status of the state machine
    :ivar _validity_checker: holds a class instance that assures the validity of the state machine
    :ivar execution_history: the history of the execution TODO: should be an list

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
        # TODO: write validity checker of the statemachine
        self._validity_checker = None
        self.execution_history = ExecutionHistory()
        logger.debug("Statemachine execution engine initialized")
        self._execution_started = False

    #TODO: pause all external modules
    @Observable.observed
    def pause(self):
        """Set the execution mode to paused
        """
        self._status.execution_mode = ExecutionMode.PAUSED

    @Observable.observed
    def start(self, state_machine_id):
        """Set the execution mode to started
        """
        if self._execution_started:
            logger.debug("Start statemachine and notify all threads waiting ")
            self._status.execution_mode = ExecutionMode.RUNNING
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            logger.debug("Restart the state machine")
            self._status.execution_mode = ExecutionMode.RUNNING
            self.state_machine_manager.active_state_machine_id = state_machine_id
            self.state_machine_manager.state_machines[state_machine_id].root_state.start()
            self._execution_started = True

    #TODO: stop all external modules
    @Observable.observed
    def stop(self):
        """Set the execution mode to stopped
        """
        self._status.execution_mode = ExecutionMode.STOPPED
        self._execution_started = False

    @Observable.observed
    def step_mode(self):
        """Set the execution mode to stepping mode. Transitions are only triggered if a new step is triggered
        """
        logger.debug("Activate step mode")
        if self._execution_started:
            self._status.execution_mode = ExecutionMode.STEPPING
        else:
            self._status.execution_mode = ExecutionMode.STEPPING
            self.state_machine_manager.state_machines[self.state_machine_manager.active_state_machine_id].root_state.start()
            self._execution_started = True

    @Observable.observed
    def backward_step_mode(self):
        """Take a backward step for all active states in the state machine
        """
        self._status.execution_mode = ExecutionMode.BACKWARD_STEPPING

    def step(self):
        """Take a forward step for all active states in the state machine
        """
        logger.debug("Notify all threads waiting for the the execution condition variable")
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    # depending on the execution state wait for the execution condition variable to be notified
    # list all execution modes to keep the overview
    def handle_execution_mode(self, state):
        """Checks the current execution status and returns stop in the case. If the execution mode is "STEPPING",
        a condition variable stops the current execution, until it gets notified by the step() or backward_step()
        function.

        :param state: the state that as for the execution mode is only passed for debugging reasons

        :return: "stop" if the state machine must stop the execution, else "run"

        This functions is called by the hierarchy states.
        """
        if self._status.execution_mode is ExecutionMode.RUNNING:
            return

        elif self._status.execution_mode is ExecutionMode.STOPPED:
            # try:
            #     self._status.execution_condition_variable.acquire()
            #     self._status.execution_condition_variable.wait()
            # finally:
            #     self._status.execution_condition_variable.release()
            logger.debug("Execution engine stopped. State %s is going to quit!", state.name)
            return "stop"

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
        return "run"

    def start_from_selected_state(self, state):
        """Starts the active statemachine from a given state

        :param state: the state the execution should start at
        """
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