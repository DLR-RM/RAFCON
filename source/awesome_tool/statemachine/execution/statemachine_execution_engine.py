"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""
import signal

from gtkmvc import Observable
from gtkmvc import ModelMT
from awesome_tool.statemachine.execution.execution_history import ExecutionHistory
from awesome_tool.statemachine.execution.statemachine_status import StateMachineStatus
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.enums import StateMachineExecutionStatus


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
        self.status = StateMachineStatus(StateMachineExecutionStatus.STOPPED)
        # TODO: write validity checker of the statemachine
        self._validity_checker = None
        logger.debug("Statemachine execution engine initialized")
        self._execution_started = False
        self._forward_step = True
        self.start_state_paths = []

    #TODO: pause all external modules
    @Observable.observed
    def pause(self):
        """Set the execution mode to paused
        """
        self._status.execution_mode = StateMachineExecutionStatus.PAUSED

    @Observable.observed
    def start(self, state_machine_id=None, start_state_path=None):
        """
        Start a specific state machine. If no state machine is provided the currently active state machine is started.
        :param state_machine_id: The id if the state machine to be started
        :param start_state_path: The path of the state in the state machine, from which the execution will start
        :return:
        """
        if self._execution_started:
            logger.debug("Resume state machine and notify all threads waiting ")
            self._status.execution_mode = StateMachineExecutionStatus.STARTED
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            logger.debug("Start the state machine")
            self._status.execution_mode = StateMachineExecutionStatus.STARTED
            if state_machine_id is not None:
                self.state_machine_manager.active_state_machine_id = state_machine_id

            if start_state_path:
                path_list = start_state_path.split("/")
                cur_path = ""
                for path in path_list:
                    if cur_path == "":
                        cur_path = curr_path = path
                    else:
                        cur_path = cur_path + "/" + path
                    self.start_state_paths.append(cur_path)

            self.state_machine_manager.state_machines[self.state_machine_manager.active_state_machine_id].start()
            self._execution_started = True

    #TODO: stop all external modules
    @Observable.observed
    def stop(self):
        """Set the execution mode to stopped
        """
        logger.debug("Stop execution ...")
        self._status.execution_mode = StateMachineExecutionStatus.STOPPED
        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_preempt_states()
        self._execution_started = False
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    @Observable.observed
    def step_mode(self):
        """Set the execution mode to stepping mode. Transitions are only triggered if a new step is triggered
        """
        logger.debug("Activate step mode")
        if self._execution_started:
            self._status.execution_mode = StateMachineExecutionStatus.STEP
        else:
            self._status.execution_mode = StateMachineExecutionStatus.STEP
            self.state_machine_manager.state_machines[self.state_machine_manager.active_state_machine_id].start()
            self._execution_started = True

    def backward_step(self):
        """Take a backward step for all active states in the state machine
        """
        logger.debug("Notify all threads waiting for the the execution condition variable")
        self._forward_step = False
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def step(self):
        """Take a forward step for all active states in the state machine
        """
        logger.debug("Triggered step ...")
        self._forward_step = True
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
        return_value = StateMachineExecutionStatus.STARTED
        if self._status.execution_mode is StateMachineExecutionStatus.STARTED:
            return_value = StateMachineExecutionStatus.STARTED

        elif self._status.execution_mode is StateMachineExecutionStatus.STOPPED:
            # try:
            #     self._status.execution_condition_variable.acquire()
            #     self._status.execution_condition_variable.wait()
            # finally:
            #     self._status.execution_condition_variable.release()
            logger.debug("Execution engine stopped. State %s is going to quit!", state.name)
            return_value = StateMachineExecutionStatus.STOPPED

        elif self._status.execution_mode is StateMachineExecutionStatus.PAUSED:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

            if self._status.execution_mode is StateMachineExecutionStatus.STARTED:
                return_value = StateMachineExecutionStatus.STARTED
            elif self._forward_step:
                return_value = StateMachineExecutionStatus.STEP
            else:
                return_value = StateMachineExecutionStatus.BACKWARD_STEP

        elif self._status.execution_mode is StateMachineExecutionStatus.STEP:
            logger.debug("Stepping mode: wait for next step")
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

            if self._forward_step:
                return_value = StateMachineExecutionStatus.STEP
            else:
                return_value = StateMachineExecutionStatus.BACKWARD_STEP

        # this is the case when the stop method wakes up the paused or step mode
        if self._status.execution_mode is StateMachineExecutionStatus.STOPPED:
            return StateMachineExecutionStatus.STOPPED
        return return_value

    def start_from_selected_state(self, state):
        """Starts the active statemachine from a given state

        :param state: the state the execution should start at
        """
        self._create_dependency_tree(state)
        self._start_tree()

    @staticmethod
    def execute_state_machine_from_path(path, start_state_path=None):
        """
        A helper function to start an arbitrary state machine at a given path.
        :param path: The path where the state machine resides
        :param start_state_path: The path to the state from which the execution will start
        :return: a reference to the created state machine
        """

        import awesome_tool.statemachine.singleton
        awesome_tool.statemachine.singleton.state_machine_manager.delete_all_state_machines()
        signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)

        awesome_tool.statemachine.singleton.library_manager.initialize()
        [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
            global_storage.load_statemachine_from_yaml(path)
        awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
        awesome_tool.statemachine.singleton.state_machine_execution_engine.start(start_state_path=start_state_path)
        sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
        if sm:
            sm.root_state.join()
        awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

        return sm

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