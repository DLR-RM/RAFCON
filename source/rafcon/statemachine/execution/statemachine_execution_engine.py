"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""
import copy
import threading
import time
import Queue
from threading import Lock

from gtkmvc import Observable
from rafcon.statemachine.execution.statemachine_status import StateMachineStatus
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.utils import log

logger = log.get_logger(__name__)


class StatemachineExecutionEngine(Observable):
    """A class that cares for the execution of the statemachine

    :ivar state_machine_manager: holds the state machine manager of all states that can be executed
    :ivar status: holds the current execution status of the state machine
    :ivar _validity_checker: holds a class instance that assures the validity of the state machine
    :ivar execution_history: the history of the execution TODO: should be an list

    """

    execution_engine = None

    __wait_for_finishing_thread = None
    __running_state_machine = None

    __observables__ = ("execution_engine",)

    def __init__(self, state_machine_manager):
        Observable.__init__(self)
        self.state_machine_manager = state_machine_manager
        self.execution_engine = self
        self._status = None
        self.status = StateMachineStatus(StateMachineExecutionStatus.STOPPED)
        # TODO: write validity checker of the statemachine
        self._validity_checker = None
        logger.debug("Statemachine execution engine initialized")
        self.start_state_paths = []

        self.execution_engine_lock = Lock()
        self._run_to_states = []
        self.run_to_states = []
        self.state_machine_running = False

    @Observable.observed
    def pause(self):
        """Set the execution mode to paused
        """

        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_pause_states()

        logger.debug("Pause execution ...")
        self._status.execution_mode = StateMachineExecutionStatus.PAUSED


    @Observable.observed
    def start(self, state_machine_id=None, start_state_path=None):
        """
        Start a specific state machine. If no state machine is provided the currently active state machine is started.
        :param state_machine_id: The id if the state machine to be started
        :param start_state_path: The path of the state in the state machine, from which the execution will start
        :return:
        """

        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_resume_states()

        if self._status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            self._status.execution_mode = StateMachineExecutionStatus.STARTED
            logger.debug("Resume execution engine ...")
            self.run_to_states = []
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            # do not start another state machine before the old one did not finish its execution
            while self.state_machine_running:
                time.sleep(1.0)
            self._status.execution_mode = StateMachineExecutionStatus.STARTED
            logger.debug("Start execution engine ...")
            if state_machine_id is not None:
                self.state_machine_manager.active_state_machine_id = state_machine_id

            self.start_state_paths = []

            if start_state_path:
                path_list = start_state_path.split("/")
                cur_path = ""
                for path in path_list:
                    if cur_path == "":
                        cur_path = path
                    else:
                        cur_path = cur_path + "/" + path
                    self.start_state_paths.append(cur_path)

            self._run_active_state_machine()

    @Observable.observed
    def stop(self):
        """Set the execution mode to stopped
        """
        logger.debug("Stop the state machine execution ...")
        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_preempt_states()
        self.set_execution_mode_to_stopped()

        # Notifies states waiting in step mode or those that are paused about execution stop
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def join(self):
        self.__wait_for_finishing_thread.join()

    @Observable.observed
    def set_execution_mode_to_stopped(self):
        """Stop and reset execution engine"""
        self._status.execution_mode = StateMachineExecutionStatus.STOPPED
        self.run_to_states = []

    @Observable.observed
    def step_mode(self):
        """Set the execution mode to stepping mode. Transitions are only triggered if a new step is triggered
        """
        logger.debug("Activate step mode")
        if self._status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
            self._run_active_state_machine()
        self.run_to_states = []

    def _run_active_state_machine(self):
        """Store running state machine and observe its status"""
        self.__running_state_machine = self.state_machine_manager.state_machines[
            self.state_machine_manager.active_state_machine_id]

        self.__running_state_machine.start()

        self.__wait_for_finishing_thread = threading.Thread(target=self._wait_for_finishing)
        self.__wait_for_finishing_thread.start()

    def _wait_for_finishing(self):
        """Observe running state machine and stop engine if execution has finished"""
        self.state_machine_running = True
        self.__running_state_machine.join()
        self.set_execution_mode_to_stopped()
        self.state_machine_running = False

    def backward_step(self):
        """Take a backward step for all active states in the state machine
        """
        logger.debug("Executing backward step ...")
        self._status.execution_mode = StateMachineExecutionStatus.BACKWARD
        self.run_to_states = []
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def step_into(self):
        """Take a forward step (into) for all active states in the state machine
        """
        logger.debug("Execution step into ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
        self.run_to_states = []
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def step_over(self):
        """Take a forward step (over) for all active states in the state machine
        """
        logger.debug("Execution step over ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_OVER
        self.run_to_states = []
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def step_out(self):
        """Take a forward step (out) for all active states in the state machine
        """
        logger.debug("Execution step out ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_OUT
        self.run_to_states = []
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()

    def run_to_selected_state(self, path, state_machine_id=None):
        """Take a forward step (out) for all active states in the state machine
        """

        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_resume_states()

        if self._status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            logger.debug("Resume execution engine and run to selected state!")
            self.run_to_states = []
            self.run_to_states.append(path)
            self._status.execution_mode = StateMachineExecutionStatus.RUN_TO_SELECTED_STATE
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()
        else:
            logger.debug("Start execution engine and run to selected state!")
            self._status.execution_mode = StateMachineExecutionStatus.RUN_TO_SELECTED_STATE
            if state_machine_id is not None:
                self.state_machine_manager.active_state_machine_id = state_machine_id
            self.run_to_states = []
            self.run_to_states.append(path)
            self._run_active_state_machine()

    # depending on the execution state wait for the execution condition variable to be notified
    # list all execution modes to keep the overview
    def handle_execution_mode(self, state, next_child_state_to_execute=None):
        """Checks the current execution status and returns it

        If the execution mode is some kind of stepping, a condition variable stops the current execution,
        until it gets notified by the step_*() or backward_step()
        functions. This function is called by the hierarchy states.

        :param state: the state that as for the execution mode is only passed for debugging reasons
        :return: the current state machine execution status
        """

        if self._status.execution_mode is StateMachineExecutionStatus.STARTED:
            # logger.debug("Execution engine started!")
            pass

        elif self._status.execution_mode is StateMachineExecutionStatus.STOPPED:
            logger.debug("Execution engine stopped. State '{0}' is going to quit in the case of "
                         "no preemption handling has to be done!".format(state.name))

        elif self._status.execution_mode is StateMachineExecutionStatus.PAUSED:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

        else:  # all other step modes
            logger.debug("Stepping mode: waiting for next step!")

            wait = True
            for state_path in copy.deepcopy(self.run_to_states):
                if state_path == state.get_path():
                    # the execution did a whole step_over for the hierarchy state "state"
                    # or a whole step_out for the hierarchy state "state"
                    # thus we delete its state path from self.run_to_states
                    # and wait for another step (of maybe different kind)
                    wait = True
                    self.run_to_states.remove(state_path)
                    break
                elif state_path == next_child_state_to_execute.get_path():
                    # this is the case that execution has reached a specifc state explicitely marked via
                    # run_to_selected_state(); if this is the case run_to_selected_state() is finished and the execution
                    # has to wait for new execution commands
                    wait = True
                    self.run_to_states.remove(state_path)
                    break
                # if there is an element in self.run_to_states which is not the same as state.get_path(),
                # in this case the execution does not have to wait and has to run until the selected state is reached
                else:
                    wait = False
                    break

            if wait:
                try:
                    self._status.execution_condition_variable.acquire()
                    self._status.execution_condition_variable.wait()
                finally:
                    self._status.execution_condition_variable.release()

            # calculate states to which should be run
            if self._status.execution_mode is StateMachineExecutionStatus.BACKWARD:
                pass
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_INTO:
                pass
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OVER:
                # the state that called this method is a hierarchy state => thus we save this state and wait until this
                # very state will execute its next state; only then we will wait on the condition variable
                self.run_to_states.append(state.get_path())
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OUT:
                from rafcon.statemachine.states.state import State
                if isinstance(state.parent, State):
                    parent_path = state.parent.get_path()
                    self.run_to_states.append(parent_path)
                else:
                    # this is the case if step_out is called from the highest level
                    # this is handled in the same way as the FORWARD_OVER
                    self.run_to_states.append(state.get_path())
            elif self._status.execution_mode is StateMachineExecutionStatus.RUN_TO_SELECTED_STATE:
                # "Run to states were already updated thus doing nothing"
                pass

        # in the case when the stop method wakes up the paused or step mode StateMachineExecutionStatus.STOPPED
        # will be returned
        return_value = self._status.execution_mode

        return return_value

    def notify_run_to_states(self, state):
        """
        This is a very special case. Inside a hierarchy state a step_over is triggered but the step_over affects the
        last child. In this case the step_over must be transformed to a step_out and thus modify the self.run_to_states
        :param state:
        :return:
        """
        if self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OVER:
            step_over_to_step_out_transform_found = False
            for state_path in copy.deepcopy(self.run_to_states):
                if state_path == state.get_path():
                    self.run_to_states.remove(state_path)
                    step_over_to_step_out_transform_found = True
                    logger.debug("Step_over is transformed to a step out for state %s!", state.name)
            if step_over_to_step_out_transform_found:
                from rafcon.statemachine.states.state import State
                if isinstance(state.parent, State):
                    parent_path = state.parent.get_path()
                    self.run_to_states.append(parent_path)

    @staticmethod
    def execute_state_machine_from_path(path, start_state_path=None, wait_for_execution_finished=True):
        """
        A helper function to start an arbitrary state machine at a given path.
        :param path: The path where the state machine resides
        :param start_state_path: The path to the state from which the execution will start
        :return: a reference to the created state machine
        """

        import rafcon.statemachine.singleton
        from rafcon.statemachine.storage import storage
        rafcon.statemachine.singleton.library_manager.initialize()
        state_machine = storage.load_statemachine_from_path(path)
        rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
        rafcon.statemachine.singleton.state_machine_execution_engine.start(start_state_path=start_state_path)
        sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()

        concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size
        sm.root_state.concurrency_queue = concurrency_queue

        if wait_for_execution_finished:
            sm.root_state.join()
            rafcon.statemachine.singleton.state_machine_execution_engine.stop()
        return sm

    @Observable.observed
    def set_execution_mode(self, execution_mode):
        """
        An observed setter for the execution mode of the state machine status. This is necessary for the
        monitoring client to update the local state machine in the same way as the root state machine of the server.
        :param execution_mode: the new execution mode of the state machine
        :return:
        """
        if not isinstance(execution_mode, StateMachineExecutionStatus):
            raise TypeError("status must be of type StateMachineStatus")
        self._status.execution_mode = execution_mode

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

    @property
    def run_to_states(self):
        """Property for the _run_to_states field

        """
        self.execution_engine_lock.acquire()
        return_value = self._run_to_states
        self.execution_engine_lock.release()
        return return_value

    @run_to_states.setter
    def run_to_states(self, run_to_states):
        if not isinstance(run_to_states, list):
            raise TypeError("run_to_states must be of type list")
        self.execution_engine_lock.acquire()
        self._run_to_states = run_to_states
        self.execution_engine_lock.release()

