"""
.. module:: statemachine_execution_engine
   :platform: Unix, Windows
   :synopsis: A module that cares for the execution of the statemachine

.. moduleauthor:: Sebastian Brunner


"""
import copy

import Queue

from gtkmvc import Observable
from gtkmvc import ModelMT
from rafcon.statemachine.execution.statemachine_status import StateMachineStatus
from rafcon.utils import log

logger = log.get_logger(__name__)
from rafcon.statemachine.enums import StateMachineExecutionStatus


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
        self.start_state_paths = []

        self.run_to_states = []

    # TODO: pause all external modules
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
            self.run_to_states = []
        else:
            logger.debug("Start the state machine")
            self._status.execution_mode = StateMachineExecutionStatus.STARTED
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

            self.state_machine_manager.state_machines[self.state_machine_manager.active_state_machine_id].start()
            self._execution_started = True

    # TODO: stop all external modules
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
        self.run_to_states = []

    @Observable.observed
    def step_mode(self):
        """Set the execution mode to stepping mode. Transitions are only triggered if a new step is triggered
        """
        logger.debug("Activate step mode")
        if self._execution_started:
            self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
        else:
            self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
            self.state_machine_manager.state_machines[self.state_machine_manager.active_state_machine_id].start()
            self._execution_started = True
        self.run_to_states = []

    def backward_step(self):
        """Take a backward step for all active states in the state machine
        """
        logger.debug("Notify all threads waiting for the the execution condition variable")
        self._status.execution_mode = StateMachineExecutionStatus.BACKWARD
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()
        self.run_to_states = []

    def step_into(self):
        """Take a forward step (into) for all active states in the state machine
        """
        logger.debug("Triggered step into ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_INTO
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()
        self.run_to_states = []

    def step_over(self):
        """Take a forward step (over) for all active states in the state machine
        """
        logger.debug("Triggered step over ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_OVER
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()
        self.run_to_states = []

    def step_out(self):
        """Take a forward step (out) for all active states in the state machine
        """
        logger.debug("Triggered step out ...")
        self._status.execution_mode = StateMachineExecutionStatus.FORWARD_OUT
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()
        self.run_to_states = []

    # depending on the execution state wait for the execution condition variable to be notified
    # list all execution modes to keep the overview
    def handle_execution_mode(self, state):
        """Checks the current execution status and returns it. If the execution mode is some kind of stepping,
        a condition variable stops the current execution, until it gets notified by the step*() or backward_step()
        functions. This function is called by the hierarchy states.

        :param state: the state that as for the execution mode is only passed for debugging reasons

        :return: the current state machine execution status

        """

        if self._status.execution_mode is StateMachineExecutionStatus.STARTED:
            logger.debug("Execution engine started!")

        elif self._status.execution_mode is StateMachineExecutionStatus.STOPPED:
            logger.debug("Execution engine stopped. State %s is going to quit!", state.name)

        elif self._status.execution_mode is StateMachineExecutionStatus.PAUSED:
            try:
                self._status.execution_condition_variable.acquire()
                self._status.execution_condition_variable.wait()
            finally:
                self._status.execution_condition_variable.release()

        else:  # all other step modes
            logger.debug("Stepping mode: wait for next step!")

            wait = True
            for state_path in copy.deepcopy(self.run_to_states):
                if state_path in state.get_path():
                    if state_path is state.get_path():
                        # the execution did a whole step_over for the hierarchy state "state" thus we delete its
                        # state path from self.run_to_states and wait for another step (of maybe different kind)
                        wait = True
                        self.run_to_states.remove(state_path)
                        break
                    else:
                        # the state_path is the path of an ancestor of "state", thus we must not wait
                        # as a step_over was triggered for the ancestor
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




        # in the case when the stop method wakes up the paused or step mode StateMachineExecutionStatus.STOPPED
        # will be returned
        return_value = self._status.execution_mode

        return return_value

    @staticmethod
    def execute_state_machine_from_path(path, start_state_path=None, wait_for_execution_finished=True):
        """
        A helper function to start an arbitrary state machine at a given path.
        :param path: The path where the state machine resides
        :param start_state_path: The path to the state from which the execution will start
        :return: a reference to the created state machine
        """

        import rafcon.statemachine.singleton
        rafcon.statemachine.singleton.library_manager.initialize()
        [state_machine, version, creation_time] = rafcon.statemachine.singleton. \
            global_storage.load_statemachine_from_path(path)
        rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
        rafcon.statemachine.singleton.state_machine_execution_engine.start(start_state_path=start_state_path)
        sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()

        concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size
        sm.root_state.concurrency_queue = concurrency_queue

        if wait_for_execution_finished:
            sm.root_state.join()
            rafcon.statemachine.singleton.state_machine_execution_engine.stop()
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
