# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: execution_engine
   :synopsis: A module that cares for the execution of the state machine

"""
from future import standard_library
standard_library.install_aliases()
import copy
import threading
import time
import queue
from threading import Lock, RLock
import sys

from gtkmvc3.observable import Observable
from rafcon.core.execution.execution_status import ExecutionStatus
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.utils import log
from rafcon.utils import plugins

logger = log.get_logger(__name__)


class ExecutionEngine(Observable):
    """A class that cares for the execution of the state machine

    :ivar state_machine_manager: holds the state machine manager of all states that can be executed
    :ivar status: holds the current execution status of the state machine
    :ivar execution_history: the history of the execution TODO: should be an list

    """

    __wait_for_finishing_thread = None
    __running_state_machine = None

    def __init__(self, state_machine_manager):
        Observable.__init__(self)
        self.state_machine_manager = state_machine_manager
        self._status = ExecutionStatus(StateMachineExecutionStatus.STOPPED)
        logger.debug("State machine execution engine initialized")
        self.start_state_paths = []

        self.execution_engine_lock = Lock()
        self._run_to_states = []
        self.run_to_states = []
        self.state_machine_running = False
        self.synchronization_counter = 0
        self.synchronization_lock = Lock()

    @Observable.observed
    def pause(self):
        """Set the execution mode to paused
        """

        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_pause_states()

        logger.debug("Pause execution ...")
        self.set_execution_mode(StateMachineExecutionStatus.PAUSED)

    def finished_or_stopped(self):
        """ Condition check on finished or stopped status

        The method returns a value which is equivalent with not 'active' status of the current state machine.

        :return: outcome of condition check stopped or finished
        :rtype: bool
        """
        return (self._status.execution_mode is StateMachineExecutionStatus.STOPPED) or \
               (self._status.execution_mode is StateMachineExecutionStatus.FINISHED)

    @Observable.observed
    def start(self, state_machine_id=None, start_state_path=None):
        """ Start state machine

        If no state machine is running start a specific state machine.
        If no state machine is provided the currently active state machine is started.
        If there is already a state machine running, just resume it without taking the passed state_machine_id argument
        into account.

        :param state_machine_id: The id if the state machine to be started
        :param start_state_path: The path of the state in the state machine, from which the execution will start
        :return:
        """

        if not self.finished_or_stopped():
            logger.debug("Resume execution engine ...")
            self.run_to_states = []
            if self.state_machine_manager.get_active_state_machine() is not None:
                self.state_machine_manager.get_active_state_machine().root_state.recursively_resume_states()
                if isinstance(state_machine_id, int) and \
                        state_machine_id != self.state_machine_manager.get_active_state_machine().state_machine_id:
                    logger.info("Resumed state machine with id {0} but start of state machine id {1} was requested."
                                "".format(self.state_machine_manager.get_active_state_machine().state_machine_id,
                                          state_machine_id))
            self.set_execution_mode(StateMachineExecutionStatus.STARTED)
        else:
            # do not start another state machine before the old one did not finish its execution
            if self.state_machine_running:
                logger.warning("An old state machine is still running! Make sure that it terminates,"
                            " before you can start another state machine!")
                return

            logger.debug("Start execution engine ...")
            if state_machine_id is not None:
                self.state_machine_manager.active_state_machine_id = state_machine_id

            if not self.state_machine_manager.active_state_machine_id:
                logger.error("There exists no active state machine!")
                return

            self.set_execution_mode(StateMachineExecutionStatus.STARTED)

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
        self.__set_execution_mode_to_stopped()

        # Notifies states waiting in step mode or those that are paused about execution stop
        self._status.execution_condition_variable.acquire()
        self._status.execution_condition_variable.notify_all()
        self._status.execution_condition_variable.release()
        self.__running_state_machine = None

    def join(self, timeout=None):
        """Blocking wait for the execution to finish

        :param float timeout: Maximum time to wait or None for infinitely
        :return: True if the execution finished, False if no state machine was started or a timeout occurred
        :rtype: bool
        """
        if self.__wait_for_finishing_thread:
            if not timeout:
                # signal handlers won't work if timeout is None and the thread is joined
                while True:
                    self.__wait_for_finishing_thread.join(0.5)
                    if not self.__wait_for_finishing_thread.isAlive():
                        break
            else:
                self.__wait_for_finishing_thread.join(timeout)
            return not self.__wait_for_finishing_thread.is_alive()
        else:
            logger.warning("Cannot join as state machine was not started yet.")
            return False

    def __set_execution_mode_to_stopped(self):
        """Stop and reset execution engine"""
        self.run_to_states = []
        self.set_execution_mode(StateMachineExecutionStatus.STOPPED)

    def __set_execution_mode_to_finished(self):
        """Stop and reset execution engine"""
        self.run_to_states = []
        self.set_execution_mode(StateMachineExecutionStatus.FINISHED)

    @Observable.observed
    def step_mode(self, state_machine_id=None):
        """Set the execution mode to stepping mode. Transitions are only triggered if a new step is triggered
        """
        logger.debug("Activate step mode")

        if state_machine_id is not None:
            self.state_machine_manager.active_state_machine_id = state_machine_id

        self.run_to_states = []
        if self.finished_or_stopped():
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)
            self._run_active_state_machine()
        else:
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)

    def _run_active_state_machine(self):
        """Store running state machine and observe its status
        """

        # Create new concurrency queue for root state to be able to synchronize with the execution
        self.__running_state_machine = self.state_machine_manager.get_active_state_machine()
        if not self.__running_state_machine:
            logger.error("The running state machine must not be None")
        self.__running_state_machine.root_state.concurrency_queue = queue.Queue(maxsize=0)

        if self.__running_state_machine:
            self.__running_state_machine.start()

            self.__wait_for_finishing_thread = threading.Thread(target=self._wait_for_finishing)
            self.__wait_for_finishing_thread.start()
        else:
            logger.warning("Currently no active state machine! Please create a new state machine.")
            self.set_execution_mode(StateMachineExecutionStatus.STOPPED)

    def _wait_for_finishing(self):
        """Observe running state machine and stop engine if execution has finished"""
        self.state_machine_running = True
        self.__running_state_machine.join()
        self.__set_execution_mode_to_finished()
        self.state_machine_manager.active_state_machine_id = None
        plugins.run_on_state_machine_execution_finished()
        # self.__set_execution_mode_to_stopped()
        self.state_machine_running = False

    def backward_step(self):
        """Take a backward step for all active states in the state machine
        """
        logger.debug("Executing backward step ...")
        self.run_to_states = []
        self.set_execution_mode(StateMachineExecutionStatus.BACKWARD)

    def step_into(self):
        """Take a forward step (into) for all active states in the state machine
        """
        logger.debug("Execution step into ...")
        self.run_to_states = []
        if self.finished_or_stopped():
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)
            self._run_active_state_machine()
        else:
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)

    def step_over(self):
        """Take a forward step (over) for all active states in the state machine
        """
        logger.debug("Execution step over ...")
        self.run_to_states = []
        if self.finished_or_stopped():
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_OVER)
            self._run_active_state_machine()
        else:
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_OVER)

    def step_out(self):
        """Take a forward step (out) for all active states in the state machine
        """
        logger.debug("Execution step out ...")
        self.run_to_states = []
        if self.finished_or_stopped():
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_OUT)
            self._run_active_state_machine()
        else:
            self.set_execution_mode(StateMachineExecutionStatus.FORWARD_OUT)

    def run_to_selected_state(self, path, state_machine_id=None):
        """Execute the state machine until a specific state. This state won't be executed. This is an asynchronous task
        """

        if self.state_machine_manager.get_active_state_machine() is not None:
            self.state_machine_manager.get_active_state_machine().root_state.recursively_resume_states()

        if not self.finished_or_stopped():
            logger.debug("Resume execution engine and run to selected state!")
            self.run_to_states = []
            self.run_to_states.append(path)
            self.set_execution_mode(StateMachineExecutionStatus.RUN_TO_SELECTED_STATE)
        else:
            logger.debug("Start execution engine and run to selected state!")
            if state_machine_id is not None:
                self.state_machine_manager.active_state_machine_id = state_machine_id
            self.set_execution_mode(StateMachineExecutionStatus.RUN_TO_SELECTED_STATE)
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
        self.synchronization_lock.acquire()
        self.synchronization_counter += 1
        self.synchronization_lock.release()

        if self._status.execution_mode is StateMachineExecutionStatus.STARTED:
            # logger.debug("Execution engine started!")
            pass

        elif self._status.execution_mode is StateMachineExecutionStatus.STOPPED:
            logger.debug("Execution engine stopped. State '{0}' is going to quit in the case of "
                         "no preemption handling has to be done!".format(state.name))

        elif self._status.execution_mode is StateMachineExecutionStatus.PAUSED:
            while self._status.execution_mode is StateMachineExecutionStatus.PAUSED:
                try:
                    self._status.execution_condition_variable.acquire()
                    self._status.execution_condition_variable.wait()
                finally:
                    self._status.execution_condition_variable.release()

        elif self._status.execution_mode is StateMachineExecutionStatus.FINISHED:
            # this must never happen during execution of the execution engine
            raise Exception

        else:  # all other step modes
            logger.debug("Stepping mode: waiting for next step!")

            wait = True
            # if there is not state in self.run_to_states then RAFCON waits for the next user input and simply does
            # one step
            for state_path in copy.deepcopy(self.run_to_states):
                next_child_state_path = None
                # can be None in case of no transition given
                if next_child_state_to_execute:
                    next_child_state_path = next_child_state_to_execute.get_path()
                if state_path == state.get_path():
                    # the execution did a whole step_over for the hierarchy state "state"
                    # or a whole step_out for the hierarchy state "state"
                    # thus we delete its state path from self.run_to_states
                    # and wait for another step (of maybe different kind)
                    wait = True
                    self.run_to_states.remove(state_path)
                    break
                elif state_path == next_child_state_path:
                    # this is the case that execution has reached a specific state explicitly marked via
                    # run_to_selected_state(); if this is the case run_to_selected_state() is finished and the execution
                    # has to wait for new execution commands
                    wait = True
                    self.run_to_states.remove(state_path)
                    break
                # if there is an element in self.run_to_states which is not the same as state.get_path(),
                # in this case the execution does not have to wait and has to run until the selected state is reached
                else:
                    wait = False
                    # do not break here, the state_path may be of another state machine branch
                    # break

            if wait:
                try:
                    self._status.execution_condition_variable.acquire()
                    self._status.execution_condition_variable.wait()
                finally:
                    self._status.execution_condition_variable.release()
                # state was notified => thus, a new user command was issued, which has to be handled!
                state.execution_history.new_execution_command_handled = False

            # calculate states to which should be run
            if self._status.execution_mode is StateMachineExecutionStatus.BACKWARD:
                pass
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_INTO:
                pass
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OVER:
                # the state that called this method is a hierarchy state => thus we save this state and wait until this
                # very state will execute its next state; only then we will wait on the condition variable
                if not state.execution_history.new_execution_command_handled:
                    self.run_to_states.append(state.get_path())
                else:
                    pass
            elif self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OUT:
                from rafcon.core.states.state import State
                if isinstance(state.parent, State):
                    from rafcon.core.states.library_state import LibraryState
                    if isinstance(state.parent, LibraryState):
                        parent_path = state.parent.parent.get_path()
                    else:
                        parent_path = state.parent.get_path()
                    if not state.execution_history.new_execution_command_handled:
                        self.run_to_states.append(parent_path)
                    else:
                        pass
                else:
                    # if step_out is called from the highest level just run the state machine to the end
                    self.run_to_states = []
                    self.set_execution_mode(StateMachineExecutionStatus.STARTED)
            elif self._status.execution_mode is StateMachineExecutionStatus.RUN_TO_SELECTED_STATE:
                # "run_to_states" were already updated thus doing nothing
                pass

        state.execution_history.new_execution_command_handled = True

        # in the case that the stop method wakes up the paused or step mode a StateMachineExecutionStatus.STOPPED
        # will be returned
        return_value = self._status.execution_mode

        return return_value

    def modify_run_to_states(self, state):
        """
        This is a special case. Inside a hierarchy state a step_over is triggered but the step_over affects the
        last child. In this case the step_over must be transformed to a step_out and thus modify the self.run_to_states
        :param state:
        :return:
        """
        if self._status.execution_mode is StateMachineExecutionStatus.FORWARD_OVER or \
            self._status.execution_mode  is StateMachineExecutionStatus.FORWARD_OUT:
            step_over_to_step_out_transform_found = False
            for state_path in copy.deepcopy(self.run_to_states):
                if state_path == state.get_path():
                    self.run_to_states.remove(state_path)
                    step_over_to_step_out_transform_found = True
                    logger.debug("Step_over is transformed to a step out for state %s!", state.name)
            if step_over_to_step_out_transform_found:
                from rafcon.core.states.state import State
                if isinstance(state.parent, State):
                    from rafcon.core.states.library_state import LibraryState
                    if isinstance(state.parent, LibraryState):
                        parent_path = state.parent.parent.get_path()
                    else:
                        parent_path = state.parent.get_path()
                    self.run_to_states.append(parent_path)

    def execute_state_machine_from_path(self, state_machine=None, path=None, start_state_path=None, wait_for_execution_finished=True):
        """ A helper function to start an arbitrary state machine at a given path.

        :param path: The path where the state machine resides
        :param start_state_path: The path to the state from which the execution will start
        :return: a reference to the created state machine
        """

        import rafcon.core.singleton
        from rafcon.core.storage import storage
        rafcon.core.singleton.library_manager.initialize()
        if not state_machine:
            state_machine = storage.load_state_machine_from_path(path)
            rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

        rafcon.core.singleton.state_machine_execution_engine.start(
            state_machine.state_machine_id, start_state_path=start_state_path)

        if wait_for_execution_finished:
            self.join()
            self.stop()
        return state_machine

    @Observable.observed
    def set_execution_mode(self, execution_mode, notify=True):
        """ An observed setter for the execution mode of the state machine status. This is necessary for the
        monitoring client to update the local state machine in the same way as the root state machine of the server.

        :param execution_mode: the new execution mode of the state machine
        :raises exceptions.TypeError: if the execution mode is of the wrong type
        """
        if not isinstance(execution_mode, StateMachineExecutionStatus):
            raise TypeError("status must be of type StateMachineExecutionStatus")
        self._status.execution_mode = execution_mode
        if notify:
            self._status.execution_condition_variable.acquire()
            self._status.execution_condition_variable.notify_all()
            self._status.execution_condition_variable.release()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
    #########################################################################

    @property
    def status(self):
        """Property for the _status field

        """
        return self._status

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

