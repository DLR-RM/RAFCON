from builtins import object
import rafcon.core.singleton
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionEngineObserver(object):

    def __init__(self):
        self.logger = log.get_logger(type(self).__name__)
        self.execution_engine = rafcon.core.singleton.state_machine_execution_engine
        self.register_observer()

    def register_observer(self):
        """ Register all observable which are of interest
        """
        self.execution_engine.add_observer(self, "start", notify_before_function=self.on_start)
        self.execution_engine.add_observer(self, "pause", notify_before_function=self.on_pause)
        self.execution_engine.add_observer(self, "stop", notify_before_function=self.on_stop)

    def on_start(self, arg, kwargs):
        self.logger.info("ExecutionEngine will be set to 'start'.")

    def on_pause(self, arg, kwargs):
        self.logger.info("ExecutionEngine will be set to 'pause'.")

    def on_stop(self, arg, kwargs):
        self.logger.info("ExecutionEngine will be set to 'stop'.")


class ExecutionStatusObserver(object):
    """ The Observer registers recursively to all states of all state-machines known at time of initiation and
    registers to newly add state-machines by the state-machine-manager.
    """

    def __init__(self):
        self.logger = log.get_logger(type(self).__name__)
        self.logger.info("Initiate ExecutionStatusObserver")
        for id, sm in list(rafcon.core.singleton.state_machine_manager.state_machines.items()):
            self.register_states_of_state_machine(sm)
        rafcon.core.singleton.state_machine_manager.add_observer(self, "add_state_machine",
                                                                         notify_after_function=self.on_add_state_machine_after)

    def register_states_of_state_machine(self, state_machine):
        """ This functions registers all states of state machine.
        :param state_machine: the state machine to register all states of
        :return:
        """
        root = state_machine.root_state
        root.add_observer(self, "state_execution_status",
                          notify_after_function=self.on_state_execution_status_changed_after)
        self.recursively_register_child_states(root)

    def recursively_register_child_states(self, state):
        """ A function tha registers recursively all child states of a state
        :param state:
        :return:
        """
        self.logger.info("Execution status observer add new state {}".format(state))
        if isinstance(state, ContainerState):
            state.add_observer(self, "add_state",
                               notify_after_function=self.on_add_state)
            for state in list(state.states.values()):
                self.recursively_register_child_states(state)
                state.add_observer(self, "state_execution_status",
                                   notify_after_function=self.on_state_execution_status_changed_after)

        if isinstance(state, LibraryState):
            self.recursively_register_child_states(state.state_copy)
            state.add_observer(self, "state_execution_status",
                               notify_after_function=self.on_state_execution_status_changed_after)

    def on_add_state_machine_after(self, observable, return_value, args):
        """ This method specifies what happens when a state machine is added to the state machine manager
        :param observable: the state machine manager
        :param return_value: the new state machine
        :param args:
        :return:
        """
        self.logger.info("Execution status observer register new state machine sm_id: {}".format(args[1].state_machine_id))
        self.register_states_of_state_machine(args[1])

    def on_add_state(self, observable, return_value, args):
        logger.info("Execution status observer add new state")
        if return_value is not None:
            state = observable.states[return_value]
            self.recursively_register_child_states(state)
            state.add_observer(self, "state_execution_status",
                               notify_after_function=self.on_state_execution_status_changed_after)
            state.add_observer(self, "add_state",
                               notify_after_function=self.on_add_state)

    def on_state_execution_status_changed_after(self, observable, return_value, args):
        """ This function specifies what happens if the state machine execution status of a state changes
        :param observable: the state whose execution status changed
        :param return_value: the new execution status
        :param args: a list of all arguments of the observed function
        :return:
        """
        self.logger.info("Execution status has changed for state '{0}' to status: {1}"
                         "".format(observable.get_path(by_name=True), observable.state_execution_status))
