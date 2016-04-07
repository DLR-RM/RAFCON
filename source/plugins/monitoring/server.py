"""
.. module:: monitoring server
   :platform: Unix, Windows
   :synopsis: A module to care about providing execution status to other RAFCON instances

.. moduleauthor:: Sebastian Brunner


"""

from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.state import State
from rafcon.statemachine.singleton import state_machine_manager, state_machine_execution_engine
from rafcon.statemachine.enums import StateMachineExecutionStatus

from acknowledged_udp.config import global_network_config
from acknowledged_udp.protocol import Protocol, MessageType, STATE_EXECUTION_STATUS_SEPARATOR
from acknowledged_udp.udp_server import UdpServer

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringServer(UdpServer):
    """
    This class takes care about providing execution data to remote clients. The paths of currently executed states
    are broadcasted while on the other hand execution commands form the clients are forwarded to the local
    execution engine
    """

    def __init__(self):
        UdpServer.__init__(self)
        self.connector = None
        self.initialized = False
        self.register_to_new_state_machines()
        self.register_all_statemachines()
        self.datagram_received_function = self.monitoring_data_received_function

    def connect(self):
        """
        This function opens a udp port for remote clients
        :return:
        """
        from twisted.internet import reactor
        self.connector = reactor.listenUDP(global_network_config.get_config_value("SERVER_UDP_PORT"), self)
        self.initialized = True
        return True

    def register_to_new_state_machines(self):
        """
        This functions registers to add_state_machine calls of the state machine manager
        :return:
        """
        state_machine_manager.add_observer(self, "add_state_machine",
                                           notify_after_function=self.on_add_state_machine_after)

    def on_add_state_machine_after(self, observable, return_value, args):
        """
        This function specifies what happens when a state machine is added to the state machine manager
        :param observable: the state machine manager
        :param return_value: the new state machine
        :param args:
        :return:
        """
        self.register_states_of_state_machine(args[1])

    def register_all_statemachines(self):
        """
        This functions registers to all state machines currently known to the state machine manager
        :return:
        """
        for id, sm in state_machine_manager.state_machines.iteritems():
            self.register_states_of_state_machine(sm)

    def register_states_of_state_machine(self, state_machine):
        """
        This functions registers all states of state machine.
        :param state_machine: the state machine to register all states of
        :return:
        """
        root = state_machine.root_state
        root.add_observer(self, "state_execution_status",
                          notify_after_function=self.on_state_execution_status_changed_after)

        self.recursively_register_child_states(root)

    def recursively_register_child_states(self, state):
        """
        A function tha registers recursively all child states of a state
        :param state:
        :return:
        """
        if isinstance(state, ContainerState):
            for state in state.states.itervalues():
                self.recursively_register_child_states(state)
                state.add_observer(self, "state_execution_status",
                                   notify_after_function=self.on_state_execution_status_changed_after)

        if isinstance(state, LibraryState):
            self.recursively_register_child_states(state.state_copy)
            state.add_observer(self, "state_execution_status",
                               notify_after_function=self.on_state_execution_status_changed_after)

    def on_state_execution_status_changed_after(self, observable, return_value, args):
        """
        This function specifies what happens if the state machine execution status of a state changes
        :param observable: the state whose execution status changed
        :param return_value: the new execution status
        :param args: a list of all arguments of the observed function
        :return:
        """
        assert isinstance(observable, State)
        message = observable.get_path() + STATE_EXECUTION_STATUS_SEPARATOR + str(observable.state_execution_status.value)
        protocol = Protocol(MessageType.STATE_ID, message)
        # if len(self.get_registered_endpoints()) == 0:
        #     logger.warn("No endpoint registered yet")
        if self.initialized:
            for address in self.get_registered_endpoints():
                self.send_message_non_acknowledged(protocol, address)
        else:
            logger.warn("Not initialized yet")

    def monitoring_data_received_function(self, message, address):
        """
        This functions receives all messages sent by the remote RAFCON server.
        :param message: the received message
        :param address: the address (port, ip-address) of the remote server
        :return:
        """
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))

        assert isinstance(message, Protocol)
        if message.message_type is MessageType.COMMAND:
            received_command = message.message_content.split("@")

            execution_mode = StateMachineExecutionStatus(int(received_command[0]))

            if execution_mode is StateMachineExecutionStatus.STARTED:
                # as there is no dedicated RUN_TO_STATE execution status the message has to be checked for an optional
                # start state path
                if len(received_command) == 2:
                    print "start state machine from state " + received_command[1]
                    state_machine_execution_engine.start(start_state_path=received_command[1])
                else:
                    state_machine_execution_engine.start()
            elif execution_mode is StateMachineExecutionStatus.STOPPED:
                state_machine_execution_engine.stop()
            elif execution_mode is StateMachineExecutionStatus.PAUSED:
                state_machine_execution_engine.pause()
            elif execution_mode is StateMachineExecutionStatus.FORWARD_INTO:
                state_machine_execution_engine.step_into()
            elif execution_mode is StateMachineExecutionStatus.FORWARD_OVER:
                state_machine_execution_engine.step_over()
            elif execution_mode is StateMachineExecutionStatus.FORWARD_OUT:
                state_machine_execution_engine.step_out()
            elif execution_mode is StateMachineExecutionStatus.BACKWARD:
                state_machine_execution_engine.backward_step()
            elif execution_mode is StateMachineExecutionStatus.RUN_TO_SELECTED_STATE:
                state_machine_execution_engine.run_to_selected_state(received_command[1])

    def print_message(self, message, address):
        """
        A dummy funcion to just print a message from a certain address.
        :param message: the received message
        :param address: the origin of the message
        :return:
        """
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))

