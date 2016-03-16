"""
.. module:: monitoring server
   :platform: Unix, Windows
   :synopsis: A module to care about providing execution status to other RAFCON instances

.. moduleauthor:: Sebastian Brunner


"""

from rafcon.utils import log
logger = log.get_logger(__name__)

from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.state import State
from rafcon.statemachine.singleton import state_machine_manager

from network.config import global_network_config
from network.protocol import Protocol, MessageType, STATE_EXECUTION_STATUS_SEPARATOR
from network.udp_server import UdpServer

from plugins.monitoring.client import MonitoringClient


class MonitoringServer(UdpServer):

    def __init__(self):
        UdpServer.__init__(self)
        self.connector = None
        self.initialized = False
        self.register_to_new_state_machines()
        self.register_all_statemachines()
        self.datagram_received_function = self.print_message

    def connect(self):
        from twisted.internet import reactor
        self.connector = reactor.listenUDP(global_network_config.get_config_value("SERVER_UDP_PORT"), self)
        self.initialized = True
        return True

    def register_to_new_state_machines(self):
        state_machine_manager.add_observer(self, "add_state_machine",
                                           notify_after_function=self.on_add_state_machine_after)

    def on_add_state_machine_after(self, observable, return_value, args):
        self.register_states_of_state_machine(args[1])

    def register_all_statemachines(self):
        for id, sm in state_machine_manager.state_machines.iteritems():
            self.register_states_of_state_machine(sm)

    def register_states_of_state_machine(self, state_machine):
        root = state_machine.root_state
        root.add_observer(self, "state_execution_status",
                          notify_after_function=self.on_state_execution_status_changed_after)

        self.recursively_register_child_states(root)

    def recursively_register_child_states(self, state):

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
        assert isinstance(observable, State)
        message = observable.get_path() + STATE_EXECUTION_STATUS_SEPARATOR + str(observable.state_execution_status.value)
        print message
        protocol = Protocol(MessageType.STATE_ID, message)
        if len(self.get_registered_endpoints()) == 0:
            print "no endpoint registered yet"
        if self.initialized:
            for address in self.get_registered_endpoints():
                self.send_message_non_acknowledged(protocol, address)
        else:
            print "not initialized yet"

    def print_message(self, message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))
        logger.info("If the datagram is a command message in will be forwarded to the execution engine")

