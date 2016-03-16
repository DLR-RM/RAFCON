"""
.. module:: monitoring client
   :platform: Unix, Windows
   :synopsis: A module to care about receiving execution status from another RAFCON instance and showing it in its own
            RAFCON instance

.. moduleauthor:: Sebastian Brunner


"""

import time

from plugins.monitoring.monitoring_menu_bar import MonitoringMenuBar

from network.udp_client import UdpClient
from network.protocol import Protocol, MessageType, STATE_EXECUTION_STATUS_SEPARATOR
from network.config import global_network_config

from rafcon.statemachine.enums import StateExecutionState, StateMachineExecutionStatus
from rafcon.statemachine.singleton import state_machine_manager
from rafcon.statemachine.singleton import state_machine_execution_engine

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringClient(UdpClient):

    def __init__(self):
        UdpClient.__init__(self)
        self.connector = None
        self.initialized = False
        self.server_address = None
        # self.datagram_received_function = self.print_message
        self.datagram_received_function = self.client_monitoring_data_received_function

    def connect(self):
        # replace GUI menu bar with own subclassed version
        from rafcon.mvc.singleton import main_window_controller
        main_window_controller.remove_controller("menu_bar_controller")
        new_menu_bar_controller = MonitoringMenuBar(main_window_controller.state_machine_manager_model,
                                                    main_window_controller.view,
                                                    main_window_controller.shortcut_manager)
        main_window_controller.add_controller("menu_bar_controller", new_menu_bar_controller)

        # register observer
        self.register_execution_engine()

        # setup twisted
        from twisted.internet import reactor
        self.connector = reactor.listenUDP(0, self)
        self.initialized = True
        protocol = Protocol(MessageType.REGISTER, "Registering")
        self.server_address = (global_network_config.get_config_value("SERVER_IP"),
                               global_network_config.get_config_value("SERVER_UDP_PORT"))

        logger.info("Connect to server {0}!".format(str(self.server_address)))
        print "sending protocol " + str(protocol)
        return_value = self.send_message_acknowledged(protocol, address=self.server_address, blocking=True)
        if return_value:
            logger.info("Connected to server!")
            return True
        else:
            logger.error("Connection to server {0} timeout".format(str(self.server_address)))
            return False

    def print_message(self, message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))

    def client_monitoring_data_received_function(self, message, address):
        assert isinstance(message, Protocol)
        if message.message_type is MessageType.STATE_ID:
            (state_path, execution_status) = message.message_content.split("@")
            state_execution_status = StateExecutionState(int(execution_status))
            # logger.info("Received state_id {0}".format(str(state_path)))
            # logger.info("Received execution_status {0} {1}".format(str(execution_status),
            #                                                        str(state_execution_status)))

            current_state = state_machine_manager.get_active_state_machine().get_state_by_path(state_path)
            current_state.state_execution_status = state_execution_status

    def register_execution_engine(self):
        state_machine_execution_engine.add_observer(self, "set_execution_mode",
                                                    notify_after_function=self.on_execution_mode_changed_after)

    def send_current_execution_mode(self):
        protocol = Protocol(MessageType.STATE_ID, str(state_machine_execution_engine.status.execution_mode.value))
        print protocol
        self.send_message_non_acknowledged(protocol, self.server_address)

    def on_execution_mode_changed_after(self, observable, return_value, args):
        logger.info("This is naice!")
        self.send_current_execution_mode()


