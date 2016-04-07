"""
.. module:: monitoring client
   :platform: Unix, Windows
   :synopsis: A module to care about receiving execution status from another RAFCON instance and showing it in its own
            RAFCON instance

.. moduleauthor:: Sebastian Brunner


"""

import time

from plugins.monitoring.monitoring_execution_engine import MonitoringExecutionEngine

from python_acknowledged_udp.udp_client import UdpClient
from python_acknowledged_udp.protocol import Protocol, MessageType, STATE_EXECUTION_STATUS_SEPARATOR
from python_acknowledged_udp.config import global_network_config

from rafcon.statemachine.enums import StateExecutionState, StateMachineExecutionStatus
from rafcon.statemachine.singleton import state_machine_manager
import rafcon

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringClient(UdpClient):
    """
    A class to care about receiving execution status from another RAFCON instance and showing it in its own
    RAFCON instance
    """

    def __init__(self):
        UdpClient.__init__(self)
        self.connector = None
        self.server_address = None
        self.datagram_received_function = self.monitoring_data_received_function
        self.execution_engine_replaced = False
        self.registered_to_execution_engine = False
        self.connector_created = False
        self.registered_to_server = False

    def connect(self):
        """
        Connect to the remote RAFCON server instance. Several things are achieved here:
        - replacing the menu bar by a menu bar controlling the remote RAFCON instance
        - registering to changes of the local execution engine
        - registering to the remote RAFCON server instance
        :return:
        """

        # replace state machine execution engine
        if not self.execution_engine_replaced:
            from rafcon.mvc.singleton import main_window_controller
            # wait until the main window controller registered its view
            while not main_window_controller.get_controller("menu_bar_controller").registered_view:
                time.sleep(0.1)
            monitoring_execution_engine = MonitoringExecutionEngine(state_machine_manager, self)
            # global replacement
            # TODO: modules that have already imported the singleton.state_machine_execution_engine
            # still have their old reference!!!
            rafcon.statemachine.singleton.state_machine_execution_engine = monitoring_execution_engine
            from rafcon.mvc.models.state_machine_execution_engine import StateMachineExecutionEngineModel
            rafcon.mvc.singleton.state_machine_execution_manager_model = \
                StateMachineExecutionEngineModel(rafcon.statemachine.singleton.state_machine_execution_engine)
            main_window_controller.switch_state_machine_execution_engine(
                rafcon.mvc.singleton.state_machine_execution_manager_model)
            logger.info("state machine execution engine replaced")

            # replacement for main_window_controller_only
            from rafcon.mvc.singleton import main_window_controller
            main_window_controller.get_controller("menu_bar_controller").state_machine_execution_engine = \
                monitoring_execution_engine
            self.execution_engine_replaced = True

        # if not self.connector_created:
        # setup twisted
        from twisted.internet import reactor
        self.server_address = (global_network_config.get_config_value("SERVER_IP"),
                               global_network_config.get_config_value("SERVER_UDP_PORT"))
        logger.info("Connect to server {0}!".format(str(self.server_address)))
        self.connector = reactor.listenUDP(0, self)
        self.connector_created = True
        logger.info("self.connector {0}".format(str(self.connector)))

        protocol = Protocol(MessageType.REGISTER, "Registering")
        logger.info("sending protocol {0}".format(str(protocol)))
        return_value = self.send_message_acknowledged(protocol, address=self.server_address, blocking=True)

        if return_value:
            logger.info("Connected to server!")
            self.registered_to_server = True
            return True
        else:
            logger.error("Connection to server {0} timeout".format(str(self.server_address)))
            sleep_time = global_network_config.get_config_value("MAX_TIME_WAITING_BETWEEN_CONNECTION_TRY_OUTS")
            logger.info("Waiting for MAX_TIME_WAITING_BETWEEN_CONNECTION_TRY_OUTS={0} seconds!".format(str(sleep_time)))
            time.sleep(float(sleep_time))
            return False

    def print_message(self, message, address):
        """
        Simply print an input message
        :param message: the message to be printed
        :param address: the address where the message originates
        :return:
        """
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))

    def monitoring_data_received_function(self, message, address):
        """
        A function that orchestrates and processes the received messages
        :param message: the received message
        :param address: the address where the message originates
        :return:
        """
        assert isinstance(message, Protocol)
        if message.message_type is MessageType.STATE_ID:
            (state_path, execution_status) = message.message_content.split("@")
            state_execution_status = StateExecutionState(int(execution_status))
            # logger.info("Received state_id {0}".format(str(state_path)))
            # logger.info("Received execution_status {0} {1}".format(str(execution_status),
            #                                                        str(state_execution_status)))
            current_state = state_machine_manager.get_active_state_machine().get_state_by_path(state_path)
            current_state.state_execution_status = state_execution_status




