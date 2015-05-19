from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers.menu_bar_controller import MenuBarController

from awesome_tool.mvc.models.container_state import ContainerStateModel
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.state_machine import StateMachineModel
from awesome_tool.mvc.models.state import StateModel

from awesome_tool.mvc.views.network_connections import NetworkConnectionsView

from awesome_tool.statemachine.storage.network_storage import NetworkStorageReader
from awesome_tool.statemachine.storage.protobuf import yaml_transmission_pb2
from awesome_tool.statemachine.enums import StateExecutionState

from awesome_tool.utils.network_messaging import Message

from awesome_tool.mvc.config_network import global_net_config

import awesome_tool.statemachine.singleton

from twisted.internet import reactor, protocol
from twisted.internet.protocol import DatagramProtocol
from twisted.internet.error import CannotListenError

from threading import Event, Thread

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class NetworkConnections(ExtendedController):
    """
    This controller takes care of the network connections to the server.
    It receives and sends messages via UDP and TCP.
    :param model: Model to observe changes and send to server
    :param view: View to control network connections
    """

    def __init__(self, model, view):
        assert isinstance(model, StateMachineManagerModel)
        assert isinstance(view, NetworkConnectionsView)
        ExtendedController.__init__(self, model, view)

        self.tcp_connection_factory = TCPFactory(self, view)
        self.udp_connection = UDPConnection(self, view)

        self.udp_connection_reactor_port = None
        self.tcp_connector = None

        self.udp_registered = False
        self.tcp_connected = False

        self.menu_bar_controller = None

        self.net_storage_reader = NetworkStorageReader(base_path=model.get_selected_state_machine_model().state_machine.base_path)

        # execution engine
        self.state_machine_execution_engine = awesome_tool.statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        self.previous_execution_message = ""

        if global_net_config.get_config_value("AUTOCONNECT_UDP_TO_SERVER"):
            self.on_udp_register_button_clicked(None)
        if global_net_config.get_config_value("AUTOCONNECT_TCP_TO_SERVER"):
            self.on_tcp_connect_button_clicked(None)

    def register_statemachine_model(self, sm_model):
        assert isinstance(sm_model, StateMachineModel)
        self.observe_model(sm_model)

    def register_menu_bar_controller(self, menu_bar_controller):
        assert isinstance(menu_bar_controller, MenuBarController)
        self.menu_bar_controller = menu_bar_controller

    def on_udp_register_button_clicked(self, widget, event=None):
        if not self.udp_registered:
            try:
                self.udp_connection_reactor_port = reactor.listenUDP(global_net_config.get_config_value("SELF_UDP_PORT"),
                                                                     self.udp_connection)
            except CannotListenError:
                logger.error("Cannot establish UDP connection - Port already in use")
            else:
                self.udp_registered = True

    def on_tcp_connect_button_clicked(self, widget, event=None):
        if global_net_config.get_config_value("SPACEBOARD_CUP_MODE"):
            logger.error("No TCP connection possible in Spaceboard Cup Mode")
            return
        if not self.tcp_connected and self.tcp_connector is None:
            self.tcp_connector = reactor.connectTCP(global_net_config.get_server_ip(),
                                                    global_net_config.get_server_tcp_port(),
                                                    self.tcp_connection_factory)
        elif not self.tcp_connected:
            self.tcp_connector.connect()
        else:
            self.tcp_connector.disconnect()

    def change_execution_mode(self, new_mode):
        """
        This method handles the change of the execution state and activates the corresponding execution mode.
        :param new_mode:
        :return:
        """
        if self.menu_bar_controller:
            if new_mode == 'run':
                self.menu_bar_controller.on_start_activate(None)
            elif new_mode == 'stop':
                self.menu_bar_controller.on_stop_activate(None)
            elif new_mode == 'pause':
                self.menu_bar_controller.on_pause_activate(None)
            elif new_mode == 'step_mode':
                self.menu_bar_controller.on_step_mode_activate(None)
            elif new_mode == 'step_forward':
                self.menu_bar_controller.on_step_activate(None)
            elif new_mode == 'step_backward':
                self.menu_bar_controller.on_backward_step_activate(None)
            else:
                logger.warning("Unrecognized mode detected.")

    @ExtendedController.observe("state_machine", after=True)
    def state_machine_change(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes.
        Here it is used to determine the active state change and send it to the server.

        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if 'method_name' in info and info['method_name'] == 'root_state_after_change' and self.udp_registered:
            kw_info = info['kwargs']['info']
            if 'method_name' in kw_info and kw_info['method_name'] == 'state_execution_status':
                kw_info_args = kw_info['args']
                if kw_info_args[1] == StateExecutionState.ACTIVE:
                    self.send_current_active_states(model.root_state)
                    if not self.previous_execution_message.startswith("-"):
                        self.udp_connection.send_non_acknowledged_message("------------------------------------",
                                                                          (global_net_config.get_server_ip(),
                                                                           global_net_config.get_server_udp_port()),
                                                                          "NAM")  # NAM = non acknowledged message
                        self.previous_execution_message = "------------------------------------"

    @ExtendedController.observe("execution_engine", after=True)
    def execution_mode_changed(self, model, prop_name, info):
        execution_mode = str(awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        execution_mode = execution_mode.replace("EXECUTION_MODE.", "")
        self.udp_connection.send_acknowledged_message(execution_mode,
                                                      (global_net_config.get_server_ip(),
                                                       global_net_config.get_server_udp_port()),
                                                      "EXE")

    @staticmethod
    def state_has_content(state_m):
        if isinstance(state_m, ContainerStateModel):
            return True
        return False

    def send_current_active_states(self, state_m):
        """
        This method recursively checks all states if they are active. If yes the path within the statemachine is sent
        to the server.
        :param state_m: StateModel to check
        """
        assert isinstance(state_m, StateModel)

        if state_m.state.state_execution_status == StateExecutionState.ACTIVE:
            if not self.state_has_content(state_m):
                print state_m.state.get_path()
                self.udp_connection.send_non_acknowledged_message(state_m.state.get_path(),
                                                                  (global_net_config.get_server_ip(),
                                                                   global_net_config.get_server_udp_port()),
                                                                  "NAM")  # NAM = non acknowledged message
                self.previous_execution_message = state_m.state.get_path()

        if self.state_has_content(state_m):
            for child_state in state_m.states.itervalues():
                self.send_current_active_states(child_state)


class TCPClient(protocol.Protocol):

    def __init__(self, factory):
        self.factory = factory

    def connectionMade(self):
        self.factory.view["tcp_connection_connected_label"].set_text("YES")
        self.factory.network_connection.tcp_connected = True
        self.factory.network_connection.view["tcp_connect_button"].set_label("Disconnect TCP")
        files = yaml_transmission_pb2.Files()
        sm_name = self.factory.network_connection.model.get_selected_state_machine_model().root_state.state.name
        files.sm_name = sm_name
        for path, content in self.factory.network_connection.net_storage_reader.file_storage.iteritems():
            my_file = files.files.add()
            my_file.file_path = str(path)
            my_file.file_content = content
        self.transport.write(files.SerializeToString())

    def connectionLost(self, reason):
        self.factory.view["tcp_connection_connected_label"].set_text("NO")
        self.factory.network_connection.tcp_connected = False
        self.factory.network_connection.view["tcp_connect_button"].set_label("Connect TCP")


class TCPFactory(protocol.ClientFactory):

    def __init__(self, network_connection, view):
        self.view = view
        self.network_connection = network_connection

    def buildProtocol(self, addr):
        return TCPClient(self)


class UDPConnection(DatagramProtocol):
    """
    This class represents the UDP connection listener.
    """

    def __init__(self, network_connection, view):
        self.view = view
        self.network_connection = network_connection

        self._rcvd_udp_messages_history = [""] * global_net_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY")
        self._current_rcvd_index = 0

        self.messages_to_be_acknowledged = {}

    def startProtocol(self):
        """
        Starts the UDP connection with the server and sends an initial "Register Statemachine" message
        """
        sm_name = self.network_connection.model.get_selected_state_machine_model().root_state.state.name
        if global_net_config.get_config_value("SPACEBOARD_CUP_MODE"):
            logger.warning("Register UDP connection without acknowledge - Spaceboard Cup Mode")
            self.send_non_acknowledged_message(sm_name,
                                               (global_net_config.get_server_ip(),
                                                global_net_config.get_server_udp_port()),
                                               "REG")
            self.view["udp_connection_registered_label"].set_text("YES")
        else:
            self.send_acknowledged_message(sm_name,
                                           (global_net_config.get_server_ip(),
                                            global_net_config.get_server_udp_port()),
                                           "REG")

    def datagramReceived(self, data, addr):
        """
        Receives data via UDP connection and performs the initial processing (check if checksum correct, checks for
        messages to stop sending as they are acknowledged, sends acknowledgement if necessary.)
        :param data: Received data
        :param addr: Sender address
        """
        msg = Message.parse_from_string(data)
        if msg.check_checksum() and not self.check_for_message_in_history(msg):
            self.view["udp_connection_registered_label"].set_text("YES")
            self.check_and_execute_flag(msg)
            self.check_send_acknowledge(msg, addr)

    def check_for_message_in_history(self, msg):
        """
        Checks if received message has already been received. Also keeps track of received messages and stores them
        until NUMBER_UDP_MESSAGES_HISTORY is reached. After reaching this number the entries are overwritten.
        :param msg: Message to check
        :return: True if message has already been received, False if it is a new message
        """
        msg_already_received = msg.message_id in self._rcvd_udp_messages_history
        self._rcvd_udp_messages_history[self._current_rcvd_index] = msg.message_id
        self._current_rcvd_index += 1
        if self._current_rcvd_index >= global_net_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY"):
            self._current_rcvd_index = 0
        return msg_already_received

    def send_message(self, message, addr):
        """
        This method sends the specified message to the given address as many times as specified in configuration.
        It does not expect an acknowledge and does not send the data again.
        :param message: Message to be sent
        :param addr: Receiver address
        :return:
        """
        assert isinstance(message, Message)
        logger.debug("Send message %s" % message.message_id)
        for i in range(0, global_net_config.get_config_value("NUMBER_UDP_MESSAGES_SENT")):
            self.transport.write(str(message), addr)

    def send_acknowledge(self, message_id, addr):
        """
        Sends acknowledge to sender for given message
        :param message_id: Message_ID of message to be acknowledged
        :param addr: Address of sender
        """
        msg = Message(message_id, 0, "ACK")
        logger.debug("Send acknowledge message for received message: %s with acknowledge id: %s" % (message_id,
                                                                                                    msg.message_id))
        self.send_message(msg, addr)

    def send_non_acknowledged_message(self, message, addr, flag="   "):
        msg = Message(message, 0, flag)
        self.send_message(msg, addr)

    def send_acknowledged_message(self, message, addr, flag="   "):
        """
        Sends the message repeatedly until it is acknowledged or timeout occurred.
        In list "messages_to_be_acknowledged" the Event 'stop_event' stops the sending when set() (see
        'check_acknowledge_and_stop_sending'), the boolean indicates if the message was acknowledged 'True'
        or aborted due to timeout 'False'.
        :param message: Message to be sent
        :param addr: Receiver address
        """
        msg = Message(message, 1, flag)
        stop_event = Event()

        self.messages_to_be_acknowledged[msg.message_id] = (stop_event, False)

        acknowledge_thread = Thread(target=self.repeat_sending_message, args=(msg, addr, stop_event))
        acknowledge_thread.start()

    def repeat_sending_message(self, message, addr, stop_event):
        """
        Sends the given message to the given address every x seconds (specified in config) until timeout occurs.
        :param message: Message to send
        :param addr: Receiver address
        :param stop_event: Event to stop thread/sending
        """
        number_of_repetitions = int(global_net_config.get_config_value("NUMBER_OF_UDP_PACKAGES_UNTIL_TIMEOUT"))
        current_repetition = 0
        while not stop_event.is_set() and current_repetition < number_of_repetitions:
            current_repetition += 1
            self.send_message(message, addr)
            stop_event.wait(global_net_config.get_config_value("SECONDS_BETWEEN_UDP_RESEND"))

        if current_repetition == number_of_repetitions:
            logger.warning("Connection Timeout: Server not reachable.")
            self.view["udp_connection_registered_label"].set_text("NO")
            self.network_connection.udp_registered = False
            self.network_connection.udp_connection_reactor_port.stopListening()

    def check_and_execute_flag(self, msg):
        """
        This method checks the flag of the incoming received data and executes the corresponding code.
        :param msg: Received message to execute
        """
        if msg.flag == "ACK" and msg.message in self.messages_to_be_acknowledged.iterkeys():
            logger.debug("Message %s acknowledged" % msg.message)
            stop_event = self.messages_to_be_acknowledged[msg.message][0]
            stop_event.set()
            self.messages_to_be_acknowledged[msg.message] = (stop_event, True)
        elif msg.flag == "EXE":
            self.network_connection.change_execution_mode(msg.message)

    def check_send_acknowledge(self, msg, addr):
        """
        Sends an acknowledge message to the sender for the given message, if the message needs to be acknowledged
        :param msg: Message to check
        :param addr: Address of sender
        """
        if msg.akg_msg == 1:
            self.send_acknowledge(msg.message_id, addr)