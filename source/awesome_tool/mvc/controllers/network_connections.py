from awesome_tool.mvc.controllers.extended_controller import ExtendedController

from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.views.network_connections import NetworkConnectionsView
from awesome_tool.utils.messaging import Message

from awesome_tool.mvc.config_network import global_net_config

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

    def on_udp_register_button_clicked(self, widget, event=None):
        if not self.udp_registered:
            try:
                self.udp_connection_reactor_port = reactor.listenUDP(global_net_config.get_config_value("SELF_UDP_PORT"),
                                                                     self.udp_connection)
            except CannotListenError:
                logger.warning("Cannot establish UDP connection - Port already in use")
            else:
                self.udp_registered = True

    def on_tcp_connect_button_clicked(self, widget, event=None):
        if not self.tcp_connected:
            self.tcp_connector = reactor.connectTCP(global_net_config.get_config_value("SERVER_IP"),
                                                    global_net_config.get_config_value("SERVER_TCP_PORT"),
                                                    self.tcp_connection_factory)
        else:
            self.tcp_connector.disconnect()


class TCPClient(protocol.Protocol):

    def __init__(self, factory):
        self.factory = factory

    def connectionMade(self):
        self.factory.view["tcp_connection_connected_label"].set_text("YES")
        self.factory.network_connection.tcp_connected = True
        self.factory.network_connection.view["tcp_connect_button"].set_label("Disconnect TCP")

    def dataReceived(self, data):
        pass

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
        # TODO: add some information about statemachine to be able to handle more than one statemachine on server
        self.send_acknowledged_message("Register Statemachine",
                                       (global_net_config.get_config_value("SERVER_IP"),
                                        global_net_config.get_config_value("SERVER_UDP_PORT")))

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
            self.check_acknowledge_and_stop_sending(msg)
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
        logger.info("Send message %s" % message.message_id)
        for i in range(0, global_net_config.get_config_value("NUMBER_UDP_MESSAGES_SENT")):
            self.transport.write(str(message), addr)

    def send_acknowledge(self, message_id, addr):
        """
        Sends acknowledge to sender for given message
        :param message_id: Message_ID of message to be acknowledged
        :param addr: Address of sender
        """
        msg = Message(message_id, 0, "ACK")
        logger.info("Send acknowledge message for received message: %s with acknowledge id: %s" % (message_id,
                                                                                                   msg.message_id))
        self.send_message(msg, addr)

    def send_acknowledged_message(self, message, addr):
        """
        Sends the message repeatedly until it is acknowledged or timeout occurred.
        In list "messages_to_be_acknowledged" the Event 'stop_event' stops the sending when set() (see
        'check_acknowledge_and_stop_sending'), the boolean indicates if the message was acknowledged 'True'
        or aborted due to timeout 'False'.
        :param message: Message to be sent
        :param addr: Receiver address
        """
        msg = Message(message, 1)
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
        number_of_repetitions = int(global_net_config.get_config_value("SECONDS_FOR_UDP_TIMEOUT") /
                                    global_net_config.get_config_value("SECONDS_BETWEEN_UDP_RESEND"))
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

    def check_acknowledge_and_stop_sending(self, msg):
        """
        This method checks if the incoming received data is an acknowledge to one of the registered messages. If the
        message is registered and active the sending will be stopped and the acknowledge flag is set to True
        :param msg: Received message to scan for acknowledge
        """
        ack_flag = msg.flag
        if ack_flag == "ACK" and msg.message in self.messages_to_be_acknowledged.iterkeys():
            logger.debug("Message %s acknowledged" % msg.message)
            stop_event = self.messages_to_be_acknowledged[msg.message][0]
            stop_event.set()
            self.messages_to_be_acknowledged[msg.message] = (stop_event, True)

    def check_send_acknowledge(self, msg, addr):
        """
        Sends an acknowledge message to the sender for the given message, if the message needs to be acknowledged
        :param msg: Message to check
        :param addr: Address of sender
        """
        if msg.akg_msg == 1:
            self.send_acknowledge(msg.message_id, addr)