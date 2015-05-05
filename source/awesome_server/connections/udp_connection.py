from twisted.internet.protocol import DatagramProtocol

from gtkmvc import Observable
from threading import Thread, Event

import gobject
from awesome_server.utils.messaging import Message
from awesome_server.utils.config import global_server_config

from awesome_server.utils import log
logger = log.get_logger(__name__)


class UDPConnection(DatagramProtocol, Observable, gobject.GObject):
    """
    This class represents the UDP connection listener.
    It manages all clients communicating via UDP.
    """

    def __init__(self):
        self.__gobject_init__()
        Observable.__init__(self)

        self.clients = []
        self.messages_to_be_acknowledged = {}

        self._rcvd_udp_messages_history = [""] * global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY")
        self._current_rcvd_index = 0

    def datagramReceived(self, data, addr):
        """
        Handles received data.
        Emits "data_received" signal to process data in different class.
        :param data: Data received
        :param addr: Sender address
        """
        msg = Message.parse_from_string(data)
        if msg.check_checksum() and not self.check_for_message_in_history(msg):
            self.check_and_execute_flag(msg, addr)
            self.check_send_acknowledge(msg, addr)

            self.emit("data_received", msg)

    def check_for_message_in_history(self, msg):
        msg_already_received = msg.message_id in self._rcvd_udp_messages_history
        self._rcvd_udp_messages_history[self._current_rcvd_index] = msg.message_id
        self._current_rcvd_index += 1
        if self._current_rcvd_index >= global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY"):
            self._current_rcvd_index = 0
        return msg_already_received

    @Observable.observed
    def append_client(self, addr):
        """
        Appends ("registers") a new client to the clients managed by the UDPConnection
        :param addr: Address to be appended ("registered")
        """
        self.clients.append(addr)

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
        for i in range(0, global_server_config.get_config_value("NUMBER_UDP_MESSAGES_SENT")):
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
        number_of_repetitions = int(global_server_config.get_config_value("SECONDS_FOR_UDP_TIMEOUT") /
                                    global_server_config.get_config_value("SECONDS_BETWEEN_UDP_RESEND"))
        current_repetition = 0
        while not stop_event.is_set() and current_repetition < number_of_repetitions:
            current_repetition += 1
            self.send_message(message, addr)
            stop_event.wait(global_server_config.get_config_value("SECONDS_BETWEEN_UDP_RESEND"))

    def check_and_execute_flag(self, msg, addr):
        """
        This method checks the flag of the incoming received data and executes the corresponding code.
        :param msg: Received message to execute
        """
        if msg.flag == "REG" and addr not in self.clients:
            logger.debug("Register new statemachine at address: %s" % repr(addr))
            self.append_client(addr)
        elif msg.flag == "ACK" and msg.message in self.messages_to_be_acknowledged.iterkeys():
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


gobject.type_register(UDPConnection)
gobject.signal_new("data_received", UDPConnection, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING, ))