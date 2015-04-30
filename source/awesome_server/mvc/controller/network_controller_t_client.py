from twisted.internet import reactor, protocol
from twisted.internet.protocol import DatagramProtocol

from awesome_server.mvc.controller.network_controller import NetworkMode
from awesome_server.utils import messaging, constants
from awesome_server.utils.messaging import Message

from awesome_server.utils.config import global_server_config


class ClientController:

    def __init__(self, view, port, mode):
        if mode == NetworkMode.TCP:
            reactor.connectTCP("localhost", port, EchoFactory(view))
        elif mode == NetworkMode.UDP:
            reactor.listenUDP(port, UDPClient("127.0.0.1", 9999, port, view))


class EchoClient(protocol.Protocol):

    def __init__(self, message, factory):
        self.message = message
        self.factory = factory

    def connectionMade(self):
        self.transport.write(self.message)

    def dataReceived(self, data):
        buffer = self.factory.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), "TCP-CLIENT: " + data + "\n")
        self.transport.loseConnection()


class EchoFactory(protocol.ClientFactory):

    def __init__(self, view):
        self.view = view

    def buildProtocol(self, addr):
        return EchoClient("Message from client using TCP", self)


class UDPClient(DatagramProtocol):

    def __init__(self, ip, port, my_port, view):
        self.ip = ip
        self.port = port
        self.my_port = my_port
        self.view = view

        self._rcvd_udp_messages_tmp = [""] * global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY")
        self._current_rcvd_index = 0

    def startProtocol(self):
        encr_msg = messaging.create_send_message("Message from client using UDP")
        for i in range(0, 10):
            self.transport.write("1"+encr_msg, (self.ip, self.port))

    def datagramReceived(self, data, addr):
        msg = Message.parse_from_string(data)
        checksum = data[1:constants.HEADER_LENGTH]
        if messaging.check_checksum(data) and msg.message_id not in self._rcvd_udp_messages_tmp:
            self._rcvd_udp_messages_tmp[self._current_rcvd_index] = checksum
            self._current_rcvd_index += 1
            if self._current_rcvd_index >= global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY"):
                self._current_rcvd_index = 0
            if self.check_acknowledge(data[constants.HEADER_LENGTH:]):
                self.transport.write("0"+messaging.create_send_message(checksum + "ACK"), addr)
            buffer = self.view["textview"].get_buffer()
            buffer.insert(buffer.get_end_iter(), "UDP-CLIENT on port %d: %s from %s\n" % (self.my_port, data[40:], repr(addr)))

    def check_acknowledge(self, message):
        if message[:1] == "1":
            return True
        return False