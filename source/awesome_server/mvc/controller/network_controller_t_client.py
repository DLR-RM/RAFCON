from twisted.internet import reactor, protocol
from twisted.internet.protocol import DatagramProtocol

from awesome_server.mvc.controller.network_controller import NetworkMode


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

    def startProtocol(self):
        self.transport.write("Message from client using UDP", (self.ip, self.port))

    def datagramReceived(self, data, addr):
        buffer = self.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), "UDP-CLIENT on port %d: %s from %s\n" % (self.my_port, data, repr(addr)))