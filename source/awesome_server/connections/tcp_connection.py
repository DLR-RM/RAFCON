from twisted.internet import protocol
import gobject


class TCPConnection(protocol.Protocol, gobject.GObject):
    """
    This class provides the server TCP connection interface.
    :param factory: The factory which built the connection.
    """

    def __init__(self, factory):
        self.__gobject_init__()
        self.factory = factory

    def connectionMade(self):
        self.factory.num_connections += 1

    def dataReceived(self, data):
        self.factory.forward_received_data(self, data)
        self.transport.write(data)

    def connectionLost(self, reason):
        self.factory.num_connections -= 1


class TCPFactory(protocol.Factory, gobject.GObject):
    """
    This class is used to manage the TCP connections. It also keeps track of how many connections are currently
    available.
    :param view: For debug purpose: a view containing a textview to print debug information
    """

    def __init__(self):
        self.__gobject_init__()
        self.num_connections = 0

    def buildProtocol(self, addr):
        return TCPConnection(self)

    def forward_received_data(self, connection, data):
        self.emit("data_received", connection, data)

    def get_num_connections(self):
        return self.num_connections


gobject.type_register(TCPFactory)
gobject.signal_new("data_received", TCPFactory, gobject.SIGNAL_RUN_FIRST, None, (TCPConnection, gobject.TYPE_STRING))