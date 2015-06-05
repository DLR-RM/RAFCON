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
        self.incoming_data_buffer = ""

    def connectionMade(self):
        self.factory.num_connections += 1

    def dataReceived(self, data):
        self.incoming_data_buffer += data
        if data.endswith("TRANSMISSION_END"):
            f = open("/home/flow/tcp_received", "w")
            f.write(self.incoming_data_buffer.split("TRANSMISSION_END")[0])
            f.close()
            self.factory.forward_received_data(self, self.incoming_data_buffer.split("TRANSMISSION_END")[0])
            self.incoming_data_buffer = ""
        # self.transport.write(data)

    def connectionLost(self, reason):
        self.factory.num_connections -= 1


class TCPFactory(protocol.Factory, gobject.GObject):
    """
    This class is used to manage the TCP connections. It also keeps track of how many connections are currently
    available.
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