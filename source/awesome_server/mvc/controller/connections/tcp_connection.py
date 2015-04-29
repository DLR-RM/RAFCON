from twisted.internet import protocol


class TCPConnection(protocol.Protocol):
    """
    This class provides the server TCP connection interface.
    :param factory: The factory which built the connection.
    """

    def __init__(self, factory):
        self.factory = factory

    def connectionMade(self):
        self.factory.num_connections += 1
        buffer = self.factory.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), "SERVER: NUMBER OF CLIENTS: %d\n" % self.factory.num_connections)

    def dataReceived(self, data):
        buffer = self.factory.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), "SERVER: " + data + "\n")
        self.transport.write(data)

    def connectionLost(self, reason):
        self.factory.num_connections -= 1
        buffer = self.factory.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), "SERVER: NUMBER OF CLIENTS: %d\n" % self.factory.num_connections)


class TCPFactory(protocol.Factory):
    """
    This class is used to manage the TCP connections. It also keeps track of how many connections are currently
    available.
    :param view: For debug purpose: a view containing a textview to print debug information
    """

    def __init__(self, view):
        self.num_connections = 0
        self.view = view

    def buildProtocol(self, addr):
        return TCPConnection(self)

    def get_num_connections(self):
        return self.num_connections