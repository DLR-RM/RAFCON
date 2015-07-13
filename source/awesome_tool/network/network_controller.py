from enum import Enum

from threading import Timer

from awesome_tool.network.udp_connection import UDPConnection
from awesome_tool.network.tcp_connection import TCPFactory, TCPClientFactory
from awesome_tool.network.enums import ConnectionMode

from awesome_tool.utils import log
logger = log.get_logger(__name__)


# --------------------------------------------
#               TWISTED
# --------------------------------------------
from twisted.internet import reactor
from twisted.internet.error import CannotListenError

NetworkMode = Enum('NETWORK_MODE', 'TCP UDP')


class NetworkController:
    """
    This class is responsible to coordinate all network connections via TCP or UDP.
    :param network_mode: The used network mode the current instance handles: TCP or UDP
    """

    def __init__(self, network_mode):
        assert isinstance(network_mode, NetworkMode)
        self.network_mode = network_mode
        self.udp_connections = {}
        self.tcp_connections = {}
        self.tcp_clients = {}

        self.udp_connection_reactor_ports = {}
        self.tcp_connectors = {}

    def start(self, port, connection_mode):
        """
        This method starts the TCP/UDP server and tells it to listen on the specified port for incoming connections.
        :param port: Port to listen to incoming connections
        :return: added connection if server started correctly, None if not
        """
        if self.network_mode == NetworkMode.TCP:
            if self.add_tcp_connection(port):
                return self.tcp_connections[port]
            else:
                return None
        elif self.network_mode == NetworkMode.UDP:
            if self.add_udp_connection(port, connection_mode):
                return self.udp_connections[port]
            else:
                return None
        else:
            logger.debug("Unrecognized NetworkMode detected")
            return None

    def restart(self, port):
        if self.network_mode == NetworkMode.UDP:
            if port in self.udp_connections.iterkeys() and port not in self.udp_connection_reactor_ports.iterkeys():
                try:
                    reactor_port = reactor.listenUDP(port, self.udp_connections[port])
                except CannotListenError:
                    timer = Timer(.1, self.restart, (port,))
                    timer.start()
                    return False
                else:
                    self.udp_connection_reactor_ports[port] = reactor_port
                    return True

    def stop(self, port):
        if self.network_mode == NetworkMode.UDP:
            if port in self.udp_connection_reactor_ports.iterkeys():
                self.udp_connection_reactor_ports[port].stopListening()
                del self.udp_connection_reactor_ports[port]

    def add_udp_connection(self, port, connection_mode):
        """
        Adds a new UDP connection to the server if not already existing
        :param port: Port to listen to incoming connections
        :return: True if port was added, no if port already existing
        """
        if port not in self.udp_connections.keys():
            udp_con = UDPConnection(connection_mode)
            try:
                reactor_port = reactor.listenUDP(port, udp_con)
                self.udp_connections[port] = udp_con
                self.udp_connection_reactor_ports[port] = reactor_port
            except CannotListenError:
                self.print_port_in_use(port)
                return False
            return True
        else:
            self.print_port_in_use(port)
            return False

    def add_tcp_connection(self, port):
        """
        Adds a new TCP connection to the server if not already existing
        :param port: Port to listen to incoming connections
        :return: True if port was added, no if port already existing
        """
        if port not in self.tcp_connections.keys():
            tcp_fac = TCPFactory()
            try:
                reactor.listenTCP(port, tcp_fac)
                self.tcp_connections[port] = tcp_fac
            except CannotListenError:
                self.print_port_in_use(port)
                return False
            return True
        else:
            self.print_port_in_use(port)
            return False

    def connect_tcp(self, ip, port):
        if self.network_mode == NetworkMode.TCP and (ip, port) not in self.tcp_clients.iterkeys():
            tcp_client_fac = TCPClientFactory()
            self.tcp_connectors[(ip, port)] = reactor.connectTCP(ip, port, tcp_client_fac)
            self.tcp_clients[(ip, port)] = tcp_client_fac

    def get_tcp_clients(self):
        if self.network_mode == NetworkMode.TCP:
            return self.tcp_clients

    def print_port_in_use(self, port):
        """
        Print message to textview stating the port is already in use
        :param port: Port in use
        """
        logger.debug("Port %d already in use for %s" % (port, self.network_mode))

    def get_listened_ports(self):
        """
        Returns all ports the server listens to for its network mode
        :return: Listened ports
        """
        if self.network_mode == NetworkMode.TCP:
            return self.tcp_connections.keys()
        elif self.network_mode == NetworkMode.UDP:
            return self.udp_connections.keys()

    def get_connections(self):
        """
        Returns the list of connections to the server
        :return: List of connections
        """
        if self.network_mode == NetworkMode.TCP:
            return self.tcp_connections
        elif self.network_mode == NetworkMode.UDP:
            return self.udp_connections

    def get_reactor_ports(self):
        if self.network_mode == NetworkMode.UDP:
            return self.udp_connection_reactor_ports
        elif self.network_mode == NetworkMode.TCP:
            return self.tcp_connectors


class TCPNetworkController(NetworkController):

    def __init__(self):
        NetworkController.__init__(self, NetworkMode.TCP)


class UDPNetworkController(NetworkController):

    def __init__(self):
        NetworkController.__init__(self, NetworkMode.UDP)