from gtkmvc import Observable

from awesome_server.mvc.controller.network_controller import NetworkController, NetworkMode


class ConnectionManager(Observable):

    def __init__(self, view):
        Observable.__init__(self)
        self._udp_connections = []
        self._tcp_connections = []

        self.server_udp = NetworkController(NetworkMode.UDP, view)
        self.server_tcp = NetworkController(NetworkMode.TCP, view)

    def add_tcp_connection(self, port):
        tcp_con = self.server_tcp.start(port)
        if tcp_con:
            self._tcp_connections.append(tcp_con)

    @Observable.observed
    def add_udp_connection(self, port):
        udp_con = self.server_udp.start(port)
        if udp_con:
            self._udp_connections.append(udp_con)
            return udp_con
        return None

    @property
    def udp_connections(self):
        return self._udp_connections

    @property
    def tcp_connections(self):
        return self._tcp_connections