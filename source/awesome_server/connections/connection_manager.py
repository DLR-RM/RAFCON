from gtkmvc import Observable

from awesome_server.mvc.controller.network_controller import NetworkController, NetworkMode

from awesome_server.utils import messaging


class ConnectionManager(Observable):

    # TODO: replace view with logger
    def __init__(self, view):
        Observable.__init__(self)
        self._udp_connections = []
        self._tcp_connections = []

        self._rcvd_udp_messages_tmp = ""

        self.server_udp = NetworkController(NetworkMode.UDP, view)
        self.server_tcp = NetworkController(NetworkMode.TCP, view)

    @Observable.observed
    def tcp_data_received(self, factory, connection, data):
        pass

    def udp_data_received(self, connection, data, ip, port):
        checksum = data[:39]
        if messaging.check_checksum(data) and checksum != self._rcvd_udp_messages_tmp:
            self._rcvd_udp_messages_tmp = checksum
            self.new_udp_message_detected(connection, data, ip, port)

    @Observable.observed
    def new_udp_message_detected(self, connection, data, ip, port):
        connection.send_acknowledge(data[:39], (ip, port))

    def add_tcp_connection(self, port):
        tcp_con = self.server_tcp.start(port)
        if tcp_con:
            tcp_con.connect("data_received", self.tcp_data_received)
            self._tcp_connections.append(tcp_con)

    @Observable.observed
    def add_udp_connection(self, port):
        udp_con = self.server_udp.start(port)
        if udp_con:
            udp_con.connect("data_received", self.udp_data_received)
            self._udp_connections.append(udp_con)
            return udp_con
        return None

    @property
    def udp_connections(self):
        return self._udp_connections

    @property
    def tcp_connections(self):
        return self._tcp_connections