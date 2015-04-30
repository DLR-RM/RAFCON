from gtkmvc import Observable

from awesome_server.mvc.controller.network_controller import NetworkController, NetworkMode

from awesome_server.utils import messaging, constants
from awesome_server.utils.config import global_server_config
from awesome_server.utils.messaging import Message


class ConnectionManager(Observable):
    """
    The ConnectionManager is responsible of all connections coming from and to the server.
    """

    def __init__(self):
        Observable.__init__(self)
        self._udp_connections = []
        self._tcp_connections = []

        self._rcvd_udp_messages_tmp = [""] * global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY")
        self._current_rcvd_index = 0

        self.server_udp = NetworkController(NetworkMode.UDP)
        self.server_tcp = NetworkController(NetworkMode.TCP)

    @Observable.observed
    def tcp_data_received(self, factory, connection, data):
        """
        Receives all data coming from TCP connections
        :param factory: TCP factory holding the connection
        :param connection: TCP connection receiving the data
        :param data: Received data
        """
        pass

    def udp_data_received(self, connection, data, ip, port):
        """
        Receives all data coming from UDP connections. Checks for transmission errors by checking the checksum.
        It keeps sorts out every duplicate message that is received due to multiple sending. Only stores each message
        and processes its data once.
        :param connection: UDP connection receiving the data
        :param data: Received data
        :param ip: Sender IP
        :param port: Sender Port
        """
        checksum = data[1:constants.HEADER_LENGTH]
        if messaging.check_checksum(data) and checksum not in self._rcvd_udp_messages_tmp:
            self._rcvd_udp_messages_tmp[self._current_rcvd_index] = checksum
            self._current_rcvd_index += 1
            if self._current_rcvd_index >= global_server_config.get_config_value("NUMBER_UDP_MESSAGES_HISTORY"):
                self._current_rcvd_index = 0
            self.new_udp_message_detected(connection, data, ip, port)

    @Observable.observed
    def new_udp_message_detected(self, connection, data, ip, port):
        """
        Method called by 'udp_data_received'. It processes the received data of the filtered message and sends an
        acknowledge to the sender.
        :param connection: UDP connection receiving the data
        :param data: Received data
        :param ip: Sender IP
        :param port: Sender Port
        """
        msg = Message.parse_from_string(data)
        if self.check_acknowledge(msg):
            connection.send_acknowledge(msg.message_id, (ip, port))

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

    def check_acknowledge(self, message):
        if message.akg_msg == 1:
            return True
        return False