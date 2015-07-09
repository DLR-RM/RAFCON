from gtkmvc import Observable

from awesome_server.mvc.controller.sm_network_controller import SmNetworkController, NetworkMode
from awesome_server.mvc.controller.html_network_controller import HtmlNetworkController

from awesome_tool.statemachine.singleton import state_machine_manager

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class ConnectionManager(Observable):
    """
    The ConnectionManager is responsible of all connections coming from and to the server.
    """

    def __init__(self):
        Observable.__init__(self)
        self._udp_connections = []
        self._tcp_connections = []

        self.server_udp = SmNetworkController(NetworkMode.UDP)
        self.server_tcp = SmNetworkController(NetworkMode.TCP)

        self.server_html = HtmlNetworkController(self)
        self.server_html.start_html_server()

    @Observable.observed
    def tcp_data_received(self, factory, connection, data):
        """
        Receives all data coming from TCP connections
        :param factory: TCP factory holding the connection
        :param connection: TCP connection receiving the data
        :param data: Received data
        """
        pass

    def udp_data_received(self, connection, message, ip, port):
        if message.flag != "ACK":
            self.new_udp_message_detected(message, ip, port)

    @Observable.observed
    def new_udp_message_detected(self, msg, ip, port):
        """
        Method called by 'udp_data_received'. It processes the received data of the filtered message.
        :param msg: Received message
        """
        sm_id = state_machine_manager.get_sm_id_for_root_state_id(msg.root_id)
        if sm_id is not None:
            if msg.flag == "ASC" and not msg.message.startswith('-'):
                current_sm = state_machine_manager.state_machines[sm_id]
                active_state = current_sm.get_state_by_path(msg.message)
                if not active_state:
                    logger.error("Current active state not found in server side representation of state machine."
                                 "Please ensure the state machine model on the server matches the actual state machine")
                    return
            self.server_html.send_data(msg.message, ip, port, msg.flag, msg.sm_name)
        else:
            logger.warning("State machine with name: %s and root-ID: %s not loaded in server. "
                           "Cannot send data to browser." % (msg.sm_name, msg.root_id))

    def add_tcp_connection(self, port):
        """
        Adds new TCP Factory to manager
        :param port: Port to listen for incoming TCP connections
        """
        tcp_con = self.server_tcp.start(port)
        if tcp_con:
            tcp_con.connect("data_received", self.tcp_data_received)
            self._tcp_connections.append(tcp_con)

    @Observable.observed
    def add_udp_connection(self, port):
        """
        Adds new UDP Connection to manager
        :param port: Port to listen for incoming UDP connections
        :return: New UDP connection if successfully created, None otherwise
        """
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