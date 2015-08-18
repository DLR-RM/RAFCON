from gtkmvc import Observable
from rafcon.statemachine.singleton import state_machine_manager
from rafcon.network.singleton import udp_net_controller, tcp_net_controller, html_network_controller
from rafcon.network.enums import ConnectionMode
from rafcon.utils import log

logger = log.get_logger(__name__)


class ConnectionManager(Observable):
    """
    The ConnectionManager is responsible of all connections coming from and to the server.
    """

    def __init__(self):
        Observable.__init__(self)

        self.server_udp = udp_net_controller
        self.server_tcp = tcp_net_controller

        self.server_html = html_network_controller
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
        tcp_con = self.server_tcp.start(port, ConnectionMode.SERVER)
        if tcp_con:
            tcp_con.connect("data_received", self.tcp_data_received)

    @Observable.observed
    def add_udp_connection(self, port):
        """
        Adds new UDP Connection to manager
        :param port: Port to listen for incoming UDP connections
        :return: New UDP connection if successfully created, None otherwise
        """
        udp_con = self.server_udp.start(port, ConnectionMode.SERVER)
        if udp_con:
            udp_con.connect("data_received", self.udp_data_received)
            return udp_con
        return None