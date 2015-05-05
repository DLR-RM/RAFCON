from gtkmvc import Model, Observable

from awesome_server.connections.connection_manager import ConnectionManager
from awesome_server.mvc.models.udp_connection import UDPConnectionModel

from awesome_server.utils import constants
from awesome_server.utils.messaging import Message


class ConnectionManagerModel(Model, Observable):
    """
    Holds data related to connection manager. Stores all UDP clients connected to the UDPConnection.
    :param connection_manager: ConnectionManager to observe.
    """

    _udp_clients = {}
    _tcp_messages_received = {}
    _udp_messages_received = {}
    connection_manager = None

    __observables__ = ("_udp_clients", "_tcp_messages_received", "_udp_messages_received", "connection_manager")

    def __init__(self, connection_manager):
        Model.__init__(self)
        Observable.__init__(self)
        self.register_observer(self)

        assert isinstance(connection_manager, ConnectionManager)
        self.connection_manager = connection_manager

    def get_udp_connection_for_address(self, addr):
        """
        Looks for the UDPConnection holding the specified address (ip, port)
        :param addr: Address to find in UDPConnection
        :return: UDPConnection containing address, None if no connection found
        """
        for connection, addresses in self.udp_clients.iteritems():
            if addr in addresses:
                return connection
        return None

    @Model.observe("connection_manager", after=True)
    def connection_manager_method_call(self, model, prop_name, info):
        """
        Observes the connection_manager and its UDPConnections.
        Stores all received messages over UDP and TCP
        """
        if info["method_name"] == "add_udp_connection":
            self.udp_clients[info.result] = []
            self.observe_model(UDPConnectionModel(info.result))
        elif info["method_name"] == "new_udp_message_detected":
            msg = info["args"][1]
            self._udp_messages_received[msg.message_id] = msg.message
        elif info["method_name"] == "tcp_data_received":
            # TODO: connection to be replaced with unique sm_id
            connection, message = info["args"][2:]
            self._tcp_messages_received[connection] = message

    @Model.observe("_udp_clients_list", after=True)
    def new_udp_client_added(self, model, prop_name, info):
        """
        Keeps track of addresses of clients connected to all UDPConnections
        """
        self.udp_clients[info.model.udp_connection] = info.instance

    @property
    def udp_clients(self):
        return self._udp_clients

    @property
    def tcp_messages_received(self):
        return self._tcp_messages_received