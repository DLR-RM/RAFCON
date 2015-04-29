from gtkmvc import Model, Observable

from awesome_server.connections.connection_manager import ConnectionManager
from awesome_server.mvc.models.udp_connection import UDPConnectionModel


class ConnectionManagerModel(Model, Observable):

    _udp_clients = {}
    connection_manager = None

    __observables__ = ("_udp_clients", "connection_manager")

    def __init__(self, connection_manager):
        Model.__init__(self)
        Observable.__init__(self)
        self.register_observer(self)

        assert isinstance(connection_manager, ConnectionManager)
        self.connection_manager = connection_manager

    def get_udp_connection_for_address(self, addr):
        for client, addresses in self.udp_clients.iteritems():
            if addr in addresses:
                return client

    @Model.observe("connection_manager", after=True)
    def test(self, model, prop_name, info):
        if info["method_name"] == "add_udp_connection":
            self.udp_clients[info.result] = []
            self.observe_model(UDPConnectionModel(info.result))

    @Model.observe("_udp_clients_list", after=True)
    def test2(self, model, prop_name, info):
        self.udp_clients[info.model.udp_connection] = info.instance

    @property
    def udp_clients(self):
        return self._udp_clients