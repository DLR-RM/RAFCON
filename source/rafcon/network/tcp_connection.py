from twisted.internet import protocol
import gobject

from rafcon.statemachine.storage.network_storage import NetworkStorageReader
from rafcon.statemachine.singleton import state_machine_manager

from rafcon.network.enums import ConnectionMode
from rafcon.network.protobuf.yaml_transmission_pb2 import Files

from rafcon.utils import log
logger = log.get_logger(__name__)


class TCPConnection(protocol.Protocol, gobject.GObject):
    """
    This class provides the server TCP connection interface.
    :param factory: The factory which built the connection.
    """

    def __init__(self, factory, connection_mode):
        assert isinstance(connection_mode, ConnectionMode)

        self.__gobject_init__()
        self.factory = factory
        self.incoming_data_buffer = ""

        self._connection_mode = connection_mode

    def connectionMade(self):
        if self._connection_mode == ConnectionMode.SERVER:
            self.factory.num_connections += 1
        elif self._connection_mode == ConnectionMode.CLIENT:
            if not self.factory.net_storage_reader:
                logger.error("Cannot send state machine as no storage reader is available.")
                self.transport.close()
                return

            files = Files()
            sm_name = state_machine_manager.get_active_state_machine().root_state.name
            files.sm_name = sm_name
            for path, content in self.factory.net_storage_reader.file_storage.iteritems():
                my_file = files.files.add()
                my_file.file_path = str(path)
                my_file.file_content = content
            self.transport.write(files.SerializeToString() + "TRANSMISSION_END")
            self.factory.emit_connected()

    def dataReceived(self, data):
        if self._connection_mode == ConnectionMode.SERVER:
            self.incoming_data_buffer += data
            if data.endswith("TRANSMISSION_END"):
                self.factory.forward_received_data(self, self.incoming_data_buffer.split("TRANSMISSION_END")[0])
                self.incoming_data_buffer = ""

    def connectionLost(self, reason):
        if self._connection_mode == ConnectionMode.SERVER:
            self.factory.num_connections -= 1
        elif self._connection_mode == ConnectionMode.CLIENT:
            self.factory.emit_disconnected()


class TCPFactory(protocol.Factory, gobject.GObject):
    """
    This class is used to manage the TCP connections. It also keeps track of how many connections are currently
    available.
    """

    def __init__(self):
        self.__gobject_init__()
        self.num_connections = 0

        self._connection_mode = ConnectionMode.SERVER

    def buildProtocol(self, addr):
        return TCPConnection(self, self._connection_mode)

    def forward_received_data(self, connection, data):
        self.emit("data_received", connection, data)

    def get_num_connections(self):
        return self.num_connections


class TCPClientFactory(protocol.ClientFactory, gobject.GObject):

    def __init__(self):
        self.__gobject_init__()

        self._connection_mode = ConnectionMode.CLIENT

        self._net_storage_reader = None

    @property
    def net_storage_reader(self):
        return self._net_storage_reader

    @net_storage_reader.setter
    def net_storage_reader(self, net_storage_reader):
        assert isinstance(net_storage_reader, NetworkStorageReader)
        self._net_storage_reader = net_storage_reader

    def buildProtocol(self, addr):
        return TCPConnection(self, self._connection_mode)

    def emit_connected(self):
        if self._connection_mode == ConnectionMode.CLIENT:
            self.emit('tcp_connected')

    def emit_disconnected(self):
        if self._connection_mode == ConnectionMode.CLIENT:
            self.emit('tcp_disconnected')


gobject.type_register(TCPFactory)
gobject.signal_new('data_received', TCPFactory, gobject.SIGNAL_RUN_FIRST, None, (TCPConnection, gobject.TYPE_STRING))

gobject.type_register(TCPClientFactory)
gobject.signal_new('tcp_connected', TCPClientFactory, gobject.SIGNAL_RUN_FIRST, None, ())
gobject.signal_new('tcp_disconnected', TCPClientFactory, gobject.SIGNAL_RUN_FIRST, None, ())