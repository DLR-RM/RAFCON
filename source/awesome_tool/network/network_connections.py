from threading import Timer

from twisted.internet import reactor
from twisted.internet.error import CannotListenError
from gtkmvc import Observer
import gobject

from awesome_tool.statemachine.storage.network_storage import NetworkStorageReader
from awesome_tool.statemachine.enums import StateExecutionState
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.states.state import State
import awesome_tool.statemachine.singleton

from awesome_tool.network.config_network import global_net_config

from awesome_tool.network.udp_connection import UDPConnection
from awesome_tool.network.tcp_connection import TCPClientFactory
from awesome_tool.network.enums import ConnectionMode

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class NetworkConnections(Observer, gobject.GObject):
    """
    This controller takes care of the network connections to the server.
    It receives and sends messages via UDP and TCP.
    :param model: Model to observe changes and send to server
    :param view: View to control network connections
    """

    def __init__(self):
        self.__gobject_init__()
        Observer.__init__(self)

        self.udp_connection_reactor_port = None
        self.tcp_connector = None

        self.udp_registered = False
        self.tcp_connected = False

        self.net_storage_reader = None

        # execution engine
        self.state_machine_execution_engine = awesome_tool.statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        # state_machine_manager
        self.state_machine_manager = awesome_tool.statemachine.singleton.state_machine_manager
        self.observe_model(self.state_machine_manager)
        self.state_machine_manager.register_observer(self)

        self.tcp_connection_factory = TCPClientFactory()
        self.tcp_connection_factory.connect('tcp_connected', self.tcp_connected_signal)
        self.tcp_connection_factory.connect('tcp_disconnected', self.tcp_disconnected_signal)

        self.udp_connection = UDPConnection(ConnectionMode.CLIENT)
        self.udp_connection.connect('udp_response_received', self.udp_response_received)
        self.udp_connection.connect('udp_no_response_received', self.udp_no_response_received)
        self.udp_connection.connect('execution_command_received', self.change_execution_mode)

        self.previous_execution_message = ""

        if global_net_config.get_config_value("AUTOCONNECT_UDP_TO_SERVER"):
            self.register_udp()
        if global_net_config.get_config_value("AUTOCONNECT_TCP_TO_SERVER"):
            self.connect_tcp()

    def tcp_connected_signal(self, tcp_conn):
        self.tcp_connected = True
        self.emit('tcp_connected')

    def tcp_disconnected_signal(self, tcp_conn):
        self.tcp_connected = False
        self.emit('tcp_disconnected')

    def udp_response_received(self, udp_conn):
        self.emit('udp_response_received')

    def udp_no_response_received(self, udp_conn):
        self.udp_registered = False
        self.udp_connection_reactor_port.stopListening()
        self.emit('udp_no_response_received')

    def set_storage_base_path(self, base_path):
        self.net_storage_reader = NetworkStorageReader(base_path)
        if self.tcp_connection_factory:
            self.tcp_connection_factory.net_storage_reader = self.net_storage_reader

    def register_udp(self):
        if not self.udp_registered:
            try:
                self.udp_connection_reactor_port = reactor.listenUDP(global_net_config.get_config_value("SELF_UDP_PORT"),
                                                                     self.udp_connection)
            except CannotListenError:
                logger.error("Cannot establish UDP connection - Port already in use")
            else:
                self.udp_registered = True
        else:
            self.emit('udp_no_response_received')
            self.udp_connection_reactor_port.stopListening()
            self.udp_registered = False
            self.reconnect_udp()

    def reconnect_udp(self):
        try:
            self.udp_connection_reactor_port = reactor.listenUDP(global_net_config.get_config_value("SELF_UDP_PORT"),
                                                                 self.udp_connection)
        except CannotListenError:
            timer = Timer(.1, self.reconnect_udp)
            timer.start()
        else:
            self.udp_registered = True

    def connect_tcp(self):
        if global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
            logger.error("No TCP connection possible in Spacebot Cup Mode")
            return
        if not self.tcp_connected and self.tcp_connector is None:
            self.tcp_connector = reactor.connectTCP(global_net_config.get_server_ip(),
                                                    global_net_config.get_server_tcp_port(),
                                                    self.tcp_connection_factory)
        elif not self.tcp_connected:
            self.tcp_connector.connect()
        else:
            self.tcp_connector.disconnect()

    def change_execution_mode(self, udp_conn, new_mode):
        """
        This method handles the change of the execution state and activates the corresponding execution mode.
        :param new_mode:
        :return:
        """
        if new_mode == 'run':
            logger.debug("Start execution engine ...")
            selected_sm_id = awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id
            awesome_tool.statemachine.singleton.state_machine_execution_engine.start(selected_sm_id)
        elif new_mode == 'stop':
            logger.debug("Stop execution engine ...")
            awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
        elif new_mode == 'pause':
            logger.debug("Pause execution engine ...")
            awesome_tool.statemachine.singleton.state_machine_execution_engine.pause()
        elif new_mode == 'step_mode':
            logger.debug("Activate execution engine step mode ...")
            awesome_tool.statemachine.singleton.state_machine_execution_engine.step_mode()
        elif new_mode == 'step_forward':
            logger.debug("Execution step ...")
            awesome_tool.statemachine.singleton.state_machine_execution_engine.step()
        elif new_mode == 'step_backward':
            logger.debug("Executing backward step ...")
            awesome_tool.statemachine.singleton.state_machine_execution_engine.backward_step()
        else:
            logger.warning("Unrecognized mode detected.")

    @Observer.observe("state_machine_manager", after=True)
    def after_add_state_machine(self, model, prop_name, info):
        if 'method_name' in info and info['method_name'] == 'add_state_machine':
            self.observe_model(info['args'][1])
            info['args'][1].register_observer(self)

    @Observer.observe("state_machine", after=True)
    def state_machine_change(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes.
        Here it is used to determine the active state change and send it to the server.

        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if 'method_name' in info and info['method_name'] == 'root_state_after_change' and self.udp_registered:
            kwargs = info['kwargs']
            if 'method_name' in kwargs and kwargs['method_name'] == 'state_change':
                kwargs = kwargs['info']
            if 'method_name' in kwargs and kwargs['method_name'] == 'state_execution_status':
                kwargs_args = kwargs['args']
                if kwargs_args[1] == StateExecutionState.ACTIVE:
                    self.send_current_active_states(model.root_state)
                    if not self.previous_execution_message.startswith("-"):
                        self.udp_connection.send_non_acknowledged_message("------------------------------------",
                                                                          (global_net_config.get_server_ip(),
                                                                           global_net_config.get_server_udp_port()),
                                                                          "ASC")  # ASC = Active State Changed
                        self.previous_execution_message = "------------------------------------"

    @Observer.observe("execution_engine", after=True)
    def execution_mode_changed(self, model, prop_name, info):
        execution_mode = str(awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        execution_mode = execution_mode.replace("EXECUTION_MODE.", "")
        if global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
            self.udp_connection.send_non_acknowledged_message(execution_mode,
                                                              (global_net_config.get_server_ip(),
                                                               global_net_config.get_server_udp_port()),
                                                              "EXE")
        else:
            self.udp_connection.send_acknowledged_message(execution_mode,
                                                          (global_net_config.get_server_ip(),
                                                           global_net_config.get_server_udp_port()),
                                                          "EXE")

    @staticmethod
    def state_has_content(state):
        if isinstance(state, ContainerState):
            return True
        return False

    def send_current_active_states(self, state):
        """
        This method recursively checks all states if they are active. If yes the path within the statemachine is sent
        to the server.
        :param state_m: StateModel to check
        """
        assert isinstance(state, State)

        if state.state_execution_status == StateExecutionState.ACTIVE:
            if not self.state_has_content(state):
                self.udp_connection.send_non_acknowledged_message(state.get_path(),
                                                                  (global_net_config.get_server_ip(),
                                                                   global_net_config.get_server_udp_port()),
                                                                  "ASC")  # ASC = Active State Changed
                self.previous_execution_message = state.get_path()

        if self.state_has_content(state):
            for child_state in state.states.itervalues():
                self.send_current_active_states(child_state)

gobject.type_register(NetworkConnections)
gobject.signal_new('tcp_connected', NetworkConnections, gobject.SIGNAL_RUN_FIRST, None, ())
gobject.signal_new('tcp_disconnected', NetworkConnections, gobject.SIGNAL_RUN_FIRST, None, ())
gobject.signal_new('udp_response_received', NetworkConnections, gobject.SIGNAL_RUN_FIRST, None, ())
gobject.signal_new('udp_no_response_received', NetworkConnections, gobject.SIGNAL_RUN_FIRST, None, ())