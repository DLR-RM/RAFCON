from gtkmvc import Observer
import gobject

from rafcon.statemachine.storage.network_storage import NetworkStorageReader
from rafcon.statemachine.enums import StateExecutionState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.state import State
import rafcon.statemachine.singleton

from rafcon.network.network_config import global_net_config

from rafcon.network.enums import ConnectionMode

from rafcon.utils import log
logger = log.get_logger(__name__)


class NetworkConnections(Observer, gobject.GObject):
    """
    This controller takes care of the network connections to the server.
    It receives and sends messages via UDP and TCP.
    :param model: Model to observe changes and send to server
    :param view: View to control network connections
    """

    def __init__(self, udp_net_controller, tcp_net_controller):
        self.__gobject_init__()
        Observer.__init__(self)

        self.tcp_connected = False
        self._udp_net_controller = udp_net_controller
        self._tcp_net_controller = tcp_net_controller

        # these variables will be initialized in the initialize method
        self.udp_port = None
        self.net_storage_reader = None
        self.state_machine_execution_engine = None
        self.state_machine_manager = None
        self.previous_execution_message = None

    def initialize(self):
        self.udp_port = global_net_config.get_config_value("CLIENT_UDP_PORT")
        self.net_storage_reader = None

        # execution engine
        self.state_machine_execution_engine = rafcon.statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        # state_machine_manager
        self.state_machine_manager = rafcon.statemachine.singleton.state_machine_manager
        self.observe_model(self.state_machine_manager)
        self.state_machine_manager.register_observer(self)

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
        self._udp_net_controller.stop(self.udp_port)
        self.emit('udp_no_response_received')

    @property
    def udp_registered(self):
        return self.udp_port in self._udp_net_controller.get_connections().iterkeys()

    def set_storage_base_path(self, base_path):
        self.net_storage_reader = NetworkStorageReader(base_path)
        ip, port = global_net_config.get_server_ip(), global_net_config.get_server_tcp_port()
        try:
            client_factory = self._tcp_net_controller.get_tcp_clients()[(ip, port)]
            if client_factory:
                client_factory.net_storage_reader = self.net_storage_reader
        except KeyError:
            pass

    def register_udp(self):
        if not self.udp_registered:
            print "self.udp_port: " + str(self.udp_port)
            udp_connection = self._udp_net_controller.start(self.udp_port, ConnectionMode.CLIENT)
            self.connect_udp_connection(udp_connection)
        else:
            self.emit('udp_no_response_received')
            self._udp_net_controller.stop(self.udp_port)
            self._udp_net_controller.restart(self.udp_port)

    def connect_udp_connection(self, udp_connection):
        udp_connection.connect('udp_response_received', self.udp_response_received)
        udp_connection.connect('udp_no_response_received', self.udp_no_response_received)
        udp_connection.connect('execution_command_received', self.change_execution_mode)

    def connect_tcp_connection(self, tcp_connection):
        tcp_connection.connect('tcp_connected', self.tcp_connected_signal)
        tcp_connection.connect('tcp_disconnected', self.tcp_disconnected_signal)

    def connect_tcp(self):
        if global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
            logger.error("No TCP connection possible in Spacebot Cup Mode")
            return

        ip, port = global_net_config.get_server_ip(), global_net_config.get_server_tcp_port()

        if not self.tcp_connected and (ip, port) not in self._tcp_net_controller.get_reactor_ports().iterkeys():
            self._tcp_net_controller.connect_tcp(ip, port)
            self.connect_tcp_connection(self._tcp_net_controller.get_tcp_clients()[(ip, port)])
            if self.net_storage_reader:
                self._tcp_net_controller.get_tcp_clients()[(ip, port)].net_storage_reader = self.net_storage_reader
        elif not self.tcp_connected:
            self._tcp_net_controller.get_reactor_ports()[(ip, port)].connect()
        else:
            self._tcp_net_controller.get_reactor_ports()[(ip, port)].disconnect()

    def change_execution_mode(self, udp_conn, new_mode):
        """
        This method handles the change of the execution state and activates the corresponding execution mode.
        :param new_mode:
        :return:
        """
        if new_mode == 'run':
            logger.debug("Start execution engine ...")
            selected_sm_id = rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id
            rafcon.statemachine.singleton.state_machine_execution_engine.start(selected_sm_id)
        elif new_mode == 'stop':
            logger.debug("Stop execution engine ...")
            rafcon.statemachine.singleton.state_machine_execution_engine.stop()
        elif new_mode == 'pause':
            logger.debug("Pause execution engine ...")
            rafcon.statemachine.singleton.state_machine_execution_engine.pause()
        elif new_mode == 'step_mode':
            logger.debug("Activate execution engine step mode ...")
            rafcon.statemachine.singleton.state_machine_execution_engine.step_mode()
        elif new_mode == 'step_forward':
            logger.debug("Execution step ...")
            rafcon.statemachine.singleton.state_machine_execution_engine.step()
        elif new_mode == 'step_backward':
            logger.debug("Executing backward step ...")
            rafcon.statemachine.singleton.state_machine_execution_engine.backward_step()
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
                        udp_connection = self._udp_net_controller.get_connections()[self.udp_port]
                        udp_connection.send_non_acknowledged_message("------------------------------------",
                                                                     (global_net_config.get_server_ip(),
                                                                      global_net_config.get_server_udp_port()),
                                                                      "ASC")  # ASC = Active State Changed
                        self.previous_execution_message = "------------------------------------"

    @Observer.observe("execution_engine", after=True)
    def execution_mode_changed(self, model, prop_name, info):
        execution_mode = str(rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        execution_mode = execution_mode.replace("EXECUTION_MODE.", "")
        if self.udp_port in self._udp_net_controller.get_connections():
            udp_connection = self._udp_net_controller.get_connections()[self.udp_port]
            if global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
                udp_connection.send_non_acknowledged_message(execution_mode,
                                                             (global_net_config.get_server_ip(),
                                                              global_net_config.get_server_udp_port()),
                                                             "EXE")
            else:
                udp_connection.send_acknowledged_message(execution_mode,
                                                         (global_net_config.get_server_ip(),
                                                          global_net_config.get_server_udp_port()),
                                                         "EXE")
        else:
            logger.warning("No udp-connection for port %d found" % self.udp_port)

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
                udp_connection = self._udp_net_controller.get_connections()[self.udp_port]
                udp_connection.send_non_acknowledged_message(state.get_path(),
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