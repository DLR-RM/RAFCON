import os

from rafcon.network.protobuf import yaml_transmission_pb2
from rafcon.network.network_config import global_net_config
import gtk
from twisted.internet import reactor

from rafcon_server.mvc.models.connection_manager import ConnectionManagerModel
from rafcon.utils import filesystem
from rafcon.utils import constants
from rafcon.utils import storage_utils
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine import interface
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.singleton import global_storage, state_machine_execution_engine, state_machine_manager
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log
logger = log.get_logger(__name__)


TEMP_FOLDER = os.getenv("HOME") + "/.rafcon_server/tmp"

command_mapping = {
    'Start': 'run',
    'Pause': 'pause',
    'Stop': 'stop',
    'Step Mode': 'step_mode',
    'Step ->': 'step_forward',
    'Step <-': 'step_backward'
}


class DebugViewController(ExtendedController):

    def __init__(self, model, view, multiprocessing_queue=None):
        assert isinstance(model, ConnectionManagerModel)
        ExtendedController.__init__(self, model, view)

        model.connection_manager.add_tcp_connection(global_net_config.get_config_value("SERVER_TCP_PORT"))
        model.connection_manager.add_udp_connection(global_net_config.get_config_value("SERVER_UDP_PORT"))
        model.connection_manager.server_html.connect("command_received", self.handle_command)

        view["send_button"].connect("clicked", self.send_button_clicked)
        view["send_ack_button"].connect("clicked", self.send_ack_button_clicked)
        view["load_statemachine_button"].connect("clicked", self.load_statemachine_from_folder)

        self.last_active_state_message = ""

        if not os.path.exists(TEMP_FOLDER):
            filesystem.create_path(TEMP_FOLDER)
        self.multiprocessing_queue = multiprocessing_queue

    @property
    def active_state_machine(self):
        return state_machine_manager.get_active_state_machine()

    def handle_command(self, server_html, command, sm_name, ip, port):
        if command in ('run', 'pause', 'stop', 'step_mode', 'step_forward', 'step_backward'):
            self.send_command(command, (ip, port))
        elif command == 'reload_sm' and self.active_state_machine:
            root_state = None
            for sm in state_machine_manager.state_machines.itervalues():
                if sm.root_state.name == sm_name:
                    root_state = sm.root_state
                    break

            if root_state:
                root_state_id = root_state.state_id
                sm_name = root_state.name
                self.send_statemachine_to_browser(sm_name, root_state_id, root_state, 1)
                self.send_statemachine_connections_to_browser(sm_name, root_state_id, root_state, 1)
                self.model.connection_manager.server_html.send_sm_transmission_end()
        elif command == 'resend_active_states' and \
                        state_machine_execution_engine.status.execution_mode != StateMachineExecutionStatus.STARTED and \
                        state_machine_execution_engine.status.execution_mode != StateMachineExecutionStatus.STOPPED:
            root_state = None
            for sm in state_machine_manager.state_machines.itervalues():
                if sm.root_state.name == sm_name:
                    root_state = sm.root_state
                    break

            if root_state:
                sm_name = root_state.name
                self.model.connection_manager.server_html.send_data(self.last_active_state_message, "none", 0, "ASC", sm_name)
                self.model.connection_manager.server_html.send_data("-", "none", 0, "ASC", sm_name)

    def on_command_button_clicked(self, widget, data=None):
        command = command_mapping[widget.get_label()]
        combo = self.view["combobox"]
        tree_iter = combo.get_active_iter()
        if tree_iter is not None and command:
            model = combo.get_model()
            name, ip, port = model[tree_iter]

            self.send_command(command, (ip, port))

    def send_command(self, command, addr):
        con = self.model.get_udp_connection_for_address(addr)
        if con:
            con.send_non_acknowledged_message(command, addr, "EXE")

    def send_button_clicked(self, widget, event=None):
        self.send_message_to_selected_connection(self.view["entry"].get_text(), False)

    def send_ack_button_clicked(self, widget, event=None):
        self.send_message_to_selected_connection(self.view["entry"].get_text(), True)

    def on_window_destroy(self, widget, event=None):
        filesystem.remove_path(TEMP_FOLDER)
        reactor.stop()
        gtk.main_quit()

    def send_message_to_selected_connection(self, message, ack):
        combo = self.view["combobox"]
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            name, ip, port = model[tree_iter]

            con = self.model.get_udp_connection_for_address((ip, port))
            if con:
                if ack:
                    con.send_acknowledged_message(message, (ip, port))
                else:
                    con.send_non_acknowledged_message(message, (ip, port))

    @ExtendedController.observe("_new_udp_client_signal", signal=True)
    def new_udp_client_added(self, model, prop_name, info):
        if self.view:
            new_entry, ip, port = info["arg"]
            self.view["liststore"].append(["%s - %s:%d" % (new_entry, ip, port), ip, port])
            self.view["combobox"].set_active(len(self.view["liststore"]) - 1)

    @ExtendedController.observe("_tcp_messages_received", after=True)
    def handle_tcp_message_received(self, model, prop_name, info):
        msg = info["args"][1]
        files = yaml_transmission_pb2.Files()
        files.ParseFromString(msg)
        self.process_yaml_files(files.files)
        tmp_sm_path = str(TEMP_FOLDER + "/" + files.files[0].file_path.rsplit('/')[1])
        [state_machine, version, creation_time] = global_storage.load_statemachine_from_yaml(tmp_sm_path)
        if not state_machine.state_machine_id in state_machine_manager.state_machines.keys():
            state_machine_manager.add_state_machine(state_machine)
        state_machine_manager.active_state_machine_id = state_machine.state_machine_id

        root_state_id = state_machine.root_state.state_id
        sm_name = state_machine.root_state.name
        self.send_statemachine_to_browser(sm_name, root_state_id, state_machine.root_state, 1)
        self.send_statemachine_connections_to_browser(sm_name, root_state_id, state_machine.root_state, 1)
        self.model.connection_manager.server_html.send_sm_transmission_end()

    def load_statemachine_from_folder(self, widget=None, data=None, path=None):
        if path is None:
            if interface.open_folder_func is None:
                logger.error("No function defined for opening a folder")
                return
            load_path = interface.open_folder_func("Please choose the folder of the state-machine")
            if load_path is None:
                return
        else:
            load_path = path

        try:
            [state_machine, version, creation_time] = global_storage.load_statemachine_from_yaml(load_path)
            state_machine_manager.add_state_machine(state_machine)

            root_state_id = state_machine.root_state.state_id
            sm_name = state_machine.root_state.name
            self.send_statemachine_to_browser(sm_name, root_state_id, state_machine.root_state, 1)
            self.send_statemachine_connections_to_browser(sm_name, root_state_id, state_machine.root_state, 1)
            self.model.connection_manager.server_html.send_sm_transmission_end()
        except AttributeError as e:
            logger.error('Error while trying to open state-machine: {0}'.format(e))

    def send_statemachine_to_browser(self, sm_name, state_id, state, hierarchy_level):
        meta_path = os.path.join(state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE_YAML)
        tmp_meta = None
        if os.path.exists(meta_path):
            tmp_meta = storage_utils.load_dict_from_yaml(meta_path)

        if not tmp_meta:
            raise AttributeError("Meta data could not be loaded from %s" % meta_path)

        rel_pos = tmp_meta["gui"]["editor_opengl"]["rel_pos"]
        if isinstance(rel_pos, dict):
            rel_pos = (5, 5)
        size = tmp_meta["gui"]["editor_opengl"]["size"]

        parent_id = "none"
        if state.parent and isinstance(state.parent, ContainerState):
            parent_id = state.parent.state_id

        self.model.connection_manager.server_html.send_state_data(sm_name,
                                                                  parent_id,
                                                                  state_id,
                                                                  state.name,
                                                                  abs(rel_pos[0] * constants.BROWSER_SIZE_MULTIPLIER),
                                                                  abs(rel_pos[1] * constants.BROWSER_SIZE_MULTIPLIER),
                                                                  size[0] * constants.BROWSER_SIZE_MULTIPLIER,
                                                                  size[1] * constants.BROWSER_SIZE_MULTIPLIER,
                                                                  hierarchy_level,
                                                                  state.outcomes)
        if isinstance(state, ContainerState):
            for child_id, child in state.states.iteritems():
                self.send_statemachine_to_browser(sm_name, child_id, child, hierarchy_level + 1)

    def send_statemachine_connections_to_browser(self, sm_name, state_id, state, hierarchy_level):
        if isinstance(state, ContainerState):
            transitions = state.transitions

            meta_path = os.path.join(state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE)
            tmp_meta = None
            if os.path.exists(meta_path):
                tmp_meta = storage_utils.load_dict_from_yaml(meta_path)

            if not tmp_meta:
                raise AttributeError("Meta data could not be loaded from %s" % meta_path)

            for transition_id, transition in transitions.iteritems():
                transition_waypoints = tmp_meta["transition%d" % transition_id]["gui"]["editor_opengl"]["waypoints"]
                if len(transition_waypoints) == 0:
                    transition_waypoints = None
                if transition.from_state:
                    from_state = transition.from_state
                    from_outcome = None
                    for child_state in state.states.itervalues():
                        if child_state.state_id == from_state:
                            from_outcome = child_state.outcomes[transition.from_outcome].name
                else:
                    from_state = state_id
                    from_outcome = ""
                if transition.to_outcome is not None:
                    to_outcome = state.outcomes[transition.to_outcome].name
                else:
                    to_outcome = ""
                to_state = transition.to_state

                if from_outcome is None:
                    raise AttributeError("From outcome needs to be set")

                self.model.connection_manager.server_html.send_connection_data(sm_name,
                                                                               state_id,
                                                                               from_outcome,
                                                                               from_state,
                                                                               to_outcome,
                                                                               to_state,
                                                                               transition_waypoints)

            for child_id, child in state.states.iteritems():
                self.send_statemachine_connections_to_browser(sm_name, child_id, child, hierarchy_level + 1)

    @ExtendedController.observe("_udp_messages_received", after=True)
    def handle_udp_message_received(self, mode, prop_name, info):
        msg = info["args"][1]

        if msg.flag == "REG":
            message = "registered"
        else:
            message = msg.message

        if not message.startswith('-'):
            self.last_active_state_message = message

        self.print_msg("%s: %s" % (msg.sm_name, message))

        # this is for unit testing purpose
        if self.multiprocessing_queue:
            self.multiprocessing_queue.put("%s: %s" % (msg.sm_name, message))

    def process_yaml_files(self, files):
        for f in files:
            dirname = os.path.dirname(TEMP_FOLDER + f.file_path)
            if not os.path.exists(dirname):
                filesystem.create_path(dirname)
            if not os.path.exists(TEMP_FOLDER + f.file_path):
                new_file = open(TEMP_FOLDER + f.file_path, 'wb')
                new_file.write(f.file_content)
                new_file.close()

    def print_msg(self, msg):
        buf = self.view["messages"].get_buffer()
        buf.insert(buf.get_end_iter(), msg + "\n")
        self.view["messages"].scroll_mark_onscreen(buf.get_insert())