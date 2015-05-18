from awesome_server.mvc.controller.extended_controller import ExtendedController
from awesome_server.mvc.models.connection_manager import ConnectionManagerModel

from awesome_server.connections.protobuf import yaml_transmission_pb2
from awesome_server.utils.storage_utils import StorageUtils

from awesome_server.statemachine.states.hierarchy_state import HierarchyState
from awesome_server.statemachine.states.execution_state import ExecutionState
from awesome_server.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from awesome_server.statemachine.states.container_state import ContainerState

from awesome_server.utils import vividict
from awesome_server.utils import constants

import gtk
from twisted.internet import reactor

from awesome_server.utils.config import global_server_config
from awesome_server.statemachine.singleton import global_storage
import os


TEMP_FOLDER = os.getenv("HOME") + "/.awesome_server/tmp"

command_mapping = {
    'Start': 'run',
    'Pause': 'pause',
    'Stop': 'stop',
    'Step Mode': 'step_mode',
    'Step ->': 'step_forward',
    'Step <-': 'step_backward'
}


class DebugViewController(ExtendedController):

    def __init__(self, model, view):
        assert isinstance(model, ConnectionManagerModel)
        ExtendedController.__init__(self, model, view)

        model.connection_manager.add_tcp_connection(global_server_config.get_config_value("TCP_PORT"))
        model.connection_manager.add_udp_connection(global_server_config.get_config_value("UDP_PORT"))
        model.connection_manager.server_html.connect("command_received", self.handle_command)

        view["send_button"].connect("clicked", self.send_button_clicked)
        view["send_ack_button"].connect("clicked", self.send_ack_button_clicked)

        self.state_machine = None
        self.gui_meta = None
        self.storage = StorageUtils("~/")
        if not self.storage.exists_path(TEMP_FOLDER):
            self.storage.create_path(TEMP_FOLDER)

    def handle_command(self, server_html, command, ip, port):
        if command in ('run', 'pause', 'stop', 'step_mode', 'step_forward', 'step_backward'):
            self.send_command(command, (ip, port))
        elif command == 'reload_sm':
            root_state = self.state_machine.root_state
            root_state_id = root_state.state_id
            self.send_statemachine_to_browser(self.gui_meta, root_state_id, root_state, 1)
            self.send_statemachine_connections_to_browser(self.gui_meta, root_state_id, root_state, 1)

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
        self.storage.remove_path(TEMP_FOLDER)
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

    def on_send_state_to_browser_button_clicked(self, widget, event=None):
        self.model.connection_manager.server_html.send_state_data("none", "my_state_id", "Test_State",
                                                                  500, 20, 250, 250)

    def on_send_nested_state_to_browser_button_clicked(self, widget, event=None):
        self.model.connection_manager.server_html.send_state_data("my_state_id", "my_nested_state_id", "Nested",
                                                                  75, 60, 100, 100)

    @ExtendedController.observe("_udp_clients", after=True)
    def add_udp_client(self, model, prop_name, info):
        if isinstance(info.args[1], dict):
            new_entry = info.args[1].iterkeys().next()
            clients = info.instance[info.args[0]]
            if self.view:
                ip, port = clients[new_entry]
                self.view["liststore"].append(["%s:%s" % (new_entry, ip), ip, port])
                self.view["combobox"].set_active(len(self.view["liststore"]) - 1)

    @ExtendedController.observe("_tcp_messages_received", after=True)
    def handle_tcp_message_received(self, model, prop_name, info):
        msg = info["args"][1]
        files = yaml_transmission_pb2.Files()
        files.ParseFromString(msg)
        self.process_yaml_files(files.files)
        tmp_sm_path = str(TEMP_FOLDER + "/" + files.files[0].file_path.rsplit('/')[1])
        [state_machine, version, creation_time] = global_storage.load_statemachine_from_yaml(tmp_sm_path)
        self.state_machine = state_machine

        self.gui_meta = global_storage.load_graphical_meta_data_from_yaml(tmp_sm_path)
        root_state_id = state_machine.root_state.state_id

        self.send_statemachine_to_browser(self.gui_meta, root_state_id, state_machine.root_state, 1)
        self.send_statemachine_connections_to_browser(self.gui_meta, root_state_id, state_machine.root_state, 1)

    def send_statemachine_to_browser(self, gui_meta, state_id, state, hierarchy_level):
        rel_pos = gui_meta[state_id][2]["gui"]["editor"]["rel_pos"]
        if isinstance(rel_pos, dict):
            rel_pos = (5, 5)
        size = gui_meta[state_id][2]["gui"]["editor"]["size"]
        self.model.connection_manager.server_html.send_state_data(gui_meta[state_id][0],
                                                                  state_id,
                                                                  gui_meta[state_id][1].name,
                                                                  abs(rel_pos[0] * constants.BROWSER_SIZE_MULTIPLIER),
                                                                  abs(rel_pos[1] * constants.BROWSER_SIZE_MULTIPLIER),
                                                                  size[0] * constants.BROWSER_SIZE_MULTIPLIER,
                                                                  size[1] * constants.BROWSER_SIZE_MULTIPLIER,
                                                                  hierarchy_level,
                                                                  gui_meta[state_id][1].outcomes)
        if isinstance(state, ContainerState):
            for child_id, child in state.states.iteritems():
                self.send_statemachine_to_browser(gui_meta, child_id, child, hierarchy_level + 1)

    def send_statemachine_connections_to_browser(self, gui_meta, state_id, state, hierarchy_level):
        if isinstance(state, ContainerState):
            transitions = gui_meta[state_id][1].transitions
            for transition_id, transition in transitions.iteritems():
                transition_waypoints = gui_meta[state_id][2]["transition%d" % transition_id]["gui"]["editor"]["waypoints"]
                if len(transition_waypoints) == 0:
                    transition_waypoints = None
                if transition.from_state:
                    from_state = transition.from_state
                    from_outcome = gui_meta[transition.from_state][1].outcomes[transition.from_outcome].name
                else:
                    from_state = state_id
                    from_outcome = ""
                if not transition.to_state:
                    to_outcome = gui_meta[state_id][1].outcomes[transition.to_outcome].name
                    to_state = state_id
                else:
                    to_outcome = ""
                    to_state = transition.to_state

                self.model.connection_manager.server_html.send_connection_data(state_id,
                                                                               from_outcome,
                                                                               from_state,
                                                                               to_outcome,
                                                                               to_state,
                                                                               transition_waypoints)

            for child_id, child in state.states.iteritems():
                self.send_statemachine_connections_to_browser(gui_meta, child_id, child, hierarchy_level + 1)


    @ExtendedController.observe("_udp_messages_received", after=True)
    def handle_udp_message_received(self, mode, prop_name, info):
        message = info["args"][1]
        self.print_msg(message)

    def process_yaml_files(self, files):
        for f in files:
            dirname = os.path.dirname(TEMP_FOLDER + f.file_path)
            if not self.storage.exists_path(dirname):
                self.storage.create_path(dirname)
            if not self.storage.exists_path(TEMP_FOLDER + f.file_path):
                new_file = open(TEMP_FOLDER + f.file_path, 'wb')
                new_file.write(f.file_content)
                new_file.close()

    def print_msg(self, msg):
        buf = self.view["messages"].get_buffer()
        buf.insert(buf.get_end_iter(), msg + "\n")
        self.view["messages"].scroll_mark_onscreen(buf.get_insert())