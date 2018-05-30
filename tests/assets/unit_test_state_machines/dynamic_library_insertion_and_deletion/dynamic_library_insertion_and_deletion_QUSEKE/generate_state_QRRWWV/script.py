from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
import time
from rafcon.gui.config import global_gui_config as gui_config
import glib

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    #libary_path = "unit_test_state_machines/bake_libraries"
    #libary_name = "bake_library1"
    libary_path = inputs["library_path"]
    libary_name = inputs["library_name"]
    s = LibraryState(libary_path, libary_name, name=libary_name)
    self.parent.add_state(s)
    self.parent.add_transition(self.state_id, 0, s.state_id, None)
    self.parent.add_transition(s.state_id, 0, "VBOFVC", None)
    parent_id = None
    for d_id, d in self.parent.input_data_ports.iteritems():
        if d.name == "str_variable":
            parent_id = d_id
    self.parent.add_data_flow(self.parent.state_id, parent_id, s.state_id, 0)
    outputs["generated_state_id"] = s.state_id
    wait_for_gui()
    time.sleep(1.0)
    return 0

def execute_old(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    #libary_path = "unit_test_state_machines/bake_libraries"
    #libary_name = "bake_library1"
    libary_path = inputs["library_path"]
    libary_name = inputs["library_name"]
    s = LibraryState(libary_path, libary_name, name=libary_name)
    glib.idle_add(self.parent.add_state, s)
    wait_for_gui()
    glib.idle_add(self.parent.add_transition, self.state_id, 0, s.state_id, None)
    wait_for_gui()
    glib.idle_add(self.parent.add_transition, s.state_id, 0, "BDIKEE", None)
    wait_for_gui()
    glib.idle_add(self.parent.add_transition, s.state_id, 1, "QLRVDQ", None)
    wait_for_gui()
    parent_id = None
    for d_id, d in self.parent.input_data_ports.iteritems():
        if d.name == "grasp_width":
            parent_id = d_id
    glib.idle_add(self.parent.add_data_flow, self.parent.state_id, parent_id, s.state_id, 0)
    wait_for_gui()
    outputs["generated_state_id"] = s.state_id
    time.sleep(3.0)
    return 0
