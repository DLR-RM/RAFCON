from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
from rafcon.utils.gui_functions import call_gui_callback
import time
from rafcon.gui.config import global_gui_config as gui_config
from gi.repository import GLib

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    #libary_path = "unit_test_state_machines/bake_libraries"
    #libary_name = "bake_library1"
    libary_path = inputs["library_path"]
    libary_name = inputs["library_name"]
    s = LibraryState(libary_path, libary_name, name=libary_name, state_id="test_state")
    call_gui_callback(self.parent.add_state, s)
    wait_for_gui()
    call_gui_callback(self.parent.add_transition, self.state_id, 0, s.state_id, None)
    wait_for_gui()
    call_gui_callback(self.parent.add_transition, s.state_id, 0, "CXVBON", None)
    wait_for_gui()
    parent_id = None
    for d_id, d in self.parent.input_data_ports.items():
        if d.name == "str_variable":
            parent_id = d_id
    call_gui_callback(self.parent.add_data_flow, self.parent.state_id, parent_id, s.state_id, 0)
    wait_for_gui()
    outputs["generated_state_id"] = s.state_id
    time.sleep(1.0)
    return 0
    

# IMPORTANT!
# do not EVER create state machine elements from outside the GUI thread!
# you have to use idle_add!
# if you want to synchronize it use call_gui_callback!
def execute_old(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    #libary_path = "unit_test_state_machines/bake_libraries"
    #libary_name = "bake_library1"
    libary_path = inputs["library_path"]
    libary_name = inputs["library_name"]
    s = LibraryState(libary_path, libary_name, name=libary_name, state_id="test_state")
    self.parent.add_state(s)
    self.parent.add_transition(self.state_id, 0, s.state_id, None)
    self.parent.add_transition(s.state_id, 0, "CXVBON", None)
    parent_id = None
    for d_id, d in self.parent.input_data_ports.items():
        if d.name == "str_variable":
            parent_id = d_id
    self.parent.add_data_flow(self.parent.state_id, parent_id, s.state_id, 0)
    outputs["generated_state_id"] = s.state_id
    wait_for_gui()
    time.sleep(1.0)
    return 0

