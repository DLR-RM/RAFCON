from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
from rafcon.utils.gui_functions import call_gui_callback
import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Insert library state")
    #libary_path = "unit_test_state_machines/bake_libraries"
    #libary_name = "bake_library1"
    libary_path = "unit_test_state_machines/dynamic_library_insertion_libraries"
    libary_name = "test_library"
    e = call_gui_callback(LibraryState, libary_path, libary_name, name="test_library_state")
    call_gui_callback(self.parent.add_state, e)
    call_gui_callback(self.parent.add_transition, self.state_id, 0, e.state_id, None)
    call_gui_callback(self.parent.add_transition, e.state_id, 0, self.parent.state_id, 0)
    wait_for_gui()
    parent_id = None
    for d_id, d in self.parent.output_data_ports.items():
        if d.name == "output_0":
            parent_id = d_id
    call_gui_callback(self.parent.add_data_flow, e.state_id, 2, self.parent.state_id, parent_id)
    wait_for_gui()
    return 0
