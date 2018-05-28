from rafcon.core.states.library_state import LibraryState
import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    libary_path = "unit_test_state_machines/bake_libraries"
    libary_name = "bake_library1"
    e = LibraryState(libary_path, libary_name, name="test_libary_state")
    self.parent.add_state(e)
    self.parent.add_transition(self.state_id, 0, e.state_id, None)
    self.parent.add_transition(e.state_id, 0, self.parent.state_id, 0)
    time.sleep(0.1)
    return 0
