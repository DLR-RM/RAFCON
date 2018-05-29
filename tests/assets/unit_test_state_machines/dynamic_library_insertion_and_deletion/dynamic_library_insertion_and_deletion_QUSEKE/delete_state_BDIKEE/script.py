from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
import time
import glib


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Delete state")
    state_id = inputs["generated_state_id"]
    glib.idle_add(self.parent.remove_state, state_id)
    #self.parent.remove_state(state_id)
    while state_id in self.parent.states.keys():
        time.sleep(0.1)
    wait_for_gui()
    #time.sleep(2.0)
    return 0
