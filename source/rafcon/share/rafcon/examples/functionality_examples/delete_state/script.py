from rafcon.gui.utils import wait_for_gui
from rafcon.utils.gui_functions import call_gui_callback
import time


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Delete state")
    state_id = inputs["generated_state_id"]
    # the target state is the hierarchy state, which holds this library state as child state
    target_state = self.parent.parent
    target_state.remove_state(state_id)
    while state_id in target_state.states.keys():
        time.sleep(0.1)
    call_gui_callback(wait_for_gui)
    return 0
