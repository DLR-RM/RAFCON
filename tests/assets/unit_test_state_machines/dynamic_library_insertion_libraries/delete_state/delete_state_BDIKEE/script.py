import time
from gi.repository import GLib
import sys

from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
from rafcon.utils.gui_functions import call_gui_callback
from threading import Lock, Condition, Event, Thread, currentThread
from rafcon.core.decorators import lock_state_machine
from rafcon.gui.helpers.state_machine import delete_core_elements_of_models
from rafcon.gui.singleton import state_machine_manager_model


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Delete state")
    state_id = inputs["generated_state_id"]
    # the target state is the hierarchy state, which holds this library state as child state
    target_state = self.parent.parent
    
    call_gui_callback(target_state.remove_state, state_id)
    # do not call this with idle_add, otherwise the oberserver will be triggered asynchronously
    # i.e. the before notification will be handled after the whole operation already happend
    #GLib.idle_add(target_state.remove_state, state_id)
    
    while state_id in target_state.states.keys():
        time.sleep(0.1)
    
    wait_for_gui()
    
    #call_gui_callback(wait_for_gui)
    
    #time.sleep(2.0)
    return 0
