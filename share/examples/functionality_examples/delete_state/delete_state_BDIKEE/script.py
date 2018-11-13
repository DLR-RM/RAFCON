from rafcon.core.states.library_state import LibraryState
from rafcon.gui.utils import wait_for_gui
import time
from gi.repository import GLib
import sys
from threading import Lock, Condition, Event, Thread, currentThread


def call_gui_callback(callback, *args, **kwargs):
    global exception_info, result
    from gi.repository import GLib
    condition = Condition()
    exception_info = None

    def fun():
        global exception_info, result
        result = None
        try:
            result = callback(*args)
        except:
            exception_info = sys.exc_info()
        finally:  # Finally is also executed in the case of exceptions
            condition.acquire()
            condition.notify()
            condition.release()

    if "priority" in kwargs:
        priority = kwargs["priority"]
    else:
        priority = GLib.PRIORITY_LOW

    GLib.idle_add(fun, priority=priority)
    # Wait for the condition to be notified
    condition.acquire()
    # TODO: implement timeout that raises an exception
    condition.wait()
    condition.release()
    if exception_info:
        raise exception_info[0], exception_info[1], exception_info[2]
    return result


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Delete state")
    state_id = inputs["generated_state_id"]
    # the target state is the hierarchy state, which holds this library state as child state
    target_state = self.parent.parent
    #GLib.idle_add(self.parent.remove_state, state_id)
    target_state.remove_state(state_id)
    while state_id in target_state.states.keys():
        time.sleep(0.1)
    #wait_for_gui()
    call_gui_callback(wait_for_gui)
    #time.sleep(2.0)
    return 0
