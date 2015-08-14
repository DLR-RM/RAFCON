from threading import Lock, Condition
import glib

test_multithrading_lock = Lock()

condition = Condition()
def call_gui_callback(callback, *args):
    def fun():
        callback(*args)
        condition.acquire()
        condition.notify()
        condition.release()
    glib.idle_add(fun)
    condition.acquire()
    condition.wait()
    condition.release()

sm_manager_model = None