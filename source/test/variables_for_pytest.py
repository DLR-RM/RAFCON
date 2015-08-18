from threading import Lock, Condition

test_multithrading_lock = Lock()


def call_gui_callback(callback, *args):
    """Wrapper method for glib.idle_add

    This method is intended as replacement for idle_add. It wraps the method with a callback option. The advantage is
    that this way, the call is blocking. The method return, when the callback method has been called and executed.

    :param callback: The callback method, e.g. on_open_activate
    :param args: The parameters to be passed to the callback method
    """
    import glib
    condition = Condition()

    def fun():
        """Call callback and notify condition variable
        """
        callback(*args)
        condition.acquire()
        condition.notify()
        condition.release()

    glib.idle_add(fun)
    # Wait for the condition to be notified
    condition.acquire()
    # TODO: implement timeout that raises an exception
    condition.wait()
    condition.release()

sm_manager_model = None