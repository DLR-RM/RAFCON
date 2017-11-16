import threading

import gtkmvc
import gtkmvc.model_mt

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

original_ModelMT = gtkmvc.model_mt.ModelMT


def patch_gtkmvc_model_mt():
    print "patch"
    from gtkmvc.model_mt import support, Model, _threading, gobject

    class ModelMT(Model):
        """A base class for models whose observable properties can be
        changed by threads different than gtk main thread. Notification is
        performed by exploiting the gtk idle loop only if needed,
        otherwise the standard notification system (direct method call) is
        used. In this model, the observer is expected to run in the gtk
        main loop thread."""

        __metaclass__ = support.metaclasses.ObservablePropertyMetaMT

        def __init__(self):
            Model.__init__(self)
            self.__observer_threads = {}
            self._prop_lock = _threading.Lock()
            return

        def register_observer(self, observer):
            Model.register_observer(self, observer)
            self.__observer_threads[observer] = _threading.currentThread()
            return

        def unregister_observer(self, observer):
            Model.unregister_observer(self, observer)
            del self.__observer_threads[observer]
            return

        # ---------- Notifiers:

        def __notify_observer__(self, observer, method, *args, **kwargs):
            """This makes a call either through the gtk.idle list or a
            direct method call depending whether the caller's thread is
            different from the observer's thread"""

            assert self.__observer_threads.has_key(observer)
            if _threading.currentThread() == self.__observer_threads[observer]:
                # standard call
                print "{0} -> {1}: single threading '{2}' in call_thread {3} object_generation_thread {3} \n{4}" \
                      "".format(self.__class__.__name__, observer.__class__.__name__, method.__name__,
                                self.__observer_threads[observer], (args, kwargs))
                return Model.__notify_observer__(self, observer, method,
                                                 *args, **kwargs)

            # multi-threading call
            print "{0} -> {1}: multi threading '{2}' in call_thread {3} object_generation_thread {4} \n{5}" \
                  "".format(self.__class__.__name__, observer.__class__.__name__, method.__name__,
                            _threading.currentThread(), self.__observer_threads[observer], (args, kwargs))
            gobject.idle_add(self.__idle_callback, observer, method, args, kwargs)
            return

        def __idle_callback(self, observer, method, args, kwargs):
            method(*args, **kwargs)
            return False


        pass # end of class

    gtkmvc.model_mt.ModelMT = ModelMT
    gtkmvc.ModelMT = ModelMT


def unpatch_gtkmvc_model_mt():
    print "unpatch"
    gtkmvc.model_mt.ModelMT = original_ModelMT
    gtkmvc.ModelMT = original_ModelMT


def run_create():
    """ By searching for the WT thread number can been seen, that
    """
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine
    import rafcon.core.singleton
    import rafcon.gui.singleton

    print "WT: ", threading.currentThread()
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(menubar_ctrl.on_new_activate, None)

    print "\n"*10, "WT generated object", "\n"*5
    rafcon.core.singleton.state_machine_manager.add_state_machine(StateMachine(HierarchyState("new root state")))


def test_thread_observer_creation_list(caplog):
    patch_gtkmvc_model_mt()
    testing_utils.run_gui()

    try:
        run_create()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)
        unpatch_gtkmvc_model_mt()

if __name__ == '__main__':
    test_thread_observer_creation_list(None)
