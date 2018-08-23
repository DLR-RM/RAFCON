import copy
import signal
import sys
import tempfile
from os import mkdir, environ
from os.path import join, dirname, realpath, exists, abspath
from threading import Lock, Condition, Event, Thread, currentThread

import rafcon
from rafcon.utils import log, constants

test_multithreading_lock = Lock()

gui_thread = None
gui_ready = None
gui_executed_once = False
exception_info = None
result = None


RAFCON_TEMP_PATH_TEST_BASE = join(constants.RAFCON_TEMP_PATH_BASE, 'unit_tests')
if not exists(RAFCON_TEMP_PATH_TEST_BASE):
    mkdir(RAFCON_TEMP_PATH_TEST_BASE)

# temporary path that can be used if multiple instance of RAFCON should use one reference-path in a test
RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE = join(constants.RAFCON_TEMP_PATH_BASE, '..', 'unit_tests')
if not exists(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE):
    mkdir(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE)

TESTS_PATH = dirname(abspath(__file__))
RAFCON_PATH = realpath(rafcon.__path__[0])
LIBRARY_SM_PATH = join(TESTS_PATH, '..', 'share', 'libraries')
EXAMPLES_PATH = join(TESTS_PATH, '..', 'share', 'examples')
TEST_ASSETS_PATH = join(TESTS_PATH, 'assets')
TEST_SCRIPT_PATH = join(TESTS_PATH, 'assets', 'scripts')
TUTORIAL_PATH = join(TESTS_PATH, "..", "share", "examples", "tutorials")
RAFCON_SHARED_LIBRARY_PATH = join(dirname(RAFCON_PATH), '..', 'share', 'libraries')
print LIBRARY_SM_PATH
print RAFCON_SHARED_LIBRARY_PATH

# from rafcon.core.config import global_config
# global_config.load(path=join(TESTS_PATH, "assets", "configs", "valid_config"))
GUI_INITIALIZED = False
GUI_SIGNAL_INITIALIZED = False


def get_unique_temp_path():
    return tempfile.mkdtemp(dir=RAFCON_TEMP_PATH_TEST_BASE)


def get_test_sm_path(state_machine_name):
    return join(TEST_ASSETS_PATH, state_machine_name)


def reload_config(config=True, gui_config=True):
    import rafcon
    if config:
        rafcon.core.config.global_config.load()
    if gui_config:
        import rafcon.gui.config
        rafcon.gui.config.global_gui_config.load()


def remove_all_libraries(init_library_manager=True):
    from rafcon.core.config import global_config
    library_paths = global_config.get_config_value("LIBRARY_PATHS")
    libs = [lib for lib in library_paths]
    for lib in libs:
        del library_paths[lib]
    if init_library_manager:
        rafcon.core.singleton.library_manager.initialize()


def remove_all_gvm_variables():
    from rafcon.core.singleton import global_variable_manager
    for gv_name in global_variable_manager.get_all_keys():
        global_variable_manager.delete_variable(gv_name)


def assert_logger_warnings_and_errors(caplog, expected_warnings=0, expected_errors=0):
    if caplog is None:
        return
    import logging
    counted_warnings = 0
    counted_errors = 0
    records = caplog.records if isinstance(caplog.records, list) else caplog.records()
    for record in records:
        if record.levelno == logging.WARNING:
            counted_warnings += 1
        elif record.levelno == logging.ERROR:
            counted_errors += 1
        # the exception info dict can hold references core and model objects
        # -> so the caplog does not allow gc to collect them
        if hasattr(record, 'exc_info'):
            record.exc_info = None

    print "counted_warnings == expected_warnings", counted_warnings, expected_warnings
    assert counted_warnings == expected_warnings
    print "counted_errors == expected_errors", counted_errors, expected_errors
    assert counted_errors == expected_errors


def call_gui_callback(callback, *args, **kwargs):
    """Wrapper method for glib.idle_add

    This method is intended as replacement for idle_add. It wraps the method with a callback option. The advantage is
    that this way, the call is blocking. The method return, when the callback method has been called and executed.

    :param callback: The callback method, e.g. on_open_activate
    :param args: The parameters to be passed to the callback method
    """
    global exception_info, result
    import glib
    condition = Condition()
    exception_info = None

    @log.log_exceptions()
    def fun():
        """Call callback and notify condition variable
        """
        global exception_info, result
        result = None
        try:
            result = callback(*args)
        except:
            # Exception within this asynchronously called function won't reach pytest. This is why we have to store
            # the information about the exception to re-raise it at the end of the synchronous call.
            exception_info = sys.exc_info()
        finally:  # Finally is also executed in the case of exceptions
            condition.acquire()
            condition.notify()
            condition.release()

    if "priority" in kwargs:
        priority = kwargs["priority"]
    else:
        priority = glib.PRIORITY_LOW

    glib.idle_add(fun, priority=priority)
    # Wait for the condition to be notified
    condition.acquire()
    # TODO: implement timeout that raises an exception
    condition.wait()
    condition.release()
    if exception_info:
        raise exception_info[0], exception_info[1], exception_info[2]
    return result


def rewind_and_set_libraries(libraries=None):
    """ Clear libraries, set new libraries and secure default libraries set."""
    from rafcon.core.config import global_config
    import rafcon.core.singleton
    if libraries is None:
        libraries = {}
    remove_all_libraries(init_library_manager=False)
    if not isinstance(libraries, dict):
        libraries = {}
    if "generic" not in libraries:
        libraries["generic"] = join(RAFCON_SHARED_LIBRARY_PATH, 'generic')
    global_config.set_config_value("LIBRARY_PATHS", libraries)
    environ['RAFCON_LIB_PATH'] = RAFCON_SHARED_LIBRARY_PATH
    rafcon.core.singleton.library_manager.initialize()


def initialize_signal_handler():
    # IMPORTANT avoid second signal initialization to indicate bad bad testing_utils usage
    # -> TODO cleanup with app-class creation
    global GUI_SIGNAL_INITIALIZED
    if GUI_SIGNAL_INITIALIZED:
        raise DeprecationWarning("Second signal handler initialization before call of shut_down_environment.")
    from rafcon.gui.start import signal_handler
    signal.signal(signal.SIGINT, signal_handler)
    GUI_SIGNAL_INITIALIZED = True


def initialize_environment(core_config=None, gui_config=None, runtime_config=None, libraries=None,
                           gui_already_started=True):
    """ Initialize global configs, libraries and acquire multi threading lock

     The function accepts tuples as arguments to load a config with (config-file, path) as tuple or a
     dictionary that sets partly or all parameters of the config dictionary.
     If the libraries dict is None the libraries set handed by the core-config tuple and respective LIBRARY_PATHS
     is used as libraries dictionary.

    :param core_config: Tuple pointing to config-file or dictionary for partly or all parameters of respective config.
    :param gui_config: Tuple pointing to config-file or dictionary for partly or all parameters of respective config.
    :param runtime_config: Tuple pointing to config-file or dictionary for partly or all parameters of
                           respective config.
    :param libraries: Dictionary with library mounting labels and hard drive paths.
    :return:
    """
    if gui_already_started:
        # gui callback needed as all state machine from former tests are deleted in initialize_environment_core
        call_gui_callback(initialize_environment_core, core_config, libraries, True)
    else:
        initialize_environment_core(core_config, libraries)
    initialize_environment_gui(gui_config, runtime_config)
    initialize_signal_handler()


def initialize_environment_core(core_config=None, libraries=None, delete=False):
    from rafcon.core.config import global_config
    import rafcon.core.singleton

    if rafcon.core.singleton.state_machine_manager.state_machines:
        raise EnvironmentError("The environment has to have an empty StateMachineManager but here the following "
                               "state machines are still existing: \n{0}"
                               "".format(rafcon.core.singleton.state_machine_manager.state_machines))

    test_multithreading_lock.acquire()

    # preserve LIBRARY_PATHS if handed with dict -> can be already be the dict of the global_config object
    if libraries is None and core_config is not None and 'LIBRARY_PATHS' in core_config:
        libraries = copy.deepcopy(core_config['LIBRARY_PATHS'])

    # initialize global core config
    if isinstance(core_config, tuple) and exists(join(core_config[1], core_config[0])):
        global_config.load(core_config[0], core_config[1])
        if global_config.get_config_value('LIBRARY_PATHS') is not None:
            libraries = copy.deepcopy(global_config.get_config_value('LIBRARY_PATHS'))
    else:
        global_config.load()
        if isinstance(core_config, dict):
            for key, value in core_config.iteritems():
                global_config.set_config_value(key, value)

    rewind_and_set_libraries(libraries=libraries)

    # delete_all_state_machines must not be called here per default
    # as old state machines might still be patched with PatchedModelMT
    # if initialize_environment_core is called in an un-patched manner, than this will result in a MultiThreading Error
    # state_machine_manager.delete_all_state_machines()


def initialize_environment_gui(gui_config=None, runtime_config=None):

    from rafcon.gui.config import global_gui_config
    from rafcon.gui.runtime_config import global_runtime_config
    global GUI_INITIALIZED

    if GUI_INITIALIZED:
        raise DeprecationWarning("Deprecated use of environment initialization. The gui was already initialized.")

    # initialize global gui config
    if isinstance(gui_config, tuple) and exists(join(gui_config[1], gui_config[0])):
        global_gui_config.load(gui_config[1], gui_config[0])
    else:
        global_gui_config.load()
        if isinstance(gui_config, dict):
            for key, value in gui_config.iteritems():
                global_gui_config.set_config_value(key, value)

    # initialize global runtime config
    if isinstance(runtime_config, tuple) and exists(join(runtime_config[1], runtime_config[0])):
        global_runtime_config.load(runtime_config[1], runtime_config[0])
    else:
        global_runtime_config.load()
        if isinstance(runtime_config, dict):
            for key, value in runtime_config.iteritems():
                global_runtime_config.set_config_value(key, value)

    GUI_INITIALIZED = True


def shutdown_environment(config=True, gui_config=True, caplog=None, expected_warnings=0, expected_errors=0,
                         unpatch_threading=True):
    """ Reset Config object classes of singletons and release multi threading lock and optional do the log-msg test

     The function reloads the default config files optional and release the multi threading lock. This function is
     intended to be the counterpart of the initialize_environment function so that the common environment test with
     can be shutdown/recovered as easy as it was initialized before.
     Therefore (recovering/reload default configs) it helps to avoid site effects with not properly initialized test
     runs, too.
     As long as the test stuck when one test is not releasing its multi-threading lock the function integrates a
     optional caplog error and warning message count and check. This raise the Assertion error reliable and release
     the lock, too.

    :param bool config: Flag to reload core config from default path.
    :param bool gui_config: Flag to reload gui config from default path.
    :return:
    """
    import rafcon.core.singleton
    global GUI_INITIALIZED, GUI_SIGNAL_INITIALIZED
    global gui_thread, gui_ready, used_gui_threads
    e = None
    try:
        if caplog is not None:
            assert_logger_warnings_and_errors(caplog, expected_warnings, expected_errors)
    finally:
        try:
            if gui_ready is None:  # gui was not initialized fully only the environment
                rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
            # check that state machine manager is empty
            assert not rafcon.core.singleton.state_machine_manager.state_machines
            if gui_ready:
                assert not rafcon.gui.singleton.state_machine_manager_model.state_machines
        except Exception as e:
            pass
        finally:
            rewind_and_set_libraries()
            reload_config(config, gui_config)
            wait_for_gui()  # is needed to empty the idle add queue and not party destroy elements in next test
            GUI_INITIALIZED = GUI_SIGNAL_INITIALIZED = False
            gui_thread = gui_ready = None
            test_multithreading_lock.release()

    if unpatch_threading:
        unpatch_gtkmvc_model_mt()
    if e:
        raise e


def shutdown_environment_only_core(config=True, caplog=None, expected_warnings=0, expected_errors=0):
    from rafcon.core.singleton import state_machine_manager
    # in the gui case, the state machines have to be deleted while the gui is still running with add_gui_callback
    # if add_gui_callback is not used, then a multi threading RuntimeError will be raised by the PatchedModelMT
    state_machine_manager.delete_all_state_machines()
    shutdown_environment(config, False, caplog, expected_warnings, expected_errors, unpatch_threading=False)


def wait_for_gui():
    import gtk
    while gtk.events_pending():
        gtk.main_iteration(False)


def run_gui_thread(gui_config=None, runtime_config=None):
    import gobject
    import gtk
    from rafcon.core.start import reactor_required
    from rafcon.gui.start import start_gtk, install_reactor
    global gui_ready
    # see https://stackoverflow.com/questions/35700140/pygtk-run-gtk-main-loop-in-a-seperate-thread
    gobject.threads_init()
    if reactor_required():
        install_reactor()
    from rafcon.gui.controllers.main_window import MainWindowController
    from rafcon.gui.views.main_window import MainWindowView

    initialize_environment_gui(gui_config, runtime_config)
    main_window_view = MainWindowView()
    main_window_view.get_top_widget().set_gravity(gtk.gdk.GRAVITY_STATIC)
    MainWindowController(rafcon.gui.singleton.state_machine_manager_model, main_window_view)

    print "run_gui thread: ", currentThread(), currentThread().ident, "gui.singleton thread ident:", \
        rafcon.gui.singleton.thread_identifier

    # Wait for GUI to initialize
    wait_for_gui()
    # Set an event when the gtk loop is running
    gobject.idle_add(gui_ready.set)
    start_gtk()


def run_gui(core_config=None, gui_config=None, runtime_config=None, libraries=None, timeout=5, patch_threading=True):

    if gui_config is None:
        gui_config = {'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False}
    if 'HISTORY_ENABLED' not in gui_config.keys():
        gui_config['HISTORY_ENABLED'] = False
    if 'AUTO_BACKUP_ENABLED' not in gui_config.keys():
        gui_config['AUTO_BACKUP_ENABLED'] = False

    if patch_threading:
        patch_gtkmvc_model_mt()
    global gui_ready, gui_thread, gui_executed_once
    # IMPORTANT enforce gtk.gtkgl import in the python main thread to avoid segfaults
    # noinspection PyUnresolvedReferences
    import gtk.gtkgl

    print "WT thread: ", currentThread(), currentThread().ident
    gui_ready = Event()
    gui_thread = Thread(target=run_gui_thread, args=[gui_config, runtime_config])
    gui_thread.start()

    used_gui_threads.append(gui_thread)
    print "used_gui_threads", used_gui_threads
    # gui callback needed as all state machine from former tests are deleted in initialize_environment_core
    call_gui_callback(initialize_environment_core, core_config, libraries)
    if not gui_ready.wait(timeout):
        from rafcon.gui.start import stop_gtk
        stop_gtk()
        raise RuntimeError("Could not start GUI")

    # IMPORTANT signal handler and respective import of gui.start to avoid that singletons are created in this thread
    # -> TODO cleanup with app-class creation
    initialize_signal_handler()
    gui_executed_once = True


def wait_for_gui_quit(timeout=5):
    global gui_thread
    gui_thread.join(timeout)
    return not gui_thread.is_alive()


def close_gui(already_quit=False, force_quit=True):
    from rafcon.core.singleton import state_machine_execution_engine
    from rafcon.gui.singleton import main_window_controller
    if not already_quit:
        call_gui_callback(state_machine_execution_engine.stop)
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        # delete_all_state_machines should be done by the quit gui method
        # TODO maybe add the force quit flag as option to the arguments
        call_gui_callback(menubar_ctrl.on_quit_activate, None, None, force_quit)
    if not wait_for_gui_quit():
        assert False, "Could not close the GUI"


original_ModelMT_notify_observer = None
original_state_start = None
original_run_state_machine = None
state_threads = []
used_gui_threads = []
auto_backup_threads = []


def patch_gtkmvc_model_mt():
    print "patch"
    global state_threads, original_ModelMT_notify_observer, original_state_start, original_run_state_machine,\
        used_gui_threads, auto_backup_threads

    import rafcon.core.states.state
    import rafcon.core.execution.execution_engine
    import gtkmvc
    from gtkmvc.model_mt import Model, _threading, gobject
    from rafcon.core.states.state import run_id_generator, threading
    from rafcon.core.execution.execution_engine import StateMachineExecutionStatus, logger, Queue

    original_ModelMT_notify_observer = gtkmvc.model_mt.ModelMT.__notify_observer__
    original_state_start = rafcon.core.states.state.State.start
    original_run_state_machine = rafcon.core.execution.execution_engine.ExecutionEngine._run_active_state_machine
    print original_ModelMT_notify_observer, original_run_state_machine, original_state_start
    state_threads = []

    def state_start(self, execution_history, backward_execution=False, generate_run_id=True):
        self.execution_history = execution_history
        if generate_run_id:
            self._run_id = run_id_generator()
        self.backward_execution = copy.copy(backward_execution)
        self.thread = threading.Thread(target=self.run)
        # !!!!!!!!!!!!! patched line !!!!!!!!!!!!!
        state_threads.append(self.thread)
        self.thread.start()

    def _patched_run_active_state_machine(self):
        """Store running state machine and observe its status
        """
        # Create new concurrency queue for root state to be able to synchronize with the execution
        self._ExecutionEngine__running_state_machine = self.state_machine_manager.get_active_state_machine()
        self._ExecutionEngine__running_state_machine.root_state.concurrency_queue = Queue.Queue(maxsize=0)

        if self._ExecutionEngine__running_state_machine:
            self._ExecutionEngine__running_state_machine.start()

            self._ExecutionEngine__wait_for_finishing_thread = threading.Thread(target=self._wait_for_finishing)
            # !!!!!!!!!!!!! patched line !!!!!!!!!!!!!
            state_threads.append(self._ExecutionEngine__wait_for_finishing_thread)
            self._ExecutionEngine__wait_for_finishing_thread.start()
        else:
            logger.warn("Currently no active state machine! Please create a new state machine.")
            self.set_execution_mode(StateMachineExecutionStatus.STOPPED)

    def __patched__notify_observer__(self, observer, method, *args, **kwargs):
        """This makes a call either through the gtk.idle list or a
        direct method call depending whether the caller's thread is
        different from the observer's thread"""

        from notifications import feed_debugging_graph
        feed_debugging_graph(self, observer, method, *args, **kwargs)

        if not self._ModelMT__observer_threads.has_key(observer):
            logger.error("ASSERT WILL COME observer not in observable threads observer: {0} observable: {1}"
                         "-> known threads are {2}".format(observer, self, self._ModelMT__observer_threads))
        assert self._ModelMT__observer_threads.has_key(observer)
        if _threading.currentThread() == self._ModelMT__observer_threads[observer]:
            # standard call => single threaded
            # print "{0} -> {1}: single threading '{2}' in call_thread {3} object_generation_thread {3} \n{4}" \
            #       "".format(self.__class__.__name__, observer.__class__.__name__, method.__name__,
            #                 self._ModelMT__observer_threads[observer], (args, kwargs))
            return Model.__notify_observer__(self, observer, method, *args, **kwargs)
        else:
            # multi-threading call
            if _threading.currentThread() in state_threads or _threading.currentThread() in auto_backup_threads:
                # print "Notification from state thread", _threading.currentThread()
                gobject.idle_add(self._ModelMT__idle_callback, observer, method, args, kwargs)
                return
            elif _threading.currentThread() in used_gui_threads \
                    and self._ModelMT__observer_threads[observer] in used_gui_threads:
                # As long as the gtk module keeps constant the gtk main thread will always have the same thread id!
                # But, if the module is patched as in "test_interface.py" the gtk thread will get another thread id!
                # Thus, if both threads are in used_gui_threads then we simply allow this case!
                print "Both threads are former gui threads! Current thread {}, Observer thread {}".format(
                    _threading.currentThread(), self._ModelMT__observer_threads[observer])
                return Model.__notify_observer__(self, observer, method, *args, **kwargs)
                # gobject.idle_add(self._ModelMT__idle_callback, observer, method, args, kwargs)
                # return
            else:
                print "{0} -> {1}: multi threading '{2}' in call_thread {3} object_generation_thread {4} \n{5}" \
                      "".format(self.__class__.__name__, observer.__class__.__name__, method.__name__,
                                _threading.currentThread(), self._ModelMT__observer_threads[observer], (args, kwargs))

                # print "state threads", state_threads
                # print "used_gui_threads", used_gui_threads
                raise RuntimeError("This test should not have multi-threading constellations.")

    gtkmvc.model_mt.ModelMT.__notify_observer__ = __patched__notify_observer__
    rafcon.core.states.state.State.start = state_start
    rafcon.core.execution.execution_engine.ExecutionEngine._run_active_state_machine = _patched_run_active_state_machine


def unpatch_gtkmvc_model_mt():
    print "unpatch"
    global state_threads, original_ModelMT_notify_observer, original_state_start, original_run_state_machine

    import rafcon.core.states.state
    import rafcon.core.execution.execution_engine
    import gtkmvc
    if any([e is None for e in original_ModelMT_notify_observer, original_state_start, original_run_state_machine]):
        raise EnvironmentError("All methods to un-patch have to be set not None.")
    gtkmvc.model_mt.ModelMT.__notify_observer__ = original_ModelMT_notify_observer
    rafcon.core.states.state.State.start = original_state_start
    rafcon.core.execution.execution_engine.ExecutionEngine._run_state_machine = original_run_state_machine
    original_ModelMT_notify_observer = original_state_start = original_run_state_machine = None
    state_threads = []


def dummy_gui(caplog):
    """ This function just starts up an empty gui and closes it again. This is needed to initially create
        the gui singletons in the gui thread.

    :param caplog: the caplog object provided by pytests's caplog fixture
    :return: None
    """
    global gui_executed_once
    if not gui_executed_once:
        run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
        try:
            # do nothing, just open gui and close it afterwards
            assert True
        except:
            raise
        finally:
            close_gui()
            shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)
