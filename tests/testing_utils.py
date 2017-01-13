import signal
import tempfile
from os import mkdir, environ
from os.path import join, dirname, realpath, exists, abspath
from threading import Lock, Condition, Event, Thread

import rafcon
from rafcon.utils import log, constants
from rafcon.core.config import global_config
global_config.load(path=join(join(dirname(abspath(__file__)), "common"), "config_path"))

test_multithreading_lock = Lock()

gui_thread = None
gui_ready = None

sm_manager_model = None

RAFCON_TEMP_PATH_TEST_BASE = join(constants.RAFCON_TEMP_PATH_BASE, 'unit_tests')
if not exists(RAFCON_TEMP_PATH_TEST_BASE):
    mkdir(RAFCON_TEMP_PATH_TEST_BASE)

# temporary path that can be used if multiple instance of RAFCON should use one reference-path in a test
RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE = join(constants.RAFCON_TEMP_PATH_BASE, '..', 'unit_tests')
if not exists(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE):
    mkdir(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE)

TESTS_PATH = dirname(__file__)
RAFCON_PATH = realpath(rafcon.__path__[0])
LIBRARY_SM_PATH = join(dirname(RAFCON_PATH), '..', 'share', 'libraries')
EXAMPLES_SM_PATH = join(dirname(RAFCON_PATH), '..', 'share', 'examples')
TEST_SM_PATH = join(TESTS_PATH, 'assets')
TEST_SCRIPT_PATH =  join(dirname(RAFCON_PATH), 'test_scripts')
TUTORIAL_PATH = join(dirname(RAFCON_PATH), "..", "share", "examples", "tutorials")


def get_unique_temp_path():
    return tempfile.mkdtemp(dir=RAFCON_TEMP_PATH_TEST_BASE)


def get_test_sm_path(state_machine_name):
    return join(TEST_SM_PATH, state_machine_name)


def reload_config(config=True, gui_config=True):
    import rafcon
    if config:
        rafcon.core.config.global_config.load()
    if gui_config:
        import rafcon.gui.config
        rafcon.gui.config.global_gui_config.load()


def remove_all_libraries():
    from rafcon.core.config import global_config
    library_paths = global_config.get_config_value("LIBRARY_PATHS")
    libs = [lib for lib in library_paths]
    for lib in libs:
        del library_paths[lib]
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
    assert counted_warnings == expected_warnings
    assert counted_errors == expected_errors


def call_gui_callback(callback, *args):
    """Wrapper method for glib.idle_add

    This method is intended as replacement for idle_add. It wraps the method with a callback option. The advantage is
    that this way, the call is blocking. The method return, when the callback method has been called and executed.

    :param callback: The callback method, e.g. on_open_activate
    :param args: The parameters to be passed to the callback method
    """
    import glib
    condition = Condition()

    @log.log_exceptions()
    def fun():
        """Call callback and notify condition variable
        """
        try:
            callback(*args)
        finally:  # Finally is also executed in the case of exceptions and reraises the exception at the end
            condition.acquire()
            condition.notify()
            condition.release()

    glib.idle_add(fun)
    # Wait for the condition to be notified
    condition.acquire()
    # TODO: implement timeout that raises an exception
    condition.wait()
    condition.release()


def initialize_rafcon(core_config=None, gui_config=None, runtime_config=None, libraries=None):
    from rafcon.core.config import global_config
    from rafcon.core.singleton import library_manager, state_machine_manager
    from rafcon.gui.config import global_gui_config
    from rafcon.gui.runtime_config import global_runtime_config
    from rafcon.gui.start import signal_handler

    test_multithreading_lock.acquire()

    global_config.load()
    global_gui_config.load()
    global_runtime_config.load()
    if isinstance(core_config, dict):
        for key, value in core_config.iteritems():
            global_config.set_config_value(key, value)
    if isinstance(gui_config, dict):
        for key, value in gui_config.iteritems():
            global_gui_config.set_config_value(key, value)
    if isinstance(runtime_config, dict):
        for key, value in runtime_config.iteritems():
            global_runtime_config.set_config_value(key, value)

    rafcon_library_path = join(dirname(RAFCON_PATH), '..', 'share', 'libraries')
    remove_all_libraries()
    if not isinstance(libraries, dict):
        libraries = {}
    if "generic" not in libraries:
        libraries["generic"] = join(rafcon_library_path, 'generic')
    global_config.set_config_value("LIBRARY_PATHS", libraries)
    environ['RAFCON_LIB_PATH'] = rafcon_library_path
    library_manager.initialize()
    state_machine_manager.delete_all_state_machines()

    signal.signal(signal.SIGINT, signal_handler)


def wait_for_gui():
    import gtk
    while gtk.events_pending():
        gtk.main_iteration(False)


def run_gui_thread():
    global gui_ready
    import gtk
    from rafcon.gui.controllers.main_window import MainWindowController
    from rafcon.gui.views.main_window import MainWindowView
    from rafcon.gui.singleton import state_machine_manager_model

    MainWindowController(state_machine_manager_model, MainWindowView())

    # Wait for GUI to initialize
    wait_for_gui()
    gtk.idle_add(gui_ready.set)
    gtk.main()


def run_gui(core_config=None, gui_config=None, runtime_config=None, libraries=None, timeout=5):
    global gui_ready, gui_thread
    initialize_rafcon(core_config, gui_config, runtime_config, libraries)
    gui_ready = Event()
    gui_thread = Thread(target=run_gui_thread)
    gui_thread.start()
    if not gui_ready.wait(timeout):
        import gtk
        gtk.idle_add(gtk.main_quit)
        raise RuntimeError("Could not start GUI")


def wait_for_gui_quit(timeout=5):
    global gui_thread
    gui_thread.join(timeout)
    return not gui_thread.is_alive()
