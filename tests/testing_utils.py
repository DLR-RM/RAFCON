import sys
import copy
import signal
import tempfile
from os import mkdir, environ
from os.path import join, dirname, realpath, exists, abspath
from threading import Lock, Condition, Event, Thread

import rafcon
from rafcon.utils import log, constants
from rafcon.core.config import global_config

test_multithreading_lock = Lock()

gui_thread = None
gui_ready = None
exception_info = None

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

global_config.load(path=join(TESTS_PATH, "assets", "configs", "valid_config"))


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
    assert counted_warnings == expected_warnings
    assert counted_errors == expected_errors


def call_gui_callback(callback, *args):
    """Wrapper method for glib.idle_add

    This method is intended as replacement for idle_add. It wraps the method with a callback option. The advantage is
    that this way, the call is blocking. The method return, when the callback method has been called and executed.

    :param callback: The callback method, e.g. on_open_activate
    :param args: The parameters to be passed to the callback method
    """
    global exception_info
    import glib
    condition = Condition()
    exception_info = None

    @log.log_exceptions()
    def fun():
        """Call callback and notify condition variable
        """
        global exception_info
        try:
            callback(*args)
        except:
            # Exception within this asynchronously called function won't reach pytest. This is why we have to store
            # the information about the exception to re-raise it at the end of the synchronous call.
            exception_info = sys.exc_info()
        finally:  # Finally is also executed in the case of exceptions
            condition.acquire()
            condition.notify()
            condition.release()

    glib.idle_add(fun, priority=glib.PRIORITY_LOW)
    # Wait for the condition to be notified
    condition.acquire()
    # TODO: implement timeout that raises an exception
    condition.wait()
    condition.release()
    if exception_info:
        raise exception_info[0], exception_info[1], exception_info[2]


def rewind_and_set_libraries(libraries=None):
    """ Clear libraries, set new libraries and secure default libraries set."""
    from rafcon.core.config import global_config
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


def initialize_environment(core_config=None, gui_config=None, runtime_config=None, libraries=None):
    """ Initialize global configs, libraries and aquire multi threading lock

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
    from rafcon.core.config import global_config
    from rafcon.core.singleton import library_manager, state_machine_manager
    from rafcon.gui.config import global_gui_config
    from rafcon.gui.runtime_config import global_runtime_config
    from rafcon.gui.start import signal_handler

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

    state_machine_manager.delete_all_state_machines()

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

    signal.signal(signal.SIGINT, signal_handler)


def shutdown_environment(config=True, gui_config=True, caplog=None, expected_warnings=0, expected_errors=0):
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
    try:
        if caplog is not None:
            assert_logger_warnings_and_errors(caplog, expected_warnings, expected_errors)
    finally:
        rewind_and_set_libraries()
        reload_config(config, gui_config)
        test_multithreading_lock.release()


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
    initialize_environment(core_config, gui_config, runtime_config, libraries)
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


def close_gui():
    from rafcon.core.singleton import state_machine_execution_engine
    from rafcon.gui.singleton import main_window_controller
    state_machine_execution_engine.stop()
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

    if not wait_for_gui_quit():
        assert False, "Could not close the GUI"
