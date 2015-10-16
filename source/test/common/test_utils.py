import rafcon
from os.path import join, dirname
import getpass
from threading import Lock, Condition

test_multithrading_lock = Lock()

TMP_TEST_PATH = "/tmp/{0}/rafcon_unit_tests".format(getpass.getuser())

RAFCON_PATH = rafcon.__path__[0]
TEST_SM_PATH = join(dirname(RAFCON_PATH), 'test_scripts')


def get_test_sm_path(state_machine_name):
    return join(TEST_SM_PATH, state_machine_name)


def reload_config(config=True, gui_config=True):
    import rafcon
    if config:
        rafcon.statemachine.config.global_config.load()
    if gui_config:
        rafcon.mvc.config.global_gui_config.load()


def remove_all_libraries():
    from rafcon.statemachine.config import global_config
    library_paths = global_config.get_config_value("LIBRARY_PATHS")
    libs = [lib for lib in library_paths]
    for lib in libs:
        del library_paths[lib]


def assert_logger_warnings_and_errors(caplog, expected_warnings=0, expected_errors=0):
    import logging
    counted_warnings = 0
    counted_errors = 0
    for record in caplog.records():
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

sm_manager_model = None