import os
from os.path import join
import sys
import time
import threading
import subprocess
import select

import rafcon

# general tool elements
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
import pytest

logger = log.get_logger(__name__)
# TODO move this test in a separate shared element test folder


def shutdown_gui(duration_wait_for_gui):
    # wait for menu bar and main window controller
    menubar_ctrl = None
    time.sleep(2)
    while menubar_ctrl is None:
        time.sleep(duration_wait_for_gui)
        menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller') if \
            rafcon.gui.singleton.main_window_controller else None

    # quit rafcon by menu bar
    testing_utils.call_gui_callback(menubar_ctrl.on_save_as_activate, None, None,
                                    testing_utils.get_unique_temp_path())
    testing_utils.call_gui_callback(menubar_ctrl.on_stop_activate, None)
    testing_utils.call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_api_example(caplog):

    path_of_api_examples = os.path.join(testing_utils.EXAMPLES_PATH, 'api', 'generate_state_machine')
    sys.path.insert(0, path_of_api_examples)

    testing_utils.test_multithreading_lock.acquire()
    try:
        import basic_turtle_state_machine
        # TODO maybe extend example to run ros processes to check functionality of state machine too
        duration_wait_for_gui = 0.1
        timed_thread = threading.Timer(5*duration_wait_for_gui, shutdown_gui, args=[duration_wait_for_gui,])
        timed_thread.daemon = True
        timed_thread.start()

        basic_turtle_state_machine.run_turtle_demo()
        logger.debug("after gtk main")
        timed_thread.join()
    finally:
        sys.path.remove(path_of_api_examples)
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_ros_library_examples(caplog):
    """This test checks whether all ros example libraries are functional"""
    # TODO implement the tests


def test_turtle_library_examples(caplog):
    """This test checks whether all turtle example libraries are functional"""
    # TODO implement the tests


@pytest.mark.timeout(60)
def test_functionality_example(caplog):
    """Test for now only tests:
    - if the state machine can be open
    - if test can be run and stopped
    - and everything can be closed again
    """
    import rafcon.core.singleton
    from rafcon.core.storage import storage

    # The test maybe should also test if functionality are correct depicted.
    # TODO check if this is done in the common tests already

    for name in ['backward_step_barrier', 'backward_step_hierarchy', 'backward_step_preemption', 'decider_statemachine',
                 'hierarchy_abortion_handling']:
        sm_path = join(testing_utils.EXAMPLES_PATH, 'functionality_examples', name)
        print(sm_path)
        state_machine = storage.load_state_machine_from_path(sm_path)
        rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    testing_utils.test_multithreading_lock.acquire()
    try:
        # main_window_controller = rafcon.gui.singleton.main_window_controller
        for state_machine_id in list(rafcon.core.singleton.state_machine_manager.state_machines.keys()):
            rafcon.core.singleton.state_machine_execution_engine.start(state_machine_id)
            max_time = 3
            current_time = 0.0
            sleep_time = 0.2
            while not rafcon.core.singleton.state_machine_execution_engine.finished_or_stopped():
                time.sleep(sleep_time)
                current_time += sleep_time
                if current_time >= max_time:
                    break
            rafcon.core.singleton.state_machine_execution_engine.stop()
            rafcon.core.singleton.state_machine_execution_engine.join()
    finally:
        testing_utils.wait_for_gui()  # to avoid execution and model notification clinches
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=6, expected_errors=4,
                                           unpatch_threading=False)


def test_plugins_example(caplog):

    os.environ['RAFCON_PLUGIN_PATH'] = os.path.join(testing_utils.EXAMPLES_PATH, 'plugins', 'templates')
    print(os.environ.get('RAFCON_PLUGIN_PATH'))
    path_of_sm_to_run = testing_utils.get_test_sm_path(join("unit_test_state_machines", "99_bottles_of_beer_monitoring"))
    # testing_utils.initialize_environment()
    testing_utils.test_multithreading_lock.acquire()
    try:
        cmd = "{python} {start_script} -o {state_machine} -ss -c {config} -g {config}".format(
            python=str(sys.executable),
            start_script=join(testing_utils.RAFCON_PATH, 'gui', 'start.py'),
            state_machine=path_of_sm_to_run,
            config=testing_utils.RAFCON_TEMP_PATH_CONFIGS
        )
        print("cmd", cmd)
        start_time = time.time()
        # use exec! otherwise the terminate() call ends up killing the shell process and cmd is still running
        # https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true
        # stderr=subprocess.STDOUT redirects stderr to stdout
        rafcon_gui_process = subprocess.Popen("exec " + cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        # See https://stackoverflow.com/a/36477512 for details
        # Note: This (select and poll) only works on POSIX systems, not on Windows!
        poller = select.poll()
        poller.register(rafcon_gui_process.stdout, select.POLLIN)

        started = False
        plugin_loaded = False
        while True:
            if poller.poll(100):
                line = str(rafcon_gui_process.stdout.readline().decode("utf-8")).rstrip()
                if line:
                    print("process:", line)
                if "Successfully loaded plugin 'templates'" in line:
                    print("=> plugin loaded")
                    plugin_loaded = True
                if "Start execution engine" in line:
                    print("=> started")
                    started = True
                if started and "Stop the state machine execution" in line:
                    print("=> ready")
                    assert plugin_loaded
                    time.sleep(0.5)  # safety margin...
                    print("=> RAFCON is now terminated")
                    rafcon_gui_process.terminate()
                    stdout, _ = rafcon_gui_process.communicate()
                    # print("stdout: ", stdout)
                    print("something: ", _)
                    print("rafcon_gui_process.returncode", rafcon_gui_process.returncode)
                    exception_count = 0
                    for line in str(stdout.rstrip()).split("\n"):
                        print("process:", line)
                        if "Exception" in line:
                            exception_count += 1
                    assert exception_count == 0
                    assert rafcon_gui_process.returncode == 0
                    break
            else:
                # kill process after 10 seconds and return with a failure
                if time.time() - start_time > 10:
                    rafcon_gui_process.kill()
                    rafcon_gui_process.communicate()
                    assert False, "RAFCON did not start in time"
    finally:
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0,
                                           unpatch_threading=False)


def test_tutorial_state_machine_examples(caplog):
    """This test checks whether all tutorial state machine example are functional"""
    # TODO implement the tests


if __name__ == '__main__':
    test_api_example(None)
    test_ros_library_examples(None)
    test_turtle_library_examples(None)
    test_functionality_example(None)
    test_plugins_example(None)
    test_tutorial_state_machine_examples(None)
    # pytest.main(['-s', __file__])
