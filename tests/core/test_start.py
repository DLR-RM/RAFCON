from __future__ import print_function
import pytest  # pragma no cover
import os
from os.path import dirname, join
from rafcon.utils import constants
import rafcon.core.start as start
from rafcon.core.singleton import state_machine_manager
import signal

from tests import utils as testing_utils


def test_start():
    """ Test core.start.py script run on console which open a state machine, run it and final
    checks the output file on consistency.
    """
    # python_executable = str(sys.executable)
    # script = join(testing_utils.RAFCON_PATH, "core", "start.py")
    start_path = testing_utils.get_test_sm_path(
        join("unit_test_state_machines", "start_script_test"))
    # proc = multiprocessing.Process([
    #     python_executable, script, '-o', start_path, '-c', testing_utils.RAFCON_TEMP_PATH_CONFIGS
    # ])
    args = (['-o', start_path, '-c', testing_utils.RAFCON_TEMP_PATH_CONFIGS])

    # output = str(proc.stdout.read())
    # print("\ntest_start_script_open")
    # assert "enter state_1" in output
    # assert "enter state_2" in output

    # def kill_process(proc):
    #     if proc is not None:
    #         proc.send_signal(signal.SIGINT)
    #         proc.wait()
    #         proc = None

    # import atexit
    # atexit.register(kill_process, proc)

    start.main(args)
    start.signal_handler(signal.SIGINT)
    state_machine_manager.delete_all_state_machines()


if __name__ == '__main__':
    test_start()
