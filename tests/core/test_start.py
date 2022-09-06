from os.path import join
import rafcon.core.start as start
from rafcon.core.singleton import state_machine_manager
import signal

from tests import utils as testing_utils


def test_start():
    """ Test core.start.py script run on console which open a state machine, run it and final
    checks the output file on consistency.
    """
    start_path = testing_utils.get_test_sm_path(
        join("unit_test_state_machines", "start_script_test"))
    args = (['-o', start_path, '-c', testing_utils.RAFCON_TEMP_PATH_CONFIGS])

    start.main(optional_args=args)
    start.signal_handler(signal.SIGINT)
    state_machine_manager.delete_all_state_machines()


if __name__ == '__main__':
    test_start()
