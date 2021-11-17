import pytest
from os.path import dirname, join
from rafcon.utils import constants
import subprocess
import os
import sys
import shutil
from tests import utils as testing_utils


def test_start_script_open():
    """ Test core.start.py script run on console which open a state machine, run it and final checks the output file on
    consistency.
    """
    testing_utils.dummy_gui(None)

    python_executable = str(sys.executable)
    script = join(testing_utils.RAFCON_PATH, "core", "start.py")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    output = str(subprocess.check_output([python_executable, script,
                                          '-o', start_path,
                                          '-c', testing_utils.RAFCON_TEMP_PATH_CONFIGS]))
    print("\ntest_start_script_open")
    assert "enter state_1" in output
    assert "enter state_2" in output


def test_start_script_state():
    """ Test core.start.py script run by python call which open a state machine, run from a specific state and  final
    checks the output file on consistency.
    """
    testing_utils.dummy_gui(None)
    script = join(testing_utils.RAFCON_PATH, "core", "start.py")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    state_path = "UTUOSC/AHWBOG"
    print(start_path)
    output = str(subprocess.check_output([sys.executable, script,
                                          '-o', start_path,
                                          '-s', state_path,
                                          '-c', testing_utils.RAFCON_TEMP_PATH_CONFIGS]))
    print("\ntest_start_script_state")
    assert "enter state_1" not in output
    assert "enter state_2" in output


def _test_initial_default_config_folder_generation():
    """ Test core.start.py and gui.start.py script run on console which should initiate the config folder.
    """
    # TODO: this test is broken
    # when specifying a config path that does not exist, RAFCON does not start
    # Yet, the tests wants to see the opposite. This hasen't failed, yet, as the implementation was wrong:
    # The ~/.config/rafcon folder was moved to ~/.config/rafcon/rafcon_backup
    testing_utils.dummy_gui(None)

    user_config_folder = testing_utils.RAFCON_TEMP_PATH_CONFIGS
    backup_user_config_folder = os.path.join(constants.RAFCON_TEMP_PATH_BASE, 'rafcon_backup')
    try:
        if os.path.exists(user_config_folder):
            shutil.move(user_config_folder, backup_user_config_folder)

        test_start_script_open()
        assert os.path.exists(user_config_folder)
        shutil.rmtree(user_config_folder)

        test_start_script_print_help_with_gui()
        assert os.path.exists(user_config_folder)
    finally:
        if os.path.exists(user_config_folder):
            shutil.rmtree(user_config_folder)
        if os.path.exists(backup_user_config_folder):
            shutil.move(backup_user_config_folder, user_config_folder)


def test_start_script_valid_config():
    """ Test rafcon_core console call which run a rafcon instance with handed config.yaml file, open a state machine,
    run it and final checks the output file on consistency.
    """
    testing_utils.dummy_gui(None)
    script = join(testing_utils.RAFCON_BIN_PATH, "rafcon_core")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    config = join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config", "config.yaml")
    output = str(subprocess.check_output([script,
                                          '-o', start_path,
                                          '-c', config]))
    print("\ntest_start_script_valid_config")
    assert "enter state_1" in output
    assert "enter state_2" in output


def test_start_script_valid_rmpm_env():
    """Tests the execution of ``rafcon_core`` in an environment created by RMPM
    """
    testing_utils.dummy_gui(None)
    import distutils.spawn
    rmpm_env = os.environ.copy()
    rmpm_env["PATH"] = "/volume/software/common/packages/rmpm/latest/bin/{}:".format(os.getenv(
        "DLRRM_HOST_PLATFORM", "osl42-x86_64")) + rmpm_env["PATH"]
    if not distutils.spawn.find_executable("rmpm_do"):
        print("Could not find rmpm_do, skipping test")
        return
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    config = join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config", "config.yaml")
    cmd = "eval `rmpm_do env --env-format=embed_sh sw.common.rafcon` && rafcon_core -o {0} -c {1}" \
          "".format(start_path, config)
    print("\ntest_start_script_valid_config: \n", cmd)
    rafcon_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, env=rmpm_env)
    rafcon_process.wait()
    output = rafcon_process.communicate()[0]
    print("LOG: \n", output)
    assert rafcon_process.returncode == 0


def test_start_script_print_help_with_gui():
    """ Test ``rafcon`` console call which run a RAFCON instance and let it print the helper message and checks
    if the process terminates correctly.
    """
    testing_utils.dummy_gui(None)
    script = join(testing_utils.RAFCON_PATH, "gui", "start.py")
    # start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    # cmd = "%s -o %s" % (script, start_path)
    cmd = sys.executable + " " + script + " -h"
    print("\ntest_start_script_open_with_gui: ", cmd)
    rafcon_gui_process = subprocess.Popen(cmd, shell=True)
    print("process PID: ", rafcon_gui_process.pid)
    # rafcon_gui_process.terminate()
    rafcon_gui_process.wait()
    assert rafcon_gui_process.returncode == 0


if __name__ == '__main__':
    # test_start_script_open()
    # test_start_script_state()
    # test_start_script_valid_config()
    pytest.main(['-s', __file__])
