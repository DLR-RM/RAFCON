import pytest
from os.path import realpath, dirname, join
import rafcon.utils.filesystem
import subprocess
import os
import sys
import shutil
import testing_utils
FILE_MODIFIED_BY_STATE_MACHINE = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE,
                                              "test_start_script.txt")
if os.path.exists(FILE_MODIFIED_BY_STATE_MACHINE):
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_open():
    """ Test core.start.py script run on console which open a state machine, run it and final checks the output file on
    consistency.
    """
    testing_utils.dummy_gui(None)

    script = join(testing_utils.RAFCON_PATH, "core", "start.py")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    cmd = "%s -o %s" % (script, start_path)
    print "\ntest_start_script_open: \n", cmd
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp_file = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp_file.read()
    tmp_file.close()
    assert (res == "start, state, "), "start script failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_state():
    """ Test core.start.py script run by python call which open a state machine, run from a specific state and  final
    checks the output file on consistency.
    """
    testing_utils.dummy_gui(None)
    script = join(testing_utils.RAFCON_PATH, "core", "start.py")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    state_path = "UTUOSC/AHWBOG"
    print start_path
    cmd = sys.executable + " %s -o %s -s %s" % (script, start_path, state_path)
    print "\ntest_start_script_state: \n", cmd
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp_file = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp_file.read()
    tmp_file.close()
    assert (res == "state, "), "start from state failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_initial_default_config_folder_generation():
    """ Test core.start.py and gui.start.py script run on console which should initiate the config folder.
    """
    testing_utils.dummy_gui(None)

    user_config_folder = rafcon.utils.filesystem.get_default_config_path()
    backup_user_config_folder = os.path.join(os.path.expanduser('~'), '.config', 'rafcon_backup')
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
    # valid config
    bin_path = join(dirname(testing_utils.RAFCON_PATH), "..", "bin")
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    config = join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config", "config.yaml")
    cmd = "export PATH={0}:$PATH && rafcon_core -o {1} -c {2}".format(bin_path, start_path, config)
    print "\ntest_start_script_valid_config: \n", cmd
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp.read()
    tmp.close()
    assert (res == "start, state, "), "start with valid config failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_valid_rmpm_env():
    """Tests the execution of rafcon_start in an environment created by RMPM
    """
    # TODO: replace rafcon_start with rafcon_core after the next release
    testing_utils.dummy_gui(None)
    import distutils.spawn
    rmpm_env = os.environ.copy()
    rmpm_env["PATH"] = "/volume/software/common/packages/rmpm/latest/bin/{}:".format(os.getenv(
        "DLRRM_HOST_PLATFORM", "osl42-x86_64")) + rmpm_env["PATH"]
    if not distutils.spawn.find_executable("rmpm_do"):
        print "Could not find rmpm_do, skipping test"
    start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    config = join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config", "config.yaml")
    cmd = "eval `rmpm_do env --env-format=embed_sh sw.common.rafcon` && rafcon_start -o {0} -c {1}" \
          "".format(start_path, config)
    print "\ntest_start_script_valid_config: \n", cmd
    rafcon_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, env=rmpm_env)
    rafcon_process.wait()
    output = rafcon_process.communicate()[0]
    print "LOG: \n", output
    assert rafcon_process.returncode == 0


def test_start_script_print_help_with_gui():
    """ Test rafcon_start_gui console call which run a RAFCON instance and let it print the helper message and checks
    if the process terminates correctly.
    """
    testing_utils.dummy_gui(None)
    script = join(testing_utils.RAFCON_PATH, "gui", "start.py")
    # start_path = testing_utils.get_test_sm_path(join("unit_test_state_machines", "start_script_test"))
    # cmd = "%s -o %s" % (script, start_path)
    cmd = script + " -h"
    print "\ntest_start_script_open_with_gui: ", cmd
    rafcon_gui_process = subprocess.Popen(cmd, shell=True)
    print "process PID: ", rafcon_gui_process.pid
    # rafcon_gui_process.terminate()
    rafcon_gui_process.wait()
    assert rafcon_gui_process.returncode == 0


if __name__ == '__main__':
    # test_start_script_open()
    # test_start_script_state()
    # test_start_script_valid_config()
    pytest.main(['-s', __file__])
