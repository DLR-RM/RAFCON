import pytest
from os.path import realpath, dirname
import rafcon
import subprocess
import os
import sys
import testing_utils
FILE_MODIFIED_BY_STATE_MACHINE = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE,
                                              "test_start_script.txt")
if os.path.exists(FILE_MODIFIED_BY_STATE_MACHINE):
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_open():
    script = os.path.join(dirname(realpath(rafcon.__file__)), "core", "start.py")
    start_path = testing_utils.get_test_sm_path("unit_test_state_machines/start_script_test")
    cmd = sys.executable + " %s -o %s" % (script, start_path)
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp_file = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp_file.read()
    tmp_file.close()
    assert (res == "start, state, "), "start script failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_state():
    script = os.path.join(dirname(realpath(rafcon.__file__)), "core", "start.py")
    start_path = testing_utils.get_test_sm_path("unit_test_state_machines/start_script_test")
    state_path = "UTUOSC/AHWBOG"
    print start_path
    cmd = sys.executable + " %s -o %s -s %s" % (script, start_path, state_path)
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp_file = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp_file.read()
    tmp_file.close()
    assert (res == "state, "), "start from state failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)


def test_start_script_valid_config():
    # valid config
    script = os.path.join(dirname(realpath(rafcon.__file__)), "core", "start.py")
    start_path = testing_utils.get_test_sm_path("unit_test_state_machines/start_script_test")
    config = os.path.join(testing_utils.TESTS_PATH, "common", "configs_for_start_script_test", "valid_config", "config.yaml")
    cmd = sys.executable + " %s -o %s -c %s" % (script, start_path, config)
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    tmp = open(FILE_MODIFIED_BY_STATE_MACHINE, "r")
    res = tmp.read()
    tmp.close()
    assert (res == "start, state, "), "start with valid config failed"
    os.remove(FILE_MODIFIED_BY_STATE_MACHINE)

'''
def test_start_script_invalid_config(caplog):
    # invalid config
    script = os.path.join(dirname(realpath(rafcon.__file__)), "core", "start.py")
    start_path = testing_utils.get_test_sm_path("unit_test_state_machines/start_script_test")
    config = os.path.join(testing_utils.TESTS_PATH, "common", "configs_for_start_script_test", "invalid_config")
    cmd = "python %s -o %s -c %s" % (script, start_path, config)
    subprocess.call(cmd, shell=True)
    tmp = open("/tmp/rafcon_unit_tests/test_start_script.txt", "r")
    res = tmp.read()
    tmp.close()
    assert (res == "start, state, "), "start with invalid config failed"
    os.remove("/tmp/rafcon_unit_tests/test_start_script.txt")
    testing_utils.assert_logger_warnings_and_errors(caplog, 0, 0)
'''

if __name__ == '__main__':
    # test_start_script_open()
    # test_start_script_state()
    # test_start_script_valid_config()
    pytest.main([__file__])
