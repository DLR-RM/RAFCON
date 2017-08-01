from os.path import join, realpath, dirname
import rafcon
import subprocess
import sys
import testing_utils


def test_library_resave():
    script = join(dirname(realpath(rafcon.__file__)), "gui", "resave_state_machines.py")
    config_path = join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config")
    library_folder = join(testing_utils.LIBRARY_SM_PATH, "generic")
    target_folder = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "resave_test", "test_library_resave")
    cmd = "PYTHONPATH={}:$PYTHONPATH".format(dirname(rafcon.__path__[0]))
    cmd += " && " + sys.executable + " %s %s %s %s" % (script, config_path, library_folder, target_folder)
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    import os.path
    assert os.path.isfile(join(target_folder, "wait", "statemachine.json"))


if __name__ == '__main__':
    test_library_resave()
    # pytest.main(['-s', __file__])
