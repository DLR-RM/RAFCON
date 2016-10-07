import pytest
from os.path import realpath, dirname
import rafcon
import subprocess
import sys


def test_library_resave():
    script = dirname(realpath(rafcon.__file__)) + "/mvc/resave_state_machines.py"
    config_path = rafcon.__path__[0] + "/../test/common/configs_for_start_script_test/valid_config/"
    library_folder = rafcon.__path__[0] + "/../libraries/generic"
    target_folder = "/tmp/rafcon/unit_testtest_library_resave"
    cmd = sys.executable + " %s %s %s %s" % (script, config_path, library_folder, target_folder)
    cmd_res = subprocess.call(cmd, shell=True)
    assert cmd_res == 0
    import os.path
    assert os.path.isfile(target_folder + "/wait/statemachine.json")

if __name__ == '__main__':
    test_library_resave()
    # pytest.main([__file__])