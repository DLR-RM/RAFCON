import os
from os.path import join
import pytest

import rafcon.gui.resave_state_machines as resave
import testing_utils


def test_resave(caplog):
    testing_utils.initialize_rafcon()
    testing_utils.remove_all_libraries()
    folder_to_convert = testing_utils.get_test_sm_path("tutorials")
    target_folder = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "resave_test")
    config_path = join(os.path.dirname(os.path.abspath(__file__)), "config_path")
    print "folder to convert: " + folder_to_convert
    print "config path: " + config_path
    resave.convert_libraries_in_path(config_path, folder_to_convert, target_folder)

    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)

if __name__ == '__main__':
    #test_resave(None)
    pytest.main(['-s', __file__])


