import os
from os.path import join, dirname, realpath
import pytest

import rafcon
import rafcon.gui.singleton
import rafcon.gui.resave_state_machines as resave
import testing_utils


def test_resave(caplog):
    testing_utils.initialize_rafcon()
    testing_utils.remove_all_libraries()
    RAFCON_PATH = realpath(rafcon.__path__[0])
    TEST_SM_PATH = join(dirname(RAFCON_PATH), 'test_scripts')

    folder_to_convert = join(TEST_SM_PATH, "tutorials")
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


