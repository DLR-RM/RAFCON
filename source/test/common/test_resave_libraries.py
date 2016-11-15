import os
from os.path import join, dirname, realpath
import getpass
import tempfile
import pytest

import rafcon
import rafcon.mvc.resave_state_machines as resave
import testing_utils


def test_resave(caplog):
    testing_utils.start_rafcon()
    testing_utils.remove_all_libraries()
    RAFCON_TEMP_PATH_BASE = os.path.join(tempfile.gettempdir(), 'rafcon-{0}/{1}'.format(getpass.getuser(),
                                                                                        os.getpid()))
    RAFCON_TEMP_PATH_TEST_BASE = join(RAFCON_TEMP_PATH_BASE, 'unit_tests')
    RAFCON_PATH = realpath(rafcon.__path__[0])
    TEST_SM_PATH = join(dirname(RAFCON_PATH), 'test_scripts')

    folder_to_convert = TEST_SM_PATH+"/tutorials"
    config_path = os.path.dirname(os.path.abspath(__file__))+ "/config_path"
    print "folder to convert: " + folder_to_convert
    print "config path: " + config_path
    resave.convert_libraries_in_path(config_path, folder_to_convert, "/tmp/rafcon_unit_tests/resave_test")

    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)

if __name__ == '__main__':
    test_resave(None)
    # pytest.main([__file__])


