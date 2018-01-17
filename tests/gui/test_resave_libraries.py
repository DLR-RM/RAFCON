import os
import pytest

import testing_utils


def test_resave(caplog):
    testing_utils.initialize_environment(gui_already_started=False)
    folder_to_convert = testing_utils.TUTORIAL_PATH
    target_folder = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "resave_test")
    config_path = os.path.join(testing_utils.TESTS_PATH, "assets", "configs", "valid_config")
    print "folder to convert: " + folder_to_convert
    print "config path: " + config_path
    import rafcon.gui.resave_state_machines as resave
    resave.convert_libraries_in_path(config_path, folder_to_convert, target_folder)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

if __name__ == '__main__':
    #test_resave(None)
    pytest.main(['-s', __file__])
