import os

from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager

from tests import utils as testing_utils

CURRENT_LIBRARY_ROOT_NAME = 'generic'
NEW_LIBRARY_ROOT_NAME = 'new_generic'
STATE_MACHINE_NAME = 'all_generic_libraries'


def test_rename_library_root(caplog):
    testing_utils.initialize_environment(gui_already_started=False, libraries={})

    try:
        from rafcon.gui.helpers.state_machine import rename_library_root
        from rafcon.gui.singleton import global_config

        state_machine_path = os.path.abspath(os.path.join(testing_utils.TEST_STATE_MACHINES_PATH, STATE_MACHINE_NAME))

        rename_library_root(CURRENT_LIBRARY_ROOT_NAME, NEW_LIBRARY_ROOT_NAME)
        library_manager.clean_loaded_libraries()
        library_manager.refresh_libraries()

        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)
        testing_utils.initialize_environment(gui_already_started=False, libraries={
            "new_generic": testing_utils.GENERIC_PATH
        })
        library = storage.load_state_machine_from_path(state_machine_path)

        assert library is not None

        library_paths = global_config.get_config_value('LIBRARY_PATHS')
        del library_paths[CURRENT_LIBRARY_ROOT_NAME]
        global_config.save_configuration()

        rename_library_root(NEW_LIBRARY_ROOT_NAME, CURRENT_LIBRARY_ROOT_NAME)
        library_manager.clean_loaded_libraries()
        library_manager.refresh_libraries()

        library = storage.load_state_machine_from_path(state_machine_path)

        assert library is not None
    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


if __name__ == '__main__':
    test_rename_library_root(None)
    # pytest.main(['-s', __file__])
