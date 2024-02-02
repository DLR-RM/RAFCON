import os

from rafcon.core.storage import storage

from tests import utils as testing_utils


CURRENT_LIBRARY_NAME = '99_bottles_of_beer'
NEW_LIBRARY_NAME = 'renamed_library'
STATE_MACHINE_NAME = '99_bottles_of_beer_in_library'

CURRENT_LIBRARY_NAME2 = 'library_low1'
NEW_LIBRARY_NAME2 = 'new_library_name'
STATE_MACHINE_NAME2 = 'library_middle1'


def test_rename_library(caplog):
    testing_utils.initialize_environment(gui_already_started=False, libraries={
        "tutorials": testing_utils.TUTORIAL_PATH,
        "ros": testing_utils.ROS_PATH,
        "turtle_libraries": testing_utils.TURTLE_PATH,
    })

    try:
        from rafcon.gui.helpers.state_machine import rename_library

        current_library_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, CURRENT_LIBRARY_NAME))
        new_library_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, NEW_LIBRARY_NAME))
        state_machine_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, STATE_MACHINE_NAME))
        library = storage.load_state_machine_from_path(current_library_path)

        assert library.file_system_path == current_library_path

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]

        rename_library(current_library_path, new_library_path, 'tutorials', CURRENT_LIBRARY_NAME, NEW_LIBRARY_NAME)
        library = storage.load_state_machine_from_path(new_library_path)

        assert library.file_system_path == new_library_path
        assert library.root_state.name == NEW_LIBRARY_NAME

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert NEW_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]

        rename_library(new_library_path, current_library_path, 'tutorials', NEW_LIBRARY_NAME, CURRENT_LIBRARY_NAME)
        library = storage.load_state_machine_from_path(current_library_path)

        assert library.file_system_path == current_library_path
        assert library.root_state.name == CURRENT_LIBRARY_NAME

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]
    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_rename_library_missing_states(caplog):
    testing_utils.initialize_environment(gui_already_started=False, libraries={
        "tutorials": testing_utils.TUTORIAL_PATH,
        "ros": testing_utils.ROS_PATH,
        "turtle_libraries": testing_utils.TURTLE_PATH,
    })

    try:
        from rafcon.gui.helpers.state_machine import rename_library

        current_library_path = os.path.abspath(os.path.join(testing_utils.DEEP_LIBRARIES_PATH, CURRENT_LIBRARY_NAME2))
        new_library_path = os.path.abspath(os.path.join(testing_utils.DEEP_LIBRARIES_PATH, NEW_LIBRARY_NAME2))
        state_machine_path = os.path.abspath(os.path.join(testing_utils.DEEP_LIBRARIES_PATH, STATE_MACHINE_NAME2))
        library = storage.load_state_machine_from_path(current_library_path)

        assert library.file_system_path == current_library_path

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME2 in [state.name for state in state_machine.root_state.states.values()]

        rename_library(current_library_path, new_library_path, 'unit_test_state_machines/deep_libraries', CURRENT_LIBRARY_NAME2, NEW_LIBRARY_NAME2)
        library = storage.load_state_machine_from_path(new_library_path)

        assert library.file_system_path == new_library_path
        assert library.root_state.name == NEW_LIBRARY_NAME2

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert NEW_LIBRARY_NAME2 in [state.name for state in state_machine.root_state.states.values()]

        rename_library(new_library_path, current_library_path, 'unit_test_state_machines/deep_libraries', NEW_LIBRARY_NAME2, CURRENT_LIBRARY_NAME2)
        library = storage.load_state_machine_from_path(current_library_path)

        assert library.file_system_path == current_library_path
        assert library.root_state.name == CURRENT_LIBRARY_NAME2

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME2 in [state.name for state in state_machine.root_state.states.values()]
    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


if __name__ == '__main__':
    test_rename_library(None)
    test_rename_library_missing_states(None)
    # pytest.main(['-s', __file__])
