import os

import pytest

from rafcon.core.singleton import library_manager
from rafcon.core.storage import storage
from rafcon.utils import log

from tests import utils as testing_utils


CURRENT_LIBRARY_NAME = '99_bottles_of_beer'
NEW_LIBRARY_NAME = 'renamed_library'
STATE_MACHINE_NAME = '99_bottles_of_beer_in_library'


def test_rename_library(caplog):

    testing_utils.initialize_environment_core()

    try:
        from rafcon.gui.helpers.state_machine import rename_state_machine

        testing_utils.rewind_and_set_libraries({
            "tutorials": testing_utils.TUTORIAL_PATH,
            "ros": testing_utils.ROS_PATH,
            "turtle_libraries": testing_utils.TURTLE_PATH,
        })

        library_manager.initialize()

        if 'unit_test_state_machines' in library_manager.libraries:
            del library_manager.libraries['unit_test_state_machines']

        current_library_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, CURRENT_LIBRARY_NAME))
        new_library_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, NEW_LIBRARY_NAME))
        state_machine_path = os.path.abspath(os.path.join(testing_utils.TUTORIAL_PATH, STATE_MACHINE_NAME))
        library = storage.load_state_machine_from_path(current_library_path)

        assert library.file_system_path == current_library_path
        assert library.root_state.name.replace('_', ' ').lower() == CURRENT_LIBRARY_NAME.replace('_', ' ').lower()

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]

        rename_state_machine(current_library_path, new_library_path, NEW_LIBRARY_NAME)
        library = storage.load_state_machine_from_path(new_library_path)

        if 'unit_test_state_machines' in library_manager.libraries:
            del library_manager.libraries['unit_test_state_machines']

        assert library.file_system_path == new_library_path
        assert library.root_state.name == NEW_LIBRARY_NAME

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert NEW_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]

        rename_state_machine(new_library_path, current_library_path, CURRENT_LIBRARY_NAME)
        library = storage.load_state_machine_from_path(current_library_path)

        if 'unit_test_state_machines' in library_manager.libraries:
            del library_manager.libraries['unit_test_state_machines']

        assert library.file_system_path == current_library_path
        assert library.root_state.name == CURRENT_LIBRARY_NAME

        state_machine = storage.load_state_machine_from_path(state_machine_path)

        assert CURRENT_LIBRARY_NAME in [state.name for state in state_machine.root_state.states.values()]
    finally:
        testing_utils.shutdown_environment_only_core()


if __name__ == '__main__':
    test_rename_library(None)
