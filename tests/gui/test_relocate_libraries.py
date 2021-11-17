import os

from rafcon.core.storage import storage

from tests import utils as testing_utils

NEW_DIRECTORY = testing_utils.TEST_STATE_MACHINES_PATH
LIBRARY_NAME = '99_bottles_of_beer'
LIBRARY_PATH = 'tutorials'
NEW_LIBRARY_PATH = 'unit_test_state_machines'
LIBRARY_OS_PATH = os.path.join(testing_utils.TUTORIAL_PATH, LIBRARY_NAME)
NEW_LIBRARY_OS_PATH = os.path.join(testing_utils.TEST_STATE_MACHINES_PATH, LIBRARY_NAME)
STATE_MACHINE_OS_PATH = os.path.join(testing_utils.TUTORIAL_PATH, '99_bottles_of_beer_in_library')


def test_relocate_library(caplog):
    testing_utils.initialize_environment(gui_already_started=False, libraries={
        "tutorials": testing_utils.TUTORIAL_PATH,
    })

    try:
        from rafcon.gui.helpers.state_machine import relocate_library

        assert os.path.exists(LIBRARY_OS_PATH)
        assert not os.path.exists(NEW_LIBRARY_OS_PATH)

        state_machine = storage.load_state_machine_from_path(STATE_MACHINE_OS_PATH)
        library = list(state_machine.root_state.states.values())[0]

        assert library.library_path == LIBRARY_PATH
        assert library.library_name == LIBRARY_NAME

        relocate_library(LIBRARY_OS_PATH, LIBRARY_PATH, LIBRARY_NAME, NEW_DIRECTORY)

        assert not os.path.exists(LIBRARY_OS_PATH)
        assert os.path.exists(NEW_LIBRARY_OS_PATH)

        state_machine = storage.load_state_machine_from_path(STATE_MACHINE_OS_PATH)
        library = list(state_machine.root_state.states.values())[0]

        assert library.library_path == NEW_LIBRARY_PATH
        assert library.library_name == LIBRARY_NAME

        relocate_library(NEW_LIBRARY_OS_PATH, NEW_LIBRARY_PATH, LIBRARY_NAME, testing_utils.TUTORIAL_PATH)

        assert os.path.exists(LIBRARY_OS_PATH)
        assert not os.path.exists(NEW_LIBRARY_OS_PATH)

        state_machine = storage.load_state_machine_from_path(STATE_MACHINE_OS_PATH)
        library = list(state_machine.root_state.states.values())[0]

        assert library.library_path == LIBRARY_PATH
        assert library.library_name == LIBRARY_NAME

    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


if __name__ == '__main__':
    test_relocate_library(None)
    # pytest.main(['-s', __file__])
