import __builtin__

from testing_utils import RAFCON_TEMP_PATH_TEST_BASE
import rafcon.core.interface as core_interface


def test_core_open_folder(monkeypatch):
    """Tests `open_folder_cmd_line` function from `rafcon.core.interface`"""

    # replaces raw_input by expression that returns "/tmp"
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "/tmp")

    # Return user input
    assert core_interface.open_folder_cmd_line("query") == "/tmp"
    # Return user input despite default path given
    assert core_interface.open_folder_cmd_line("query", "/home") == "/tmp"

    # replaces raw_input by expression that returns ""
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "")
    # Return None if no user input and no default path
    assert core_interface.open_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.open_folder_cmd_line("query", "/tmp") == "/tmp"
    # Return None if no user input and default path does not exist
    assert core_interface.open_folder_cmd_line("query", "/non/existing/path") is None


def test_core_create_folder(monkeypatch):
    """Tests `create_folder_cmd_line` function from `rafcon.core.interface`"""

    # replaces raw_input by expression that returns RAFCON_TEMP_PATH_TEST_BASE
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert core_interface.create_folder_cmd_line("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert core_interface.create_folder_cmd_line("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces raw_input by expression that returns ""
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "")

    # Return None if no user input and no default path
    assert core_interface.create_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.create_folder_cmd_line("query", RAFCON_TEMP_PATH_TEST_BASE) == RAFCON_TEMP_PATH_TEST_BASE
    # Return None if no user input and default path cannot be created (without root permissions)
    assert core_interface.create_folder_cmd_line("query", "/root/not/writable") is None
