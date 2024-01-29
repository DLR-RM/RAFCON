import os

from tests import utils as testing_utils
from tests.utils import RAFCON_TEMP_PATH_TEST_BASE


def test_core_open_folder(monkeypatch):
    """Tests `open_folder_cmd_line` function from `rafcon.core.interface`"""
    print("execute test_core_open_folder")
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns "/tmp"
    monkeypatch.setattr(core_interface, 'input', lambda _: "/tmp")

    # Return user input
    assert core_interface.open_folder_cmd_line("query") == "/tmp"
    # Return user input despite default path given
    assert core_interface.open_folder_cmd_line("query", "/home") == "/tmp"

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(core_interface, 'input', lambda _: "")
    # Return None if no user input and no default path
    assert core_interface.open_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.open_folder_cmd_line("query", "/tmp") == "/tmp"
    # Return None if no user input and default path does not exist
    assert core_interface.open_folder_cmd_line("query", "/non/existing/path") is None


def test_core_create_folder(monkeypatch):
    """Tests `create_folder_cmd_line` function from `rafcon.core.interface`"""
    print("execute test_core_create_folder")
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns RAFCON_TEMP_PATH_TEST_BASE
    monkeypatch.setattr(core_interface, 'input', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert core_interface.create_folder_cmd_line("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert core_interface.create_folder_cmd_line("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert core_interface.create_folder_cmd_line("query", "new",
                                                 "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(core_interface, 'input', lambda _: "")

    # Return None if no user input and no default path
    assert core_interface.create_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.create_folder_cmd_line("query", "new_folder",
                                                 RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                     RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be created (without root permissions)
    # in some ci environments the path "/root/not/writable" is writable
    # assert core_interface.create_folder_cmd_line("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert core_interface.create_folder_cmd_line("query", "new_folder") is None


def test_core_save_folder(monkeypatch):
    """Tests `save_folder_cmd_line` function from `rafcon.core.interface`"""
    print("execute test_core_save_folder")
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns RAFCON_TEMP_PATH_TEST_BASE
    monkeypatch.setattr(core_interface, 'input', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert core_interface.save_folder_cmd_line("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert core_interface.save_folder_cmd_line("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert core_interface.save_folder_cmd_line("query", "new",
                                               "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(core_interface, 'input', lambda _: "")

    # Return None if no user input and no default path
    assert core_interface.save_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.save_folder_cmd_line("query", "new_folder",
                                               RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                   RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be saved (without root permissions)
    # in some ci environments the path "/root/not/writable" is writable
    # assert core_interface.save_folder_cmd_line("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert core_interface.save_folder_cmd_line("query", "new_folder") is None


if __name__ == '__main__':
    import pytest
    pytest.main(['-s', __file__])
