import os

from tests import utils as testing_utils
from tests.utils import RAFCON_TEMP_PATH_TEST_BASE


def test_core_open_folder(monkeypatch):
    """Tests `open_folder_cmd_line` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
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
    testing_utils.dummy_gui(None)
    print("execute test_core_create_folder")
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns RAFCON_TEMP_PATH_TEST_BASE
    monkeypatch.setattr(core_interface, 'input', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert core_interface.create_folder_cmd_line("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert core_interface.create_folder_cmd_line("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert core_interface.create_folder_cmd_line("query", "new", "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(core_interface, 'input', lambda _: "")

    # Return None if no user input and no default path
    assert core_interface.create_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.create_folder_cmd_line("query", "new_folder", RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                          RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be created (without root permissions)
    # in some ci environments the path "/root/not/writable" is writable
    # assert core_interface.create_folder_cmd_line("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert core_interface.create_folder_cmd_line("query", "new_folder") is None


def test_gui_open_folder(monkeypatch):
    """Tests `open_folder` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    print("execute test_gui_open_folder")
    import rafcon.gui.interface as gui_interface
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk

    class PatchedFileChooserDialog(Gtk.FileChooserDialog):
        """Subclass for FileChooserDialog

        FileChooserDialog cannot be monkey-patched directly. It must first be replaced by a subclass, which is this one.
        """
        pass

    # prepare FileChooserDialog for monkey-patching
    monkeypatch.setattr(Gtk, "FileChooserDialog", PatchedFileChooserDialog)
    # replaces run by an expression that returns Gtk.ResponseType.OK
    monkeypatch.setattr(Gtk.FileChooserDialog, 'run', lambda _: Gtk.ResponseType.OK)
    # replaces get_filename by an expression that returns "/tmp"
    monkeypatch.setattr(Gtk.FileChooserDialog, 'get_filename', lambda _: "/tmp")

    # Return user input
    assert gui_interface.open_folder("query") == "/tmp"
    # Return user input despite default path given
    assert gui_interface.open_folder("query", "/home") == "/tmp"

    # replaces run by an expression that returns Gtk.ResponseType.CANCEL
    monkeypatch.setattr(Gtk.FileChooserDialog, 'run', lambda _: Gtk.ResponseType.CANCEL)

    # Return None if no user input and no default path
    assert gui_interface.open_folder("query") is None
    # Return default path if no user input is given
    assert gui_interface.open_folder("query", "/tmp") == "/tmp"
    # Return None if no user input and default path does not exist
    assert gui_interface.open_folder("query", "/non/existing/path") is None


def test_gui_create_folder(monkeypatch):
    """Tests `create_folder` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    print("execute test_gui_create_folder")
    import rafcon.gui.interface as gui_interface
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk

    class PatchedFileChooserDialog(Gtk.FileChooserDialog):
        """Subclass for FileChooserDialog

        FileChooserDialog cannot be monkey-patched directly. It must first be replaced by a subclass, which is this one.
        """
        pass

    # prepare FileChooserDialog for monkey-patching
    monkeypatch.setattr(Gtk, "FileChooserDialog", PatchedFileChooserDialog)
    # replaces run by an expression that returns Gtk.ResponseType.OK
    monkeypatch.setattr(Gtk.FileChooserDialog, 'run', lambda _: Gtk.ResponseType.OK)
    # replaces get_filename by an expression that returns "/tmp"
    monkeypatch.setattr(Gtk.FileChooserDialog, 'get_filename', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    class PatchedConfirmDialog(Gtk.Dialog):
        """Subclass for Dialog (when asking to replace existing files in folder)

        FileChooserDialog cannot be monkey-patched directly. It must first be replaced by a subclass, which is this one.
        """
        pass

    # prepare Dialog for monkey-patching
    monkeypatch.setattr(Gtk, "Dialog", PatchedConfirmDialog)
    # replaces run by an expression that returns Gtk.ResponseType.ACCEPT
    monkeypatch.setattr(Gtk.Dialog, 'run', lambda _: Gtk.ResponseType.ACCEPT)

    # Return user input
    assert gui_interface.create_folder("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert gui_interface.create_folder("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert gui_interface.create_folder("query", "new", "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces run by an expression that returns Gtk.ResponseType.CANCEL
    monkeypatch.setattr(Gtk.FileChooserDialog, 'run', lambda _: Gtk.ResponseType.CANCEL)

    # Return None if no user input and no default path
    assert gui_interface.create_folder("query") is None
    # Return default path if no user input is given
    assert gui_interface.create_folder("query", "new_folder", RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                          RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be created (without root permissions)
    # in some ci environments the path "/root/not/writable" is writable
    # assert gui_interface.create_folder("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert gui_interface.create_folder("query", "new_folder") is None


if __name__ == '__main__':
    # test_gui_open_folder(None)
    # _test_gui_create_folder(None)
    import pytest
    pytest.main(['-s', __file__])
