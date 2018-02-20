import os
import gtk

import testing_utils
from testing_utils import RAFCON_TEMP_PATH_TEST_BASE


def test_core_open_folder(monkeypatch):
    """Tests `open_folder_cmd_line` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    import __builtin__
    print "execute test_core_open_folder"
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns "/tmp"
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "/tmp")

    # Return user input
    assert core_interface.open_folder_cmd_line("query") == "/tmp"
    # Return user input despite default path given
    assert core_interface.open_folder_cmd_line("query", "/home") == "/tmp"

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "")
    # Return None if no user input and no default path
    assert core_interface.open_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.open_folder_cmd_line("query", "/tmp") == "/tmp"
    # Return None if no user input and default path does not exist
    assert core_interface.open_folder_cmd_line("query", "/non/existing/path") is None


def test_core_create_folder(monkeypatch):
    """Tests `create_folder_cmd_line` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    print "execute test_core_create_folder"
    import __builtin__
    import rafcon.core.interface as core_interface
    # replaces raw_input by an expression that returns RAFCON_TEMP_PATH_TEST_BASE
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert core_interface.create_folder_cmd_line("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert core_interface.create_folder_cmd_line("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert core_interface.create_folder_cmd_line("query", "new", "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces raw_input by an expression that returns ""
    monkeypatch.setattr(__builtin__, 'raw_input', lambda _: "")

    # Return None if no user input and no default path
    assert core_interface.create_folder_cmd_line("query") is None
    # Return default path if no user input is given
    assert core_interface.create_folder_cmd_line("query", "new_folder", RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                          RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be created (without root permissions)
    assert core_interface.create_folder_cmd_line("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert core_interface.create_folder_cmd_line("query", "new_folder") is None


class PatchedFileChooserDialog(gtk.FileChooserDialog):
    """Subclass for FileChooserDialog
    
    FileChooserDialog cannot be monkey-patched directly. It must first be replaced by a subclass, which is this one.
    """
    pass


def test_gui_tests(monkeypatch, caplog):
    # let the gui thread create the gui singletons by opening and closing an empty gui
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})

    try:
        pass
    except:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)

    # # creating a new monkeypatch object will only work in a py.test case, not by calling test_gui_tests directly
    # from _pytest.monkeypatch import MonkeyPatch
    # monkeypatch = MonkeyPatch()

    # This test must not be called by py.test directly!
    # As it is a test without gui it must not create the core and gui singletons,
    # otherwise the multi-threading test will fail
    # _test_gui_open_folder(monkeypatch)
    # _test_gui_create_folder(monkeypatch)


def test_gui_open_folder(monkeypatch):
    """Tests `open_folder` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    print "execute test_gui_open_folder"
    import rafcon.gui.interface as gui_interface
    # prepare FileChooserDialog for monkey-patching
    monkeypatch.setattr(gtk, "FileChooserDialog", PatchedFileChooserDialog)
    # replaces run by an expression that returns gtk.RESPONSE_OK
    monkeypatch.setattr(gtk.FileChooserDialog, 'run', lambda _: gtk.RESPONSE_OK)
    # replaces get_filename by an expression that returns "/tmp"
    monkeypatch.setattr(gtk.FileChooserDialog, 'get_filename', lambda _: "/tmp")

    # Return user input
    assert gui_interface.open_folder("query") == "/tmp"
    # Return user input despite default path given
    assert gui_interface.open_folder("query", "/home") == "/tmp"

    # replaces run by an expression that returns gtk.RESPONSE_CANCEL
    monkeypatch.setattr(gtk.FileChooserDialog, 'run', lambda _: gtk.RESPONSE_CANCEL)

    # Return None if no user input and no default path
    assert gui_interface.open_folder("query") is None
    # Return default path if no user input is given
    assert gui_interface.open_folder("query", "/tmp") == "/tmp"
    # Return None if no user input and default path does not exist
    assert gui_interface.open_folder("query", "/non/existing/path") is None


def test_gui_create_folder(monkeypatch):
    """Tests `create_folder` function from `rafcon.core.interface`"""
    testing_utils.dummy_gui(None)
    print "execute test_gui_create_folder"
    import rafcon.gui.interface as gui_interface
    # prepare FileChooserDialog for monkey-patching
    monkeypatch.setattr(gtk, "FileChooserDialog", PatchedFileChooserDialog)
    # replaces run by an expression that returns gtk.RESPONSE_OK
    monkeypatch.setattr(gtk.FileChooserDialog, 'run', lambda _: gtk.RESPONSE_OK)
    # replaces get_filename by an expression that returns "/tmp"
    monkeypatch.setattr(gtk.FileChooserDialog, 'get_filename', lambda _: RAFCON_TEMP_PATH_TEST_BASE)

    # Return user input
    assert gui_interface.create_folder("query") == RAFCON_TEMP_PATH_TEST_BASE
    # Return user input despite default path given
    assert gui_interface.create_folder("query", "/home") == RAFCON_TEMP_PATH_TEST_BASE
    assert gui_interface.create_folder("query", "new", "/home") == RAFCON_TEMP_PATH_TEST_BASE

    # replaces run by an expression that returns gtk.RESPONSE_CANCEL
    monkeypatch.setattr(gtk.FileChooserDialog, 'run', lambda _: gtk.RESPONSE_CANCEL)

    # Return None if no user input and no default path
    assert gui_interface.create_folder("query") is None
    # Return default path if no user input is given
    assert gui_interface.create_folder("query", "new_folder", RAFCON_TEMP_PATH_TEST_BASE) == os.path.join(
                                                          RAFCON_TEMP_PATH_TEST_BASE, "new_folder")
    # Return None if no user input and default path cannot be created (without root permissions)
    assert gui_interface.create_folder("query", "new_folder", "/root/not/writable") is None
    # Return None if no user input and insufficient path information given
    assert gui_interface.create_folder("query", "new_folder") is None


if __name__ == '__main__':
    # test_gui_open_folder(None)
    # _test_gui_create_folder(None)
    import pytest
    pytest.main(['-s', __file__])
