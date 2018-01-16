import gtk
import threading
import pytest

# gui elements
import rafcon.gui.singleton
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView

# core elements
import rafcon.core.config
import rafcon.core.singleton

# general tool elements
from rafcon.utils import log
import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


def trigger_gvm_signals(main_window_controller):

    gvm = rafcon.core.singleton.global_variable_manager
    gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')

    view = gvm_controller.view['global_variable_tree_view']
    view.grab_focus()

    gvm.set_variable('new_0', 0)

    # use gui callback to wait for gv row generation
    call_gui_callback(gvm_controller.apply_new_global_variable_value, 0, '2')
    assert gvm.get_variable('new_0') == 2

    call_gui_callback(gvm_controller.apply_new_global_variable_name, 0, 'changed_global_0')
    assert gvm.get_variable('changed_global_0')

    call_gui_callback(gvm_controller.apply_new_global_variable_type, 0, 'float')
    assert gvm.get_data_type('changed_global_0') is float

    access_key = gvm.lock_variable('changed_global_0')
    assert not gvm_controller.global_variable_is_editable('changed_global_0', 'testing...')

    gvm.unlock_variable('changed_global_0', access_key)

    call_gui_callback(gvm_controller.on_add, view)
    assert len(gvm.get_all_keys()) is 2

    call_gui_callback(gvm_controller.remove_core_element, 'changed_global_0')
    assert len(gvm.get_all_keys()) is 1

    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(menubar_ctrl.on_quit_activate, None)

# TODO this needs a test run from the widget view site


def test_gui(caplog):
    testing_utils.initialize_environment(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})

    testing_utils.remove_all_gvm_variables()

    main_window_controller = MainWindowController(rafcon.gui.singleton.state_machine_manager_model, MainWindowView())

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()

    thread = threading.Thread(target=trigger_gvm_signals, args=[main_window_controller])
    thread.start()
    gtk.main()
    logger.debug("after gtk main")

    thread.join()

    # expected_errors=1 because global_variable_is_editable throws an error
    testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=1)


if __name__ == '__main__':
    test_gui(None)
    # pytest.main(['-s', __file__])
