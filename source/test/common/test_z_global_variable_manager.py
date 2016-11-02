
import gtk
import threading

# general tool elements
from rafcon.utils import log

# core elements
import rafcon.statemachine.config
import rafcon.statemachine.singleton

# mvc elements
import rafcon.mvc.singleton

import rafcon.mvc.config as gui_config
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

# test environment elements

import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)

@log.log_exceptions(None, gtk_quit=True)
def trigger_gvm_signals(*args):

    main_window_controller = args[0]

    gvm = rafcon.statemachine.singleton.global_variable_manager
    gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')

    view = gvm_controller.view['global_variable_tree_view']
    view.grab_focus()

    gvm.set_variable('new_0', 0)

    gvm_controller.update_global_variables_list_store()

    # change value to something else than the inital 0, so its true for the assert function
    gvm_controller.apply_new_global_variable_value(0, '2')
    assert gvm.get_variable('new_0')

    gvm_controller.apply_new_global_variable_name(0, 'changed_global_0')
    assert gvm.get_variable('changed_global_0')

    gvm_controller.apply_new_global_variable_type(0, 'float')
    assert gvm.get_data_type('changed_global_0') is float

    gvm.lock_variable('changed_global_0')
    assert not gvm_controller.global_variable_is_editable('changed_global_0', 'testing...')

    gvm_controller.on_add(view)
    assert gvm.variable_exist('new_global_0')

    gvm_controller.remove_core_element('new_global_0')
    assert not gvm.variable_exist('new_global_0')

    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_gui(caplog):
    testing_utils.start_rafcon()

    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)

    testing_utils.remove_all_libraries()

    testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()

    thread = threading.Thread(target=trigger_gvm_signals, args=[main_window_controller, main_window_view])
    thread.start()
    gtk.main()
    logger.debug("after gtk main")

    thread.join()
    testing_utils.test_multithrading_lock.release()

    # expected_errors=1 because global_variable_is_editable throws an error
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors = 1)


if __name__ == '__main__':
    test_gui(None)
    # pytest.main(['-s', __file__])
