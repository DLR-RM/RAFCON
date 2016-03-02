import os
import gtk
import threading
import time

from rafcon.utils import log

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

import rafcon.mvc.singleton
from rafcon.mvc.runtime_config import global_runtime_config

import testing_utils
from testing_utils import call_gui_callback
import pytest

from test_zz_gui_docking import mirror_runtime_config_file, DOCKING_TEST_FOLDER


@log.log_exceptions(None, gtk_quit=True)
def trigger_docking_signals(*args):
    print "Wait for the gui to initialize"
    # time.sleep(1)
    main_window_controller = args[0]
    menu_bar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    min_sleep = 0.5

    # Left Bar
    call_gui_callback(main_window_controller.on_left_bar_undock_clicked, None)
    time.sleep(2*min_sleep)
    window_pos = main_window_controller.view.left_bar_window.get_top_widget().get_position()
    window_size = main_window_controller.view.left_bar_window.get_top_widget().get_size()
    assert window_pos == global_runtime_config.get_config_value('LEFT_BAR_WINDOW_POS')
    assert window_size == global_runtime_config.get_config_value('LEFT_BAR_WINDOW_SIZE')
    call_gui_callback(main_window_controller.view.left_bar_window.get_top_widget().resize, 300, 600)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.view.left_bar_window.get_top_widget().move, 100, 100)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_left_bar_dock_clicked, None)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_left_bar_undock_clicked, None)
    time.sleep(min_sleep)
    assert main_window_controller.view.left_bar_window.get_top_widget().get_position() == (100, 100)
    assert main_window_controller.view.left_bar_window.get_top_widget().get_size() == (300, 600)
    call_gui_callback(main_window_controller.on_left_bar_dock_clicked, None)

    # Right Bar
    call_gui_callback(main_window_controller.on_right_bar_undock_clicked, None)
    time.sleep(2*min_sleep)
    window_pos = main_window_controller.view.right_bar_window.get_top_widget().get_position()
    window_size = main_window_controller.view.right_bar_window.get_top_widget().get_size()
    assert window_pos == global_runtime_config.get_config_value('RIGHT_BAR_WINDOW_POS')
    assert window_size == global_runtime_config.get_config_value('RIGHT_BAR_WINDOW_SIZE')
    call_gui_callback(main_window_controller.view.right_bar_window.get_top_widget().resize, 300, 600)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.view.right_bar_window.get_top_widget().move, 100, 100)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_right_bar_dock_clicked, None)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_right_bar_undock_clicked, None)
    time.sleep(min_sleep)
    assert main_window_controller.view.right_bar_window.get_top_widget().get_position() == (100, 100)
    assert main_window_controller.view.right_bar_window.get_top_widget().get_size() == (300, 600)
    call_gui_callback(main_window_controller.on_right_bar_dock_clicked, None)

    # Console
    call_gui_callback(main_window_controller.on_console_undock_clicked, None)
    time.sleep(2*min_sleep)
    window_pos = main_window_controller.view.console_window.get_top_widget().get_position()
    window_size = main_window_controller.view.console_window.get_top_widget().get_size()
    assert window_pos == global_runtime_config.get_config_value('CONSOLE_WINDOW_POS')
    assert window_size == global_runtime_config.get_config_value('CONSOLE_WINDOW_SIZE')
    call_gui_callback(main_window_controller.view.console_window.get_top_widget().resize, 600, 300)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.view.console_window.get_top_widget().move, 100, 100)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_console_dock_clicked, None)
    time.sleep(min_sleep)
    call_gui_callback(main_window_controller.on_console_undock_clicked, None)
    time.sleep(min_sleep)
    assert main_window_controller.view.console_window.get_top_widget().get_position() == (100, 100)
    assert main_window_controller.view.console_window.get_top_widget().get_size() == (600, 300)
    call_gui_callback(main_window_controller.on_console_dock_clicked, None)

    call_gui_callback(menu_bar_ctrl.on_quit_activate, None)


def test_window_positions(caplog):
    testing_utils.test_multithrading_lock.acquire()
    os.chdir(testing_utils.RAFCON_PATH + "/mvc/")
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    mirror_runtime_config_file()
    global_runtime_config.load(config_file='runtime_config.yaml', path=DOCKING_TEST_FOLDER)
    testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')
    thread = threading.Thread(target=trigger_docking_signals, args=[main_window_controller])
    thread.start()

    gtk.main()
    thread.join()
    os.chdir(testing_utils.RAFCON_PATH + "/../test/common")
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])
