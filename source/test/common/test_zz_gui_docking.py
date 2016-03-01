import os
import gtk
import threading
import time
import shutil

from rafcon.utils import log

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

import rafcon.mvc.singleton
from rafcon.mvc.runtime_config import global_runtime_config

import testing_utils
from testing_utils import call_gui_callback
import pytest

DOCKING_TEST_FOLDER = testing_utils.RAFCON_TEMP_PATH_TEST_BASE + '/config_docking_test'


def mirror_runtime_config_file():
    path = os.path.join(os.path.expanduser('~'), '.config', 'rafcon')
    runtime_config_file = os.path.join(path, 'runtime_config.yaml')
    if os.path.isfile(runtime_config_file):
        if not os.path.exists(DOCKING_TEST_FOLDER):
            os.mkdir(DOCKING_TEST_FOLDER)
        shutil.copyfile(runtime_config_file, testing_utils.RAFCON_TEMP_PATH_TEST_BASE + '/config_docking_test/runtime_config.yaml')


@log.log_exceptions(None, gtk_quit=True)
def trigger_docking_signals(*args):
    print "Wait for the gui to initialize"
    # time.sleep(1.0)
    main_window_controller = args[0]
    menu_bar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    call_gui_callback(main_window_controller.on_left_bar_undock_clicked, None)
    assert main_window_controller.view.left_bar_window.get_top_widget().get_property('visible') == True
    call_gui_callback(main_window_controller.on_left_bar_dock_clicked, None)
    assert main_window_controller.view.left_bar_window.get_top_widget().get_property('visible') == False

    call_gui_callback(main_window_controller.on_right_bar_undock_clicked, None)
    assert main_window_controller.view.right_bar_window.get_top_widget().get_property('visible') == True
    call_gui_callback(main_window_controller.on_right_bar_dock_clicked, None)
    assert main_window_controller.view.right_bar_window.get_top_widget().get_property('visible') == False

    call_gui_callback(main_window_controller.on_console_undock_clicked, None)
    assert main_window_controller.view.console_window.get_top_widget().get_property('visible') == True
    call_gui_callback(main_window_controller.on_console_dock_clicked, None)
    assert main_window_controller.view.console_window.get_top_widget().get_property('visible') == False

    call_gui_callback(menu_bar_ctrl.on_quit_activate, None)


def test_docking(caplog):
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
