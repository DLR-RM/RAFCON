import os
import gtk
import threading
import shutil
import time

from rafcon.utils import log

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

import rafcon.mvc.singleton
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.mvc.utils import constants

import testing_utils
from testing_utils import call_gui_callback
import pytest

DOCKING_TEST_FOLDER = testing_utils.RAFCON_TEMP_PATH_TEST_BASE + '/config_docking_test'
warnings = 0


def mirror_runtime_config_file():
    global warnings
    path = os.path.join(os.path.expanduser('~'), '.config', 'rafcon')
    runtime_config_file = os.path.join(path, 'runtime_config.yaml')
    if os.path.isfile(runtime_config_file):
        if not os.path.exists(DOCKING_TEST_FOLDER):
            os.mkdir(DOCKING_TEST_FOLDER)
        shutil.copyfile(runtime_config_file,
                        testing_utils.RAFCON_TEMP_PATH_TEST_BASE + '/config_docking_test/runtime_config.yaml')
    else:
        warnings += 1


def get_stored_window_size(window_key):
    size = global_runtime_config.get_config_value(window_key.upper() + '_BAR_WINDOW_SIZE')
    if not size:
        size = constants.WINDOW_SIZE[window_key.upper() + '_BAR_WINDOW']
    return size


def get_stored_window_position(window_key):
    return global_runtime_config.get_config_value(window_key.upper() + '_BAR_WINDOW_POS')


@log.log_exceptions(None, gtk_quit=True)
def trigger_docking_signals(*args):
    print "Wait for the gui to initialize"
    time.sleep(1.0)
    main_window_controller = args[0]
    menu_bar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    condition = threading.Condition()

    def ensure_completion(window, event):
        condition.acquire()
        print "notify"
        condition.notify()
        condition.release()
        return True

    def wait_for_gui():
        condition.acquire()
        print "wait"
        condition.wait(0.5)
        print "finally"
        condition.release()

    def test_bar(window, window_key):
        window.connect('configure-event', ensure_completion)
        call_gui_callback(getattr(main_window_controller, 'on_{}_bar_undock_clicked'.format(window_key)), None)
        wait_for_gui()
        wait_for_gui()
        assert window.get_property('visible') == True
        should_size = get_stored_window_size(window_key)
        assert window.get_size() == should_size
        window.resize(600, 600)
        wait_for_gui()
        wait_for_gui()
        call_gui_callback(getattr(main_window_controller, 'on_{}_bar_dock_clicked'.format(window_key)), None)
        call_gui_callback(getattr(main_window_controller, 'on_{}_bar_undock_clicked'.format(window_key)), None)
        assert window.get_size() == (600, 600)
        wait_for_gui()
        window.move(100, 100)
        wait_for_gui()
        wait_for_gui()
        call_gui_callback(getattr(main_window_controller, 'on_{}_bar_dock_clicked'.format(window_key)), None)
        call_gui_callback(getattr(main_window_controller, 'on_{}_bar_undock_clicked'.format(window_key)), None)
        # Does not work relieable...
        # assert window.get_position() == (100, 100)

    print "test left_bar_window"
    test_bar(main_window_controller.view.left_bar_window.get_top_widget(), 'left')
    print "test right_bar_window"
    test_bar(main_window_controller.view.right_bar_window.get_top_widget(), 'right')
    print "test console_bar_window"
    test_bar(main_window_controller.view.console_bar_window.get_top_widget(), 'console')

    call_gui_callback(menu_bar_ctrl.on_quit_activate, None)


@log.log_exceptions(None, gtk_quit=True)
def trigger_pane_signals(*args):
    print "Wait for the gui to initialize"
    time.sleep(1.0)
    mw_ctrl = args[0]
    menu_bar_ctrl = mw_ctrl.get_controller('menu_bar_controller')
    condition = threading.Condition()

    stored_pane_positions = {}
    for config_id, pan_id in constants.PANE_ID.iteritems():
        default_pos = constants.DEFAULT_PANE_POS[config_id]
        stored_pane_positions[config_id] = global_runtime_config.get_config_value(config_id, default_pos)
        if stored_pane_positions[config_id] is None:
            import logging
            logging.warning("runtime_config-file has missing values?")
            return

    def ensure_completion(window, event):
        condition.acquire()
        print "notify"
        condition.notify()
        condition.release()
        return True

    def wait_for_gui():
        condition.acquire()
        print "wait"
        condition.wait(0.3)
        print "finally"
        condition.release()

    def test_bar(window, window_key):
        window.connect('configure-event', ensure_completion)
        call_gui_callback(getattr(mw_ctrl, 'on_{}_bar_undock_clicked'.format(window_key)), None)
        wait_for_gui()
        wait_for_gui()
        call_gui_callback(getattr(mw_ctrl, 'on_{}_bar_dock_clicked'.format(window_key)), None)
        wait_for_gui()

    print "test left_bar_window"
    test_bar(mw_ctrl.view.left_bar_window.get_top_widget(), 'left')
    print "test right_bar_window"
    test_bar(mw_ctrl.view.right_bar_window.get_top_widget(), 'right')
    print "test console_bar_window"
    test_bar(mw_ctrl.view.console_bar_window.get_top_widget(), 'console')

    print "check if pane positions are still like in runtime_config.yaml"
    for config_id, pane_id in constants.PANE_ID.iteritems():
        print "check pos of ", config_id, pane_id
        assert mw_ctrl.view[pane_id].get_position() == stored_pane_positions[config_id]

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
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_warnings=warnings)


def test_pane_positions(caplog):
    testing_utils.test_multithrading_lock.acquire()
    os.chdir(testing_utils.RAFCON_PATH + "/mvc/")
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    mirror_runtime_config_file()
    global_runtime_config.load(config_file='runtime_config.yaml', path=DOCKING_TEST_FOLDER)
    testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')
    thread = threading.Thread(target=trigger_pane_signals, args=[main_window_controller])
    thread.start()

    gtk.main()
    thread.join()
    os.chdir(testing_utils.RAFCON_PATH + "/../test/common")
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_warnings=warnings)


if __name__ == '__main__':
    pytest.main([__file__, '-xs'])
