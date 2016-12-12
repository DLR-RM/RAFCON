import os
import gtk
import threading
import shutil

# mvc
from rafcon.gui.runtime_config import global_runtime_config
import rafcon.gui.singleton
from rafcon.gui.utils import constants
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView

from rafcon.utils import log

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


def get_stored_window_size(window_name):
    size = global_runtime_config.get_config_value(window_name.upper() + '_SIZE')
    if not size:
        size = constants.WINDOW_SIZE[window_name.upper()]
    return size


def get_stored_window_position(window_key):
    return global_runtime_config.get_config_value(window_key.upper() + '_BAR_WINDOW_POS')


@log.log_exceptions(None, gtk_quit=True)
def trigger_docking_signals(*args):
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

    def test_bar(window, window_name, window_key):
        window.connect('configure-event', ensure_completion)
        call_gui_callback(main_window_controller.view["undock_{}_button".format(window_key)].emit, "clicked")
        wait_for_gui()
        wait_for_gui()
        assert window.get_property('visible') == True
        should_size = get_stored_window_size(window_name)
        assert window.get_size() == should_size
        window.resize(600, 600)
        wait_for_gui()
        wait_for_gui()

        undocked_window_view = getattr(main_window_controller.view, window_name.lower())
        redock_button = getattr(undocked_window_view, "top_tool_bar")['redock_button']
        call_gui_callback(redock_button.emit, "clicked")
        main_window_controller.view["undock_{}_button".format(window_key)].emit("clicked")
        assert window.get_size() == (600, 600)
        wait_for_gui()
        window.move(100, 100)
        wait_for_gui()
        wait_for_gui()
        call_gui_callback(redock_button.emit, "clicked")
        main_window_controller.view["undock_{}_button".format(window_key)].emit("clicked")
        # Does not work reliable...
        # assert window.get_position() == (100, 100)

    print "test left_bar_window"
    test_bar(main_window_controller.view.left_bar_window.get_top_widget(), "LEFT_BAR_WINDOW", 'left_bar')
    print "test right_bar_window"
    test_bar(main_window_controller.view.right_bar_window.get_top_widget(), "RIGHT_BAR_WINDOW", 'right_bar')
    print "test console_bar_window"
    test_bar(main_window_controller.view.console_bar_window.get_top_widget(), "CONSOLE_BAR_WINDOW", 'console')

    call_gui_callback(menu_bar_ctrl.on_quit_activate, None)


@log.log_exceptions(None, gtk_quit=True)
def trigger_pane_signals(*args):
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

    def test_bar(window, window_name, window_key):
        window.connect('configure-event', ensure_completion)
        call_gui_callback(mw_ctrl.view["undock_{}_button".format(window_key)].emit, "clicked")
        wait_for_gui()
        wait_for_gui()
        undocked_window_view = getattr(mw_ctrl.view, window_name.lower())
        redock_button = getattr(undocked_window_view, "top_tool_bar")['redock_button']
        call_gui_callback(redock_button.emit, "clicked")
        wait_for_gui()

    print "test left_bar_window"
    test_bar(mw_ctrl.view.left_bar_window.get_top_widget(), "LEFT_BAR_WINDOW", 'left_bar')
    print "test right_bar_window"
    test_bar(mw_ctrl.view.right_bar_window.get_top_widget(), "RIGHT_BAR_WINDOW", 'right_bar')
    print "test console_bar_window"
    test_bar(mw_ctrl.view.console_bar_window.get_top_widget(), "CONSOLE_BAR_WINDOW", 'console')

    print "check if pane positions are still like in runtime_config.yaml"
    for config_id, pane_id in constants.PANE_ID.iteritems():
        print "check pos of ", config_id, pane_id
        assert mw_ctrl.view[pane_id].get_position() == stored_pane_positions[config_id]

    call_gui_callback(menu_bar_ctrl.on_quit_activate, None)


def test_window_positions(caplog):
    testing_utils.initialize_rafcon()
    mirror_runtime_config_file()
    global_runtime_config.load(config_file='runtime_config.yaml', path=DOCKING_TEST_FOLDER)
    testing_utils.sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_docking_signals, args=[main_window_controller])
    thread.start()

    gtk.main()
    thread.join()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_warnings=warnings)


def test_pane_positions(caplog):
    testing_utils.initialize_rafcon()
    mirror_runtime_config_file()
    global_runtime_config.load(config_file='runtime_config.yaml', path=DOCKING_TEST_FOLDER)
    testing_utils.sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_pane_signals, args=[main_window_controller])
    thread.start()

    gtk.main()
    thread.join()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_warnings=warnings)


if __name__ == '__main__':
    pytest.main([__file__, '-xs'])
