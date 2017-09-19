import threading
import pytest
import time

from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.utils import constants

import testing_utils
from testing_utils import call_gui_callback, run_gui, close_gui
from rafcon.utils import log

logger = log.get_logger(__name__)
ready = threading.Event()
event_size = (0, 0)


def get_stored_window_size(window_name):
    size = global_runtime_config.get_config_value(window_name.upper() + '_SIZE')
    if not size:
        size = constants.WINDOW_SIZE[window_name.upper()]
    return size


def notify_on_event(window, event=None):
    logger.info("event: {}".format(event))
    ready.set()
    return True


def notify_on_resize_event(window, event=None):
    global event_size
    logger.info("event: {}".format(event))
    ready.set()
    event_size = (event.width, event.height)


def wait_for_event_notification():
    if not ready.wait(5):
        raise RuntimeError("A timeout occurred")


def assert_size_equality(size1, size2):
    assert abs(size1[0] - size2[0]) <= 10
    assert abs(size1[1] - size2[1]) <= 10


def undock_sidebars():
    from rafcon.gui.singleton import main_window_controller
    debug_sleep_time = 0

    def test_bar(window, window_name, window_key):
        configure_handler_id = window.connect('configure-event', notify_on_resize_event)
        hide_handler_id = window.connect('hide', notify_on_event)

        logger.info("undocking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        call_gui_callback(main_window_controller.view["undock_{}_button".format(window_key)].emit, "clicked")
        wait_for_event_notification()
        assert window.get_property('visible') is True
        expected_size = get_stored_window_size(window_name)
        new_size = window.get_size()
        if not bool(window.maximize_initially):
            assert_size_equality(new_size, expected_size)
        else:
            maximized_parameter_name = window_key.upper() + "_WINDOW_MAXIMIZED"
            assert bool(window.maximize_initially) and global_runtime_config.get_config_value(maximized_parameter_name)

        logger.info("resizing...")
        time.sleep(debug_sleep_time)
        ready.clear()
        target_size = (600, 600)
        if new_size == target_size:
            target_size = (700, 700)
        logger.debug("target size: {}".format(target_size))
        window.resize(*target_size)
        wait_for_event_notification()
        try:
            assert_size_equality(event_size, target_size)
        except AssertionError:
            # For unknown reasons, there are two configure events and only the latter one if for the new window size
            ready.clear()
            wait_for_event_notification()
            assert_size_equality(event_size, target_size)
            logger.info("got additional configure-event")

        logger.info("docking...")
        undocked_window_view = getattr(main_window_controller.view, window_name.lower())
        redock_button = getattr(undocked_window_view, "top_tool_bar")['redock_button']
        time.sleep(debug_sleep_time)
        ready.clear()
        call_gui_callback(redock_button.emit, "clicked")
        wait_for_event_notification()
        assert window.get_property('visible') is False

        logger.info("undocking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        show_handler_id = window.connect('show', notify_on_event)
        main_window_controller.view["undock_{}_button".format(window_key)].emit("clicked")
        wait_for_event_notification()
        assert window.get_property('visible') is True
        assert_size_equality(window.get_size(), target_size)

        logger.info("docking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        call_gui_callback(redock_button.emit, "clicked")
        wait_for_event_notification()
        assert window.get_property('visible') is False

        window.disconnect(configure_handler_id)
        window.disconnect(show_handler_id)
        window.disconnect(hide_handler_id)

    print "=> test left_bar_window"
    test_bar(main_window_controller.view.left_bar_window.get_top_widget(), "LEFT_BAR_WINDOW", 'left_bar')
    print "=> test right_bar_window"
    test_bar(main_window_controller.view.right_bar_window.get_top_widget(), "RIGHT_BAR_WINDOW", 'right_bar')
    print "=> test console_bar_window"
    test_bar(main_window_controller.view.console_bar_window.get_top_widget(), "CONSOLE_BAR_WINDOW", 'console')


def check_pane_positions():
    from rafcon.gui.singleton import main_window_controller
    debug_sleep_time = 0

    stored_pane_positions = {}
    for config_id, pan_id in constants.PANE_ID.iteritems():
        default_pos = constants.DEFAULT_PANE_POS[config_id]
        stored_pane_positions[config_id] = global_runtime_config.get_config_value(config_id, default_pos)
        if stored_pane_positions[config_id] is None:
            import logging
            logging.warning("runtime_config-file has missing values?")
            return

    def test_bar(window, window_name, window_key):
        configure_handler_id = window.connect('configure-event', notify_on_event)
        hide_handler_id = window.connect('hide', notify_on_event)

        print "undocking..."
        time.sleep(debug_sleep_time)
        ready.clear()
        call_gui_callback(main_window_controller.view["undock_{}_button".format(window_key)].emit, "clicked")
        wait_for_event_notification()

        print "docking..."
        time.sleep(debug_sleep_time)
        ready.clear()
        undocked_window_view = getattr(main_window_controller.view, window_name.lower())
        redock_button = getattr(undocked_window_view, "top_tool_bar")['redock_button']
        call_gui_callback(redock_button.emit, "clicked")
        wait_for_event_notification()

        window.disconnect(configure_handler_id)
        window.disconnect(hide_handler_id)

    print "=> test left_bar_window"
    test_bar(main_window_controller.view.left_bar_window.get_top_widget(), "LEFT_BAR_WINDOW", 'left_bar')
    print "=> test right_bar_window"
    test_bar(main_window_controller.view.right_bar_window.get_top_widget(), "RIGHT_BAR_WINDOW", 'right_bar')
    print "=> test console_bar_window"
    test_bar(main_window_controller.view.console_bar_window.get_top_widget(), "CONSOLE_BAR_WINDOW", 'console')

    print "check if pane positions are still like in runtime_config.yaml"
    for config_id, pane_id in constants.PANE_ID.iteritems():
        print "check pos of ", config_id, pane_id
        assert main_window_controller.view[pane_id].get_position() == stored_pane_positions[config_id]


def test_window_positions(caplog):
    run_gui(None, {
                    'HISTORY_ENABLED': False,
                    'AUTO_BACKUP_ENABLED': False
                  }, {})
    original_runtime_config = global_runtime_config.as_dict()

    try:
        undock_sidebars()
    finally:
        for key, value in original_runtime_config.iteritems():
            global_runtime_config.set_config_value(key, value)

        close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


def test_pane_positions(caplog):

    run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    original_runtime_config = global_runtime_config.as_dict()

    try:
        check_pane_positions()
    finally:
        for key, value in original_runtime_config.iteritems():
            global_runtime_config.set_config_value(key, value)

        close_gui()
        testing_utils.shutdown_environment(caplog=caplog)

if __name__ == '__main__':
    pytest.main([__file__, '-xs'])
