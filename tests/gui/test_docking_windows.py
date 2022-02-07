import threading
import pytest
import time

from tests import utils as testing_utils
from tests.utils import wait_for_gui
from rafcon.utils import log

logger = log.get_logger(__name__)
ready = threading.Event()
event_size = (0, 0)


def get_stored_window_size(window_name):
    from rafcon.gui.runtime_config import global_runtime_config
    from rafcon.gui.utils import constants
    size = global_runtime_config.get_config_value(window_name.upper() + '_SIZE')
    if not size:
        size = constants.WINDOW_SIZE[window_name.upper()]
    return size


def notify_on_event(window, event=None):
    logger.info("show/hide event: type={}".format(event.type if event else "None"))
    ready.set()
    return True


def notify_on_resize_event(window, event=None):
    global event_size
    logger.info("resize event: type={} size=({}, {})".format(event.type, event.width, event.height))
    ready.set()
    event_size = (event.width, event.height)


def wait_for_event_notification(gui):
    if not ready.wait(5):
        raise RuntimeError("A timeout occurred")
    # time.sleep(0.1)
    gui(wait_for_gui)


def assert_pos_equality(pos1, pos2, allow_delta=10):
    # print "assert_pos_equality: abs(pos1 - pos2)", abs(pos1 - pos2)
    assert abs(pos1 - pos2) <= allow_delta


def assert_size_equality(size1, size2, allow_delta=10):
    assert_pos_equality(size1[0], size2[0], allow_delta)
    assert_pos_equality(size1[1], size2[1], allow_delta)


def connect_window(gui, window, event, method):
    handler_id = gui(window.connect, event, method)
    return handler_id


@pytest.mark.unstable
@pytest.mark.parametrize('gui', [{"runtime_config": {
    'MAIN_WINDOW_MAXIMIZED': False,
    'MAIN_WINDOW_SIZE': (1500, 800),
    'MAIN_WINDOW_POS': (0, 0),
    'LEFT_BAR_WINDOW_SIZE': (800, 800),
    'RIGHT_BAR_WINDOW_SIZE': (800, 800),
    'CONSOLE_WINDOW_SIZE': (800, 800),
    'LEFT_BAR_WINDOW_POS': (10, 10),
    'RIGHT_BAR_WINDOW_POS': (10, 10),
    'CONSOLE_WINDOW_POS': (10, 10),
    'LEFT_BAR_HIDDEN': False,
    'RIGHT_BAR_HIDDEN': False,
    'CONSOLE_HIDDEN': False,
    'LEFT_BAR_WINDOW_UNDOCKED': False,
    'RIGHT_BAR_WINDOW_UNDOCKED': False,
    'CONSOLE_WINDOW_UNDOCKED': False
}}], indirect=True, ids=["with fixed runtime config values"])
def test_window_positions(gui):

    from rafcon.gui.runtime_config import global_runtime_config
    main_window_controller = gui.singletons.main_window_controller
    debug_sleep_time = 0

    def test_bar(window, window_key):
        attribute_name_of_undocked_window_view = window_name = window_key.lower() + "_window"

        configure_handler_id = connect_window(gui, window, 'configure-event', notify_on_resize_event)
        hide_handler_id = connect_window(gui, window, 'hide', notify_on_event)

        logger.info("undocking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        gui(main_window_controller.view["undock_{}_button".format(window_key.lower())].emit, "clicked")
        wait_for_event_notification(gui)
        assert window.get_property('visible') is True
        expected_size = get_stored_window_size(window_name)
        new_size = window.get_size()
        # print dir(window)
        if not bool(window.is_maximized()):
            assert_size_equality(new_size, expected_size, 90)
        else:
            maximized_parameter_name = window_key + "_WINDOW_MAXIMIZED"
            assert bool(window.is_maximized()) and global_runtime_config.get_config_value(maximized_parameter_name)

        logger.info("resizing...")
        time.sleep(debug_sleep_time)
        ready.clear()
        target_size = (1400, 800)
        try:
            assert_size_equality(new_size, target_size, 90)
            # Change target size if it is similar to the current size
            target_size = (1600, 900)
        except AssertionError:
            pass

        logger.debug("target size: {}".format(target_size))
        gui(window.resize, *target_size)
        wait_for_event_notification(gui)
        try:
            assert_size_equality(event_size, target_size, 90)
        except AssertionError:
            # For unknown reasons, there are two configure events and only the latter one if for the new window size
            ready.clear()
            wait_for_event_notification(gui)
            assert_size_equality(event_size, target_size, 90)
            logger.info("got additional configure-event")

        logger.info("docking...")
        undocked_window_view = getattr(main_window_controller.view, attribute_name_of_undocked_window_view)
        redock_button = undocked_window_view['redock_button']
        time.sleep(debug_sleep_time)
        ready.clear()
        gui(redock_button.emit, "clicked")
        wait_for_event_notification(gui)
        assert window.get_property('visible') is False

        logger.info("undocking...")
        time.sleep(debug_sleep_time)
        ready.clear()

        show_handler_id = connect_window(gui, window, 'show', notify_on_event)

        gui(main_window_controller.view["undock_{}_button".format(window_key.lower())].emit, "clicked")
        wait_for_event_notification(gui)
        assert window.get_property('visible') is True
        assert_size_equality(window.get_size(), target_size, 90)

        logger.info("docking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        gui(redock_button.emit, "clicked")
        wait_for_event_notification(gui)
        assert window.get_property('visible') is False

        gui(window.disconnect, configure_handler_id)
        gui(window.disconnect, show_handler_id)
        gui(window.disconnect, hide_handler_id)

    print("=> test left_bar_window")
    test_bar(main_window_controller.view.left_bar_window.get_parent_widget(), "LEFT_BAR")
    print("=> test right_bar_window")
    test_bar(main_window_controller.view.right_bar_window.get_parent_widget(), "RIGHT_BAR")
    print("=> test console_window")
    test_bar(main_window_controller.view.console_window.get_parent_widget(), "CONSOLE")
    gui(wait_for_gui)


@pytest.mark.unstable
@pytest.mark.parametrize('gui', [{"runtime_config": {
    'MAIN_WINDOW_MAXIMIZED': False,
    'MAIN_WINDOW_SIZE': (1500, 800),
    'MAIN_WINDOW_POS': (0, 0),
    'LEFT_BAR_DOCKED_POS': 400,
    'RIGHT_BAR_DOCKED_POS': 800,
    'CONSOLE_DOCKED_POS': 600,
    'LEFT_BAR_WINDOW_UNDOCKED': False,
    'RIGHT_BAR_WINDOW_UNDOCKED': False,
    'CONSOLE_WINDOW_UNDOCKED': False,
    'LEFT_BAR_HIDDEN': False,
    'RIGHT_BAR_HIDDEN': False,
    'CONSOLE_HIDDEN': False
}}], indirect=True, ids=["with fixed runtime config values"])
def test_pane_positions(gui):
    from rafcon.gui.runtime_config import global_runtime_config
    from rafcon.gui.utils import constants
    main_window_controller = gui.singletons.main_window_controller
    debug_sleep_time = 0.0

    stored_pane_positions = {}
    for config_id, pan_id in constants.PANE_ID.items():
        default_pos = constants.DEFAULT_PANE_POS[config_id]
        stored_pane_positions[config_id] = global_runtime_config.get_config_value(config_id, default_pos)
        if stored_pane_positions[config_id] is None:
            import logging
            logging.warning("runtime_config-file has missing values?")
            return

    def test_bar(window, window_key):

        configure_handler_id = connect_window(gui, window, 'configure-event', notify_on_event)
        hide_handler_id = connect_window(gui, window, 'hide', notify_on_event)

        print("undocking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        gui(main_window_controller.view["undock_{}_button".format(window_key.lower())].emit, "clicked")
        wait_for_event_notification(gui)

        print("docking...")
        time.sleep(debug_sleep_time)
        ready.clear()
        attribute_name_of_undocked_window_view = window_key.lower() + "_window"
        undocked_window_view = getattr(main_window_controller.view, attribute_name_of_undocked_window_view)
        redock_button = undocked_window_view['redock_button']
        gui(redock_button.emit, "clicked")
        wait_for_event_notification(gui)

        time.sleep(debug_sleep_time)
        gui(window.disconnect, configure_handler_id)
        gui(window.disconnect, hide_handler_id)

    # Info: un- and redocking the left bar will change the right bar position;
    # thus, the equality check has to be done directly after un- and redocking the right bar
    print("=> test right_bar_window")
    test_bar(main_window_controller.view.right_bar_window.get_parent_widget(), "RIGHT_BAR")
    gui(wait_for_gui)
    config_id = 'RIGHT_BAR_DOCKED_POS'
    pane_id = constants.PANE_ID['RIGHT_BAR_DOCKED_POS']
    print("check pos of ", config_id, pane_id)
    assert_pos_equality(main_window_controller.view[pane_id].get_position(), stored_pane_positions[config_id], 10)

    print("=> test console_window")
    test_bar(main_window_controller.view.console_window.get_parent_widget(), "CONSOLE")
    gui(wait_for_gui)
    config_id = 'CONSOLE_DOCKED_POS'
    pane_id = constants.PANE_ID['CONSOLE_DOCKED_POS']
    print("check pos of ", config_id, pane_id)
    assert_pos_equality(main_window_controller.view[pane_id].get_position(), stored_pane_positions[config_id], 10)

    print("=> test left_bar_window")
    test_bar(main_window_controller.view.left_bar_window.get_parent_widget(), "LEFT_BAR")
    gui(wait_for_gui)
    config_id = 'LEFT_BAR_DOCKED_POS'
    pane_id = constants.PANE_ID['LEFT_BAR_DOCKED_POS']
    print("check pos of ", config_id, pane_id)
    assert_pos_equality(main_window_controller.view[pane_id].get_position(), stored_pane_positions[config_id], 10)

if __name__ == '__main__':
    test_window_positions(None)
    test_pane_positions(None)
