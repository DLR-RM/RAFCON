import time
from os.path import join

# general tool elements
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils

import pytest

logger = log.get_logger(__name__)


@pytest.mark.unstable
@pytest.mark.parametrize('gui', [{
"gui_config": {
    "CONSOLE_FOLLOW_LOGGING": False,
    "LOGGING_SHOW_VERBOSE": True,
    "LOGGING_SHOW_DEBUG": True,
    "LOGGING_SHOW_WARNING": True,
    "LOGGING_SHOW_ERROR": True
},
"libraries": {
    "ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
}}], indirect=True, ids=["with logging on, ros and turtle libraries"])
def test_logging_view_widget(gui):
    """The function triggers and test basic functions of the logging widget in the debug console.

    At the moment those functions are tested:
    - in general:
        - the cursor position is constant while logging view is updated no matter if the follow mode is enabled or not
        - if the old cursor line disappears because of disabling of logging levels the cursor recovers on neighbour line
          (TODO #1)
    - for the follow mode:
        - the scrollbar is positioned to see the last logs if follow mode is enabled
        - if the follow mode is disabled the cursor is on its last position and the focus jumps back (TODO #2)
    """
    from rafcon.gui.config import global_gui_config
    from .test_menu_bar import create_state_machine
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # take the cursor on the first debug line
    debug_console_ctrl = main_window_controller.debug_console_controller
    logging_console_ctrl = debug_console_ctrl.logging_console_controller

    def check_scrollbar_adjustment_to_be_at_bottom():
        testing_utils.wait_for_gui()
        gui(testing_utils.wait_for_gui)
        # waiting for the gui is not sufficient
        # calling show(), show_now(), show_all(), realize() or reset_style() on the scrollbar does not work either
        scrolled_down = False
        counter = 0
        while (not scrolled_down) and counter < 10:
            adj = logging_console_ctrl.view['scrollable'].get_vadjustment()
            if not int(adj.get_value()) == int(adj.get_upper() - adj.get_page_size()):
                testing_utils.wait_for_gui()
                time.sleep(0.1)
            else:
                scrolled_down = False
            counter += 1

        if not int(adj.get_value()) == int(adj.get_upper() - adj.get_page_size()):
            logger.warning('The scroller seems not to be at the end of the page {0} == {1}'
                           ''.format(int(adj.get_value()), int(adj.get_upper() - adj.get_page_size())))

    line_number = 8
    line_offset = 10
    gui(logging_console_ctrl.view.set_cursor_position, line_number, line_offset)
    gui(logging_console_ctrl.view.scroll_to_cursor_onscreen)

    logger.debug("0 test if line was selected")
    current_line_number, current_line_offset = gui(logging_console_ctrl.view.get_cursor_position)
    text_of_line_number = gui(logging_console_ctrl.view.get_text_of_line, line_number)
    text_of_current_line = gui(logging_console_ctrl.view.get_text_of_line, current_line_number)
    assert text_of_current_line == text_of_line_number
    print('#'*20, "\n\nThis is focused {0} \n\n".format(text_of_current_line), '#'*20)

    logger.debug("1 test if cursor line is constant for change to 'CONSOLE_FOLLOW_LOGGING' True")
    gui(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', True)
    current_line_number, current_line_offset = gui(logging_console_ctrl.view.get_cursor_position)
    current_lenght = gui(logging_console_ctrl.view.len)
    assert line_number == current_line_number
    # 1.1 test check if scrollbar is on max position"
    check_scrollbar_adjustment_to_be_at_bottom()

    logger.debug("2 test if cursor line is constant for change to 'CONSOLE_FOLLOW_LOGGING' False")
    gui(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', False)
    current_line_number, current_line_offset = gui(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number
    # TODO #1 check that the scrollbar position allows to see the cursor

    logger.debug("3 test if cursor line is constant for active logging")
    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    gui(gui.core_singletons.state_machine_manager.add_state_machine, state_machine)

    current_line_number, current_line_offset = gui(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number

    gui(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', True)

    gui(menubar_ctrl.on_new_activate, None)

    current_line_number, current_line_offset = gui(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number
    # 3.1 test check if scrollbar is on max position"
    check_scrollbar_adjustment_to_be_at_bottom()

    # TODO #1 check for recovery onto close by logger messages if current line type is disabled

    # check bug case -> config will not be written to file system if logging parameters are changed
    with open(global_gui_config.config_file_path, 'r') as f:
        config_file_start = f.read()

    gui(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', False)

    with open(global_gui_config.config_file_path, 'r') as f:
        config_file_end = f.read()

    assert config_file_end == config_file_start

    print("finished debug console test")

if __name__ == '__main__':
    # test_logging_view_widget(None)
    pytest.main(['-s', __file__])
