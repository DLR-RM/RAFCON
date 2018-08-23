# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

import pytest

logger = log.get_logger(__name__)


@log.log_exceptions(None, gtk_quit=True)
def trigger_logging_view_gui_signals():
    """The function triggers and test basic functions of the logging widget in the debug console.

    At the moment those functions are tested:
    - in general:
        - the cursor position is constant the logging view is updated no madder if the follow mode is enabled or not
        - if the old cursor line disappears because of disabling of logging levels the cursor recovers on neighbour line
          (TODO #1)
    - for the follow mode:
        - the scrollbar is positioned to see the last logs if follow mode is enabled
        - if the follow mode is disabled the cursor is on it last position and the focus jumps back (TODO #2)
    """
    import rafcon.core.singleton
    import rafcon.gui.singleton
    from rafcon.gui.config import global_gui_config
    from test_menu_bar import create_state_machine
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # take the cursor on the first debug line
    debug_console_ctrl = main_window_controller.get_controller('debug_console_controller')
    logging_console_ctrl = debug_console_ctrl.get_controller('logging_console_controller')

    def check_scrollbar_adjustment_to_be_at_bottom():
        testing_utils.wait_for_gui()
        call_gui_callback(testing_utils.wait_for_gui)
        adj = logging_console_ctrl.view['scrollable'].get_vadjustment()
        if not int(adj.get_value()) == int(adj.get_upper() - adj.get_page_size()):
            logger.warning('The scroller seems not to be at the end of the page {0} == {1}'
                           ''.format(int(adj.get_value()), int(adj.get_upper() - adj.get_page_size())))

    line_number = 8
    line_offset = 10
    call_gui_callback(logging_console_ctrl.view.set_cursor_position, line_number, line_offset)
    call_gui_callback(logging_console_ctrl.view.scroll_to_cursor_onscreen)

    logger.debug("0 test if line was selected")
    current_line_number, current_line_offset = call_gui_callback(logging_console_ctrl.view.get_cursor_position)
    text_of_line_number = call_gui_callback(logging_console_ctrl.view.get_text_of_line, line_number)
    text_of_current_line = call_gui_callback(logging_console_ctrl.view.get_text_of_line, current_line_number)
    assert text_of_current_line == text_of_line_number
    print '#'*20, "\n\nThis is focused {0} \n\n".format(text_of_current_line), '#'*20

    logger.debug("1 test if cursor line is constant for change to 'CONSOLE_FOLLOW_LOGGING' True")
    call_gui_callback(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', True)
    current_line_number, current_line_offset = call_gui_callback(logging_console_ctrl.view.get_cursor_position)
    current_lenght = call_gui_callback(logging_console_ctrl.view.len)
    assert line_number == current_line_number
    # 1.1 test check if scrollbar is on max position"
    check_scrollbar_adjustment_to_be_at_bottom()

    logger.debug("2 test if cursor line is constant for change to 'CONSOLE_FOLLOW_LOGGING' False")
    call_gui_callback(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', False)
    current_line_number, current_line_offset = call_gui_callback(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number
    # TODO #1 check that the scrollbar position allows to see the cursor

    logger.debug("3 test if cursor line is constant for active logging")
    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, state_machine)
    call_gui_callback(rafcon.core.singleton.state_machine_manager.__setattr__, "active_state_machine_id", first_sm_id)

    current_line_number, current_line_offset = call_gui_callback(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number

    call_gui_callback(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', True)

    call_gui_callback(menubar_ctrl.on_new_activate, None)

    current_line_number, current_line_offset = call_gui_callback(logging_console_ctrl.view.get_cursor_position)
    assert line_number == current_line_number
    # 3.1 test check if scrollbar is on max position"
    check_scrollbar_adjustment_to_be_at_bottom()

    # TODO #1 check for recovery onto close by logger messages if current line type is disabled

    # check bug case -> config will not be written to file system if logging parameters are changed
    with open(global_gui_config.config_file_path, 'r') as f:
        config_file_start = f.read()

    call_gui_callback(global_gui_config.set_config_value, 'CONSOLE_FOLLOW_LOGGING', False)

    with open(global_gui_config.config_file_path, 'r') as f:
        config_file_end = f.read()

    assert config_file_end == config_file_start

    print "finished debug console test"


def test_logging_view_widget(caplog):
    from os.path import join

    change_in_gui_config = {'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False,
                            "CONSOLE_FOLLOW_LOGGING": False,
                            "LOGGING_SHOW_VERBOSE": True, "LOGGING_SHOW_DEBUG": True,
                            "LOGGING_SHOW_WARNING": True, "LOGGING_SHOW_ERROR": True}

    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}

    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries)

    try:
        trigger_logging_view_gui_signals()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)

if __name__ == '__main__':
    # test_gui(None)
    pytest.main(['-s', __file__])
