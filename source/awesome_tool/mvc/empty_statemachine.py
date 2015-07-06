import logging
import sys
import os
import gtk
import signal

from awesome_tool.utils import log
from awesome_tool.mvc.controllers import MainWindowController
from awesome_tool.mvc.views.logging import LoggingView
from awesome_tool.mvc.views.main_window import MainWindowView
from awesome_tool.mvc.config import global_gui_config
from awesome_tool.statemachine.config import global_config
import awesome_tool.statemachine.singleton
import awesome_tool.mvc.singleton

from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState


def setup_logger(logging_view):
    import sys
    # Set the views for the loggers
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)

    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)

    # Set logging level
    # logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    # logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)


def run_empty_statemachine():
    signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)

    # setup logging view first
    logging_view = LoggingView()
    setup_logger(logging_view)
    setup_logger(logging_view)
    logger = log.get_logger("turtle demo")

    home_path = os.path.join(os.path.expanduser('~'), '.awesome_tool')
    global_config.load(home_path)
    global_gui_config.load(home_path)

    awesome_tool.statemachine.singleton.library_manager.initialize()

    # set base path of global storage
    awesome_tool.statemachine.singleton.global_storage.base_path = "/tmp"

    root_state = HierarchyState()
    state_machine = StateMachine(root_state)

    awesome_tool.statemachine.singleton.library_manager.initialize()
    main_window_view = MainWindowView(logging_view)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = awesome_tool.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type="LogicDataGrouped")

    gtk.main()
    logger.debug("Gtk main loop exited!")

    sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    run_empty_statemachine()
