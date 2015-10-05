import logging
import sys
import os
import gtk
import signal

from rafcon.utils import log
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config
import rafcon.statemachine.singleton
import rafcon.mvc.singleton

from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState


def setup_logger():
    import sys
    # Set the views for the loggers

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
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)

    # setup logging view first
    setup_logger()
    logger = log.get_logger("turtle demo")

    home_path = os.path.join(os.path.expanduser('~'), '.rafcon')
    global_config.load(home_path)
    global_gui_config.load(home_path)

    rafcon.statemachine.singleton.library_manager.initialize()

    # set base path of global storage
    rafcon.statemachine.singleton.global_storage.base_path = "/tmp"

    root_state = HierarchyState()
    state_machine = StateMachine(root_state)

    rafcon.statemachine.singleton.library_manager.initialize()
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type="LogicDataGrouped")

    gtk.main()
    logger.debug("Gtk main loop exited!")

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    run_empty_statemachine()
