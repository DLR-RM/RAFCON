from twisted.internet import gtk2reactor
gtk2reactor.install()

from twisted.internet import reactor

import logging
import sys
import os
import gtk
import signal

from rafcon.utils import log
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
import rafcon.statemachine.singleton
import rafcon.mvc.singleton
from rafcon.mvc.config import global_gui_config
from rafcon.network.network_config import global_net_config
from rafcon.statemachine.config import global_config
from rafcon.statemachine.states.library_state import LibraryState


def setup_logger():
    import sys
    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def run_turtle_demo():
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    # setup logging view first
    setup_logger()
    logger = log.get_logger("turtle demo")

    home_path = os.path.join(os.path.expanduser('~'), '.config/rafcon')
    global_config.load(path=home_path)
    global_gui_config.load(path=home_path)
    global_net_config.load()

    rafcon.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/tutorials/99_bottles_of_beer")

    rafcon.statemachine.singleton.library_manager.initialize()
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type="LogicDataGrouped")
    #main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model)

    reactor.run()
    gtk.main()
    logger.debug("Gtk main loop exited!")

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    run_turtle_demo()
