from twisted.internet import gtk2reactor
gtk2reactor.install()

from twisted.internet import reactor

import logging

import sys
import os
import gtk
import signal
from os.path import join, dirname

from rafcon.utils import log
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.models import GlobalVariableManagerModel
import rafcon.statemachine.singleton
import rafcon.mvc.singleton
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.network.network_config import global_net_config
from rafcon.statemachine.config import global_config


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    global_var_manager_model = GlobalVariableManagerModel()
    return logger, global_var_manager_model


def run_sm():
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)

    home_path = os.path.join(os.path.expanduser('~'), '.config/rafcon')
    global_config.load(path=home_path)
    global_gui_config.load(path=home_path)
    global_net_config.load()
    # global_runtime_config.load(path=home_path)

    library_paths = rafcon.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["unit_test_state_machines"] = join(join(dirname(rafcon.__path__[0]), 'test_scripts'), 'unit_test_state_machines')

    rafcon.statemachine.singleton.library_manager.initialize()

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/unit_test_state_machines/error_propagation_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/return_none_test_sm")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/decider_test_statemachine")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/barrier_concurrency_test_sm")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/backward_step_barrier_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/backward_step_preemption_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/backward_step_hierarchy_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/backward_step_library_test")

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
        global_storage.load_statemachine_from_path("../../test_scripts/unit_test_state_machines/stepping_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/unit_test_state_machines/library_runtime_value_test")

    # [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
    #     global_storage.load_statemachine_from_path("../../test_scripts/tutorials/99_bottles_of_beer_in_library")

    [logger, gvm_model] = create_models()
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, gvm_model)

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
    #print sys.path
    run_sm()
