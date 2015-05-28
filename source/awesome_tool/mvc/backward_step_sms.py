import logging
import sys
import os
import gtk
import signal

from awesome_tool.utils import log
from awesome_tool.mvc.controllers import MainWindowController
from awesome_tool.mvc.views.logging import LoggingView
from awesome_tool.mvc.views.main_window import MainWindowView
from awesome_tool.mvc.models import GlobalVariableManagerModel
import awesome_tool.statemachine.singleton
import awesome_tool.mvc.singleton


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


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
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, global_var_manager_model


def run_turtle_demo():
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)
    # setup logging view first
    logging_view = LoggingView()
    setup_logger(logging_view)

    awesome_tool.statemachine.singleton.library_manager.initialize()

    # set base path of global storage
    awesome_tool.statemachine.singleton.global_storage.base_path = "../../test_scripts/tutorials/basic_turtle_demo_sm"

    [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/error_propagation_test")

    # [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
    #     global_storage.load_statemachine_from_yaml("../../test_scripts/backward_step_barrier_test")

    # [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
    #     global_storage.load_statemachine_from_yaml("../../test_scripts/backward_step_preemption_test")

    # [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
    #     global_storage.load_statemachine_from_yaml("../../test_scripts/backward_step_hierarchy_test")

    # [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
    #     global_storage.load_statemachine_from_yaml("../../test_scripts/backward_step_library_test")

    [logger, gvm_model] = create_models()
    main_window_view = MainWindowView(logging_view)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = awesome_tool.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, gvm_model)

    gtk.main()
    logger.debug("Gtk main loop exited!")

    sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    #print sys.path
    run_turtle_demo()
