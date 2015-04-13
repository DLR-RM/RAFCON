import sys
import logging
import gtk
import threading
import time
import glib
import os
import signal

from awesome_tool.utils import log
from awesome_tool.mvc.models import ContainerStateModel, StateModel, GlobalVariableManagerModel
from awesome_tool.mvc.controllers import MainWindowController, StateDataPortEditorController,\
    SingleWidgetWindowController, SourceEditorController
from awesome_tool.mvc.views.main_window import MainWindowView
from awesome_tool.mvc.views import LoggingView, StateDataportEditorView, SingleWidgetWindowView, SourceEditorView
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.execution_state import ExecutionState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


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

def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def trigger_gui_signals(*args):
    print "Wait for the gui to initialize"
    time.sleep(2.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    glib.idle_add(menubar_ctrl.on_step_mode_activate, None, None)
    number_of_steps = 16
    sleep_time = 0.5
    time.sleep(sleep_time)
    for i in range(number_of_steps):
        glib.idle_add(menubar_ctrl.on_step_activate, None, None)
        time.sleep(sleep_time)

    for i in range(number_of_steps):
        glib.idle_add(menubar_ctrl.on_backward_step_activate, None, None)
        time.sleep(sleep_time)

    for i in range(number_of_steps):
        glib.idle_add(menubar_ctrl.on_step_activate, None, None)
        time.sleep(sleep_time)

    sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
    time.sleep(sleep_time)

    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_number":
            assert sd.value == 94
        elif sd.name == "wine_number":
            assert sd.value == 34
        elif sd.name == "whiskey_number":
            assert sd.value == 14

    glib.idle_add(menubar_ctrl.on_stop_activate, None)
    glib.idle_add(menubar_ctrl.on_quit_activate, None)


def test_backward_stepping():

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir("../awesome_tool/mvc/")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)
    logging_view = LoggingView()
    setup_logger(logging_view)

    logger, gvm_model = create_models()

    awesome_tool.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/backward_step_barrier_test")

    main_window_view = MainWindowView(logging_view)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = StateMachineManagerModel(awesome_tool.statemachine.singleton.state_machine_manager)

    # load the meta data for the state machine
    sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, gvm_model,
                                                  editor_type="LogicDataGrouped")
    thread = threading.Thread(target=trigger_gui_signals, args=[sm_manager_model, main_window_controller])
    thread.start()

    gtk.main()
    logger.debug("Gtk main loop exited!")
    sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()
    os.chdir("../../test")
    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_backward_stepping()