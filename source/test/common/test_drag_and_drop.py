import logging
import gtk
import threading
import time
import os
import signal

# general tool elements
from rafcon.utils import log

# core elements
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.state_machine import StateMachine

# mvc elements
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.controllers.graphical_editor_gaphas import GraphicalEditorController as \
        GraphicalEditorGaphasController

# singleton elements
import rafcon.mvc.singleton
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config

# test environment elements
import testing_utils
from testing_utils import test_multithrading_lock, call_gui_callback, TMP_TEST_PATH
import pytest


class StructHelper:
    def __init__(self, x, y, text):
        self.x = x
        self.y = y
        self.text = text

    def set_text(self, text):
        self.text = text

    def get_text(self):
        return self.text


def create_models(*args, **kargs):

    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)

    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)

    state1 = HierarchyState('State1', state_id="State1")

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.name = "Container"

    state_dict = {'Container': ctr_state, 'State1': state1}
    sm = StateMachine(ctr_state)

    # remove existing state machines
    for sm_id in rafcon.statemachine.singleton.state_machine_manager.state_machines.keys():
        rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm_id)
    # add new state machine
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)
    # select state machine
    rafcon.mvc.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id
    # get state machine model
    sm_m = rafcon.mvc.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]

    global_var_manager_model = rafcon.mvc.singleton.global_variable_manager_model

    return logger, ctr_state, global_var_manager_model, sm_m, state_dict


@log.log_exceptions(None, gtk_quit=True)
def trigger_drag_and_drop_tests(*args):
    print "Wait for the gui to initialize"
    time.sleep(2.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    logger = args[2]

    rafcon.statemachine.singleton.library_manager.initialize()

    states_machines_editor_controller = main_window_controller.get_controller('state_machines_editor_ctrl')
    library_tree_controller = main_window_controller.get_controller('library_controller')
    time.sleep(1.0)
    graphical_editor_controller = states_machines_editor_controller.get_child_controllers()[0]
    state_icon_controller = main_window_controller.get_controller('state_icon_controller')

    library_tree_controller.view.expand_all()
    tree_model = library_tree_controller.view.get_model()
    library_tree_controller.view.get_selection().select_iter(tree_model.iter_children(tree_model.get_iter_root()))

    selection_data = StructHelper(0, 0, None)

    # insert state in rootstate
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 200, 200, None)
    call_gui_callback(library_tree_controller.on_drag_data_get, library_tree_controller.view, None, selection_data, 0, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 200, 200, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 2

    # insert state from IconView
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 300, 300, None)
    call_gui_callback(state_icon_controller.on_mouse_motion, None, StructHelper(30, 15, None))
    call_gui_callback(state_icon_controller.on_drag_data_get, None, None, selection_data, None, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 300, 300, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 3

    # insert state next to root state
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 700, 300, None)
    call_gui_callback(state_icon_controller.on_mouse_motion, None, StructHelper(30, 15, None))
    call_gui_callback(state_icon_controller.on_drag_data_get, None, None, selection_data, None, None)

    # insert state in state1
    if isinstance(graphical_editor_controller, GraphicalEditorGaphasController):
        call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 100, 100, None)
    else:
        call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 150, 150, None)
    call_gui_callback(library_tree_controller.on_drag_data_get, library_tree_controller.view, None, selection_data, 0, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 20, 20, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states['State1'].states) == 1

    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    menubar_ctrl.model.get_selected_state_machine_model().state_machine.file_system_path = TMP_TEST_PATH + "/test_states_editor_widget"
    call_gui_callback(menubar_ctrl.on_save_activate, None)
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_drag_and_drop_test(caplog):

    test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(testing_utils.RAFCON_PATH + "/mvc")
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    global_config.load()  # load the default config
    global_gui_config.load()  # load the default config

    logger, state, gvm_model, sm_m, state_dict = create_models()

    testing_utils.remove_all_libraries()
    library_paths = rafcon.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["test_libraries"] = testing_utils.get_test_sm_path("test_libraries")

    rafcon.statemachine.singleton.library_manager.initialize()


    if testing_utils.sm_manager_model is None:
            testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_view = MainWindowView()

    # load the meta data for the state machine
    testing_utils.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data()

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                      editor_type='LogicDataGrouped')

    thread = threading.Thread(target=trigger_drag_and_drop_tests,
                              args=[testing_utils.sm_manager_model, main_window_controller, logger])
    thread.start()

    gtk.main()
    logger.debug("Gtk main loop exited!")
    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()
        logger.debug("Joined currently executing state machine!")
        thread.join()
        logger.debug("Joined test triggering thread!")
    os.chdir(testing_utils.TEST_SM_PATH + "/../test/common")
    test_multithrading_lock.release()

    testing_utils.reload_config()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])
