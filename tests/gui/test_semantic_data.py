import os
import time
import datetime
import glib

# state machine elements
from docutils.statemachine import StateMachine
from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
from rafcon.core.storage import storage
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.state_machine import StateMachine

# gui elements
import rafcon.gui.singleton as gui_singleton
from rafcon.gui.controllers.state_editor.semantic_data_editor import SemanticDataEditorController

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_data(state):
    state.semantic_data["key 1"] = "value 1"
    state.semantic_data["key 2"] = "value 2"
    state.semantic_data["key 3"] = "value 3"
    state.semantic_data["dict 1"] = dict()
    state.semantic_data["dict 1"]["dkey 1"] = "dvalue 1"
    state.semantic_data["dict 1"]["dkey 2"] = "dvalue 2"
    state.semantic_data["dict 2"] = dict()


def change_semantic_data_values():
    menu_bar_controller = gui_singleton.main_window_controller.get_controller("menu_bar_controller")
    menu_bar_controller.on_new_activate()
    # call_gui_callback(menu_bar_controller.on_new_activate)
    state_machine = state_machine_manager.state_machines[state_machine_manager.active_state_machine_id]
    root_state = state_machine.root_state
    initialize_data(root_state)

    state_machine_model = gui_singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    states_editor_controller = gui_singleton.main_window_controller.get_controller("states_editor_ctrl")
    # print states_editor_controller
    # print state_machine_model.root_state
    states_editor_controller.add_state_editor(state_machine_model.root_state)
    page_info, state_identifier = states_editor_controller.find_page_of_state_m(state_machine_model.root_state)
    # print page_info, state_identifier

    state_editor_controller = states_editor_controller.tabs[state_identifier]["controller"]
    semantic_data_controller = state_editor_controller.get_controller("semantic_data_ctrl")

    assert isinstance(semantic_data_controller, SemanticDataEditorController)
    tree_test_path = [1]
    # options for selection: select_all, select_iter, select_path, select_range
    semantic_data_controller.tree_view.get_selection().select_path(tuple(tree_test_path))
    model, paths = semantic_data_controller.tree_view.get_selection().get_selected_rows()
    # print model, paths
    # print root_state.semantic_data

    # test delete
    assert len(root_state.semantic_data.keys()) == 5
    semantic_data_controller.on_remove(None)
    assert len(root_state.semantic_data.keys()) == 4

    # test add normal entry into first hierarchy
    semantic_data_controller.tree_view.get_selection().select_path(tuple(tree_test_path))
    semantic_data_controller.on_add(None, False)
    assert len(root_state.semantic_data.keys()) == 5

    # test add dictionary entry in second hierarchy
    tree_test_path = [0]
    assert len(root_state.semantic_data["dict 1"].keys()) == 2
    semantic_data_controller.tree_view.get_selection().select_path(tuple(tree_test_path))
    semantic_data_controller.on_add(None, False)
    assert len(root_state.semantic_data.keys()) == 5
    assert len(root_state.semantic_data["dict 1"].keys()) == 3


def test_semantic_data(caplog):

    testing_utils.run_gui()
    try:
        change_semantic_data_values()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_semantic_data(None)
    # import pytest
    # pytest.main(['-s', __file__])