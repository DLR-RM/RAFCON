
# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback

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


def test_semantic_data(gui):
    from rafcon.gui.controllers.state_editor.semantic_data_editor import SemanticDataEditorController
    state_machine_manager = gui.core_singletons.state_machine_manager

    menu_bar_controller = gui.singletons.main_window_controller.menu_bar_controller
    # All executions from a thread which is not the gtk main thread must be called via "idle_add" or call_gui_callback
    # see https://developer.gnome.org/gdk3/stable/gdk3-Threads.html#gdk-threads-add-idle-full
    # and https://stackoverflow.com/questions/35700140/pygtk-run-gtk-main-loop-in-a-seperate-thread
    state_machine = gui(menu_bar_controller.on_new_activate)

    root_state = state_machine.root_state
    gui(initialize_data, root_state)

    state_machine_model = gui.singletons.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    states_editor_controller = gui.singletons.main_window_controller.states_editor_ctrl

    page_info, state_identifier = states_editor_controller.get_page_of_state_m(state_machine_model.root_state)
    # print page_info, state_identifier

    state_editor_controller = states_editor_controller.tabs[state_identifier]["controller"]
    semantic_data_controller = state_editor_controller.semantic_data_ctrl

    assert isinstance(semantic_data_controller, SemanticDataEditorController)
    # TODO: why is this reload necessary?
    gui(semantic_data_controller.reload_tree_store_data)
    gui(semantic_data_controller.select_entry, ['key 2'])
    tree_test_path = (1, )
    # semantic_data_controller.tree_view.get_selection().select_path(tree_test_path)

    # wait while an element is selected
    # while True:
    #     treeiter, path = semantic_data_controller.get_selected_object()
    #     # check if an element is selected
    #     if treeiter:
    #         dict_path_as_list = semantic_data_controller.tree_store[path][semantic_data_controller.ID_STORAGE_ID]
    #         # semantic_data_controller.model.state.remove_semantic_data(dict_path_as_list)
    #         data = semantic_data_controller.model.state.get_semantic_data(dict_path_as_list)
    #         print data
    #         break
    #     time.sleep(0.1)
    # model, paths = semantic_data_controller.tree_view.get_selection().get_selected_rows()
    # print model, paths
    # print root_state.semantic_data

    # test delete
    assert len(list(root_state.semantic_data.keys())) == 5
    gui(semantic_data_controller.select_entry, ['dict 2'])
    # semantic_data_controller.select_entry(['dict 2'])
    gui(semantic_data_controller.on_remove, None)
    assert len(list(root_state.semantic_data.keys())) == 4 and 'dict 2' not in list(root_state.semantic_data.keys())

    # test add normal entry into first hierarchy
    gui(semantic_data_controller.tree_view.get_selection().select_path, tree_test_path)
    gui(semantic_data_controller.on_add, None, False)
    assert len(list(root_state.semantic_data.keys())) == 5

    # test add dictionary entry in second hierarchy
    # tree_test_path = semantic_data_controller.get_path_for_core_element(["dict 1"])
    # print "second hierarchy: ", tree_test_path
    assert len(list(root_state.semantic_data["dict 1"].keys())) == 2
    # gui(semantic_data_controller.tree_view.get_selection().select_path, tree_test_path)
    gui(semantic_data_controller.select_entry, ['dict 1'])
    gui(semantic_data_controller.on_add, None, False)
    assert len(list(root_state.semantic_data.keys())) == 5
    assert len(list(root_state.semantic_data["dict 1"].keys())) == 3


if __name__ == '__main__':
    test_semantic_data(None)
    # import pytest
    # pytest.main(['-s', __file__])
