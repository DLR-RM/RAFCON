
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
    # core elements
    from rafcon.core.singleton import state_machine_manager
    # gui elements
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.controllers.state_editor.semantic_data_editor import SemanticDataEditorController

    menu_bar_controller = gui_singleton.main_window_controller.get_controller("menu_bar_controller")
    # All executions from a thread which is not the gtk main thread must be called via "idle_add" or call_gui_callback
    # see https://developer.gnome.org/gdk3/stable/gdk3-Threads.html#gdk-threads-add-idle-full
    # and https://stackoverflow.com/questions/35700140/pygtk-run-gtk-main-loop-in-a-seperate-thread
    call_gui_callback(menu_bar_controller.on_new_activate)

    state_machine = state_machine_manager.get_active_state_machine()
    root_state = state_machine.root_state
    call_gui_callback(initialize_data, root_state)

    state_machine_model = gui_singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    states_editor_controller = gui_singleton.main_window_controller.get_controller("states_editor_ctrl")

    page_info, state_identifier = states_editor_controller.find_page_of_state_m(state_machine_model.root_state)
    # print page_info, state_identifier

    state_editor_controller = states_editor_controller.tabs[state_identifier]["controller"]
    semantic_data_controller = state_editor_controller.get_controller("semantic_data_ctrl")

    assert isinstance(semantic_data_controller, SemanticDataEditorController)
    # TODO: why is this reload necessary?
    call_gui_callback(semantic_data_controller.reload_tree_store_data)
    call_gui_callback(semantic_data_controller.select_entry, ['key 2'])
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
    assert len(root_state.semantic_data.keys()) == 5
    call_gui_callback(semantic_data_controller.select_entry, ['dict 2'])
    # semantic_data_controller.select_entry(['dict 2'])
    call_gui_callback(semantic_data_controller.on_remove, None)
    assert len(root_state.semantic_data.keys()) == 4 and 'dict 2' not in root_state.semantic_data.keys()

    # test add normal entry into first hierarchy
    call_gui_callback(semantic_data_controller.tree_view.get_selection().select_path, tree_test_path)
    call_gui_callback(semantic_data_controller.on_add, None, False)
    assert len(root_state.semantic_data.keys()) == 5

    # test add dictionary entry in second hierarchy
    # tree_test_path = semantic_data_controller.get_path_for_core_element(["dict 1"])
    # print "second hierarchy: ", tree_test_path
    assert len(root_state.semantic_data["dict 1"].keys()) == 2
    # call_gui_callback(semantic_data_controller.tree_view.get_selection().select_path, tree_test_path)
    call_gui_callback(semantic_data_controller.select_entry, ['dict 1'])
    call_gui_callback(semantic_data_controller.on_add, None, False)
    assert len(root_state.semantic_data.keys()) == 5
    assert len(root_state.semantic_data["dict 1"].keys()) == 3


def test_semantic_data(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        change_semantic_data_values()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_semantic_data(None)
    # import pytest
    # pytest.main(['-s', __file__])