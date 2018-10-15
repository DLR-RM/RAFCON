
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
import pytest

logger = log.get_logger(__name__)


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
    import rafcon.core.singleton
    import rafcon.gui.singleton
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    state1 = HierarchyState('State1', state_id="State1")

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.name = "Container"

    sm = StateMachine(ctr_state)

    # add new state machine
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    testing_utils.wait_for_gui()
    # select state machine
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id


@log.log_exceptions(None, gtk_quit=True)
def trigger_drag_and_drop_tests(*args):
    # TODO test needs check on position -> is the state drawn where it was dropped?
    sm_manager_model = args[0]
    main_window_controller = args[1]

    states_machines_editor_controller = main_window_controller.get_controller('state_machines_editor_ctrl')
    library_tree_controller = main_window_controller.get_controller('library_controller')
    state_icon_controller = main_window_controller.get_controller('state_icon_controller')
    graphical_editor_controller = states_machines_editor_controller.get_child_controllers()[0]

    # the view is required here; as it is created asynchronously we explicitly wait for its creation
    while not graphical_editor_controller.view:
        testing_utils.wait_for_gui()

    call_gui_callback(graphical_editor_controller.view.editor.set_size_request, 500, 500)
    call_gui_callback(library_tree_controller.view.expand_all)
    # generic and unit_test_state_machines in library tree index 1 is unit_test_state_machines
    call_gui_callback(library_tree_controller.view.get_selection().select_path, (1, 0))

    selection_data = StructHelper(0, 0, None)
    state_machine_m = sm_manager_model.get_selected_state_machine_model()

    # insert state in root_state
    print "insert state in root_state"
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 200, 200, None)
    # Override selection
    state_m = state_machine_m.root_state
    call_gui_callback(state_machine_m.selection.set, [state_m])
    call_gui_callback(library_tree_controller.on_drag_data_get, library_tree_controller.view, None, selection_data, 0, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 200, 200, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 2

    # insert state from IconView
    print "insert state from IconView"
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 300, 300, None)
    # Override selection
    state_m = state_machine_m.root_state
    call_gui_callback(state_machine_m.selection.set, [state_m])
    call_gui_callback(state_icon_controller.on_mouse_motion, None, StructHelper(30, 15, None))
    call_gui_callback(state_icon_controller.on_drag_data_get, None, None, selection_data, None, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 300, 300, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 3

    # insert state next to root state
    print "insert state next to root state"
    # Position (0, 0) in left above the root state
    call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 0, 0, None)
    call_gui_callback(state_icon_controller.on_mouse_motion, None, StructHelper(30, 15, None))
    call_gui_callback(state_icon_controller.on_drag_data_get, None, None, selection_data, None, None)

    # insert state in state1
    print "insert state in state1"
    state_m = state_machine_m.root_state.states['State1']
    call_gui_callback(state_machine_m.selection.set, [state_m])
    # Selecting a state using the drag_motion event is too unreliable, as the exact position depends on the size of
    # the editor. Therefore, it is now selected using the selection object directly
    # if isinstance(graphical_editor_controller, GraphicalEditorGaphasController):
    #     call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 100, 100, None)
    # else:
    #     call_gui_callback(graphical_editor_controller.on_drag_motion, None, None, 150, 150, None)
    call_gui_callback(library_tree_controller.on_drag_data_get, library_tree_controller.view, None, selection_data, 0, None)
    call_gui_callback(graphical_editor_controller.on_drag_data_received, None, None, 20, 20, selection_data, None, None)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states['State1'].states) == 1


def test_drag_and_drop_test(caplog):
    testing_utils.run_gui(
        gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False},
        libraries={"unit_test_state_machines": testing_utils.get_test_sm_path("unit_test_state_machines")}
    )
    import rafcon.core.singleton
    call_gui_callback(create_models)

    try:
        trigger_drag_and_drop_tests(rafcon.gui.singleton.state_machine_manager_model,
                                    rafcon.gui.singleton.main_window_controller)
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=1, expected_errors=0)


if __name__ == '__main__':
    test_drag_and_drop_test(None)
    # pytest.main([__file__, '-xs'])
