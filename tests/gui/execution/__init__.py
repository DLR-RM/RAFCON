

def state_machines_editor_tab_status_check(state_machine_id, active):
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.utils import constants
    state_machines_editor_ctrl = gui_singleton.main_window_controller.get_controller('state_machines_editor_ctrl')
    notebook = state_machines_editor_ctrl.view['notebook']
    page = state_machines_editor_ctrl.get_page_for_state_machine_id(state_machine_id)
    if page:
        label = notebook.get_tab_label(page).get_child().get_children()[0]
        tab_is_active = label.get_style_context().has_class(constants.execution_running_style_class)
        if tab_is_active is not active:
            raise RuntimeError('Assume sm_id {0} "ACTIVE" {2} but is {1}"'
                               ''.format(state_machine_id, tab_is_active,  active))
    else:
        raise RuntimeError('The state machine id {0} is not open in the state machines editor controller'
                           ''.format(state_machine_id))
