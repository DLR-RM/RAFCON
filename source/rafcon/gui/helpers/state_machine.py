# Copyright (C) 2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""This module covers functionality which is state machine model related, e.g. use selection, dialogs ,storage and
   that are basically menu bar functions. Further the it holds methods that are not StateModel based and more generic.
   Additional this module holds methods that employing the state machine manager. Maybe this changes in future.
"""

import copy

import gtk

import rafcon.gui.helpers.state as gui_helper_state
import rafcon.gui.singleton
from rafcon.core import interface, id_generator
from rafcon.core.singleton import library_manager
from rafcon.core.singleton import state_machine_manager, state_machine_execution_engine
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import State, StateType
from rafcon.core.storage import storage
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.config import global_gui_config
from rafcon.gui.controllers.state_substitute import StateSubstituteChooseLibraryDialog
from rafcon.gui.models import AbstractStateModel, StateModel, ContainerStateModel, LibraryStateModel, TransitionModel, \
    DataFlowModel, DataPortModel, ScopedVariableModel, OutcomeModel, StateMachineModel, get_state_model_class_for_state
from rafcon.gui.utils.dialog import RAFCONButtonDialog, RAFCONCheckBoxTableDialog
from rafcon.utils import log

logger = log.get_logger(__name__)

# TODO think about to generate a state machine manager helper to separate this functions from this module


def new_state_machine(menubar=None):
    if not menubar:
        error_no_menubar("new_state_machine")
        return
    import glib
    logger.debug("Creating new state-machine...")
    root_state = HierarchyState("new root state")
    state_machine = StateMachine(root_state)
    state_machine_manager.add_state_machine(state_machine)
    state_machine_manager.activate_state_machine_id = state_machine.state_machine_id
    state_machine_m = menubar.model.get_selected_state_machine_model()
    # If idle_add isn't used, gaphas crashes, as the view is not ready
    glib.idle_add(state_machine_m.selection.set, state_machine_m.root_state)

    def grab_focus():
        editor_controller = menubar.state_machines_editor_ctrl.get_controller(state_machine.state_machine_id)
        editor_controller.view.editor.grab_focus()

    # The editor parameter of view is created belated, thus we have to use idle_add again
    glib.idle_add(grab_focus)


def open_state_machine(path=None):
    if path is None:
            if interface.open_folder_func is None:
                logger.error("No function defined for opening a folder")
                return
            load_path = interface.open_folder_func("Please choose the folder of the state machine")
            if load_path is None:
                return
    else:
        load_path = path

    try:
        state_machine = storage.load_state_machine_from_path(load_path)
        state_machine_manager.add_state_machine(state_machine)
    except (AttributeError, ValueError, IOError) as e:
        logger.error('Error while trying to open state machine: {0}'.format(e))


def save_state_machine(menubar, widget, save_as=False, delete_old_state_machine=False):
    def on_message_dialog_response_signal(widget, response_id, source_editor_ctrl):
        state = source_editor_ctrl.model.state
        if response_id == 1:
            logger.debug("Applying source code changes of state '{}'".format(state.name))
            source_editor_ctrl.apply_clicked(None)

        elif response_id == 2:
            logger.debug("Ignoring source code changes of state '{}'".format(state.name))
        else:
            logger.warning("Response id: {} is not considered".format(response_id))
            return
        widget.destroy()

    state_machine_m = menubar.model.get_selected_state_machine_model()
    if state_machine_m is None:
        logger.warning("Can not 'save state machine' because no state machine is selected.")
        return

    all_tabs = menubar.states_editor_ctrl.tabs.values()
    all_tabs.extend(menubar.states_editor_ctrl.closed_tabs.values())
    dirty_source_editor_ctrls = [tab_dict['controller'].get_controller('source_ctrl') for tab_dict in all_tabs if
                                 tab_dict['source_code_view_is_dirty'] is True and
                                 tab_dict['state_m'].state.get_state_machine().state_machine_id ==
                                 state_machine_m.state_machine.state_machine_id]

    for dirty_source_editor_ctrl in dirty_source_editor_ctrls:
        state = dirty_source_editor_ctrl.model.state
        message_string = "The source code of the state '{}' (path: {}) has net been applied yet and would " \
                         "therefore not be stored.\n\nDo you want to apply the changes now?".format(state.name,
                                                                                                 state.get_path())
        if global_gui_config.get_config_value("AUTO_APPLY_SOURCE_CODE_CHANGES", False):
            dirty_source_editor_ctrl.apply_clicked(None)
        else:
            RAFCONButtonDialog(message_string, ["Apply", "Ignore changes"],
                               callback=on_message_dialog_response_signal, callback_args=[dirty_source_editor_ctrl],
                               message_type=gtk.MESSAGE_WARNING, parent=menubar.get_root_window())

    save_path = state_machine_m.state_machine.file_system_path
    if save_path is None:
        if not menubar.on_save_as_activate(widget, data=None):
            return

    logger.debug("Saving state machine to {0}".format(save_path))

    state_machine = menubar.model.get_selected_state_machine_model().state_machine
    storage.save_state_machine_to_path(state_machine, state_machine.file_system_path,
                                       delete_old_state_machine=delete_old_state_machine, save_as=save_as)

    menubar.model.get_selected_state_machine_model().store_meta_data()
    logger.debug("Successfully saved state machine and its meta data.")
    return True


def save_state_machine_as(menubar=None, widget=None, data=None, path=None):
    if not menubar:
        error_no_menubar("save_state_machine_as")
        return
    if menubar.model.get_selected_state_machine_model() is None:
        logger.warning("Can not 'save state machine as' because no state machine is selected.")
        return

    if path is None:
        if interface.create_folder_func is None:
            logger.error("No function defined for creating a folder")
            return False
        state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
        sm_m = state_machine_manager_model.state_machines[state_machine_manager_model.selected_state_machine_id]
        folder_name = sm_m.state_machine.root_state.name if sm_m else ''
        path = interface.create_folder_func("Please choose a root folder and a name for the state-machine",
                                            folder_name)
        if path is None:
            logger.warning("No valid path specified")
            return False

    menubar.model.get_selected_state_machine_model().state_machine.file_system_path = path
    save_state_machine(menubar=menubar, widget=widget, save_as=True, delete_old_state_machine=True)


def save_selected_state_as():
    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    selected_states = state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    state_machine_id = state_machine_manager_model.get_selected_state_machine_model().state_machine.state_machine_id
    if selected_states and len(selected_states) == 1:
        state_m = copy.copy(selected_states[0])
        sm_m = StateMachineModel(StateMachine(root_state=state_m.state), state_machine_manager_model)
        sm_m.root_state = state_m
        path = interface.create_folder_func("Please choose a root folder and a name for the state-machine",
                                            selected_states[0].state.name)
        if path:
            storage.save_state_machine_to_path(sm_m.state_machine, base_path=path, save_as=True)
            sm_m.store_meta_data()
        else:
            logger.warning("No valid path specified")
            return False

        def open_as_state_machine_saved_state_as_separate_state_machine():
            logger.debug("Open state machine.")
            try:
                state_machine = storage.load_state_machine_from_path(path)
                state_machine_manager.add_state_machine(state_machine)
            except (ValueError, IOError) as e:
                logger.error('Error while trying to open state machine: {0}'.format(e))

        # check if state machine is in library path
        root_window = rafcon.gui.singleton.main_window_controller.get_root_window()
        if library_manager.is_os_path_within_library_root_paths(path):

            _library_path, _library_name = \
                library_manager.get_library_path_and_name_for_os_path(sm_m.state_machine.file_system_path)
            overwrote_old_lib = library_manager.is_library_in_libraries(_library_path, _library_name)

            message_string = "You stored your state machine in a path that is within the library root paths. " \
                             "Thereby your state machine can be used as a library state.\n\n"\
                             "Do you want to:"

            table_header = ["Option", "Description"]
            table_data = [(True, "Substitute the original state by this new library state."),
                          (True, "Open the newly created library state machine.")]
            if overwrote_old_lib:
                table_data.append((False, "Refresh all open state machines, as an already existing library was "
                                          "overwritten."))

            dialog = RAFCONCheckBoxTableDialog(message_string,
                                               button_texts=("Apply", "Cancel"),
                                               table_header=table_header, table_data=table_data,
                                               message_type=gtk.MESSAGE_QUESTION,
                                               parent=root_window,
                                               width=800, standalone=False)
            response_id = dialog.run()
            if response_id == 1:  # Apply pressed

                if overwrote_old_lib and dialog.list_store[2][0]:  # refresh all open state machine selected
                    logger.debug("Refresh all is triggered.")
                    refresh_all(rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller'))
                else:  # if not all was refreshed at least the libraries are refreshed
                    logger.debug("Library refresh is triggered.")
                    refresh_libraries()

                if dialog.list_store[0][0]:  # Substitute saved state with Library selected
                    logger.debug("Substitute saved state with Library.")
                    if dialog.list_store[0][0] or dialog.list_store[0][1]:
                        refresh_libraries()
                    state_machine_manager_model.selected_state_machine_id = state_machine_id
                    [library_path, library_name] = library_manager.get_library_path_and_name_for_os_path(path)
                    state = library_manager.get_library_instance(library_path, library_name)
                    try:
                        substitute_selected_state(state, as_template=False)
                    except ValueError as e:
                        logger.error('Error while trying to open state machine: {0}'.format(e))
                if dialog.list_store[1][0]:  # Open as state machine saved state as separate state machine selected
                    open_as_state_machine_saved_state_as_separate_state_machine()
            elif response_id in [2, -4]:  # Cancel or Close pressed
                pass
            else:
                raise ValueError("Response id: {} is not considered".format(response_id))
            dialog.destroy()
        else:
            # Offer to open saved state machine dialog
            message_string = "Should the newly created state machine be opened?"
            dialog = RAFCONButtonDialog(message_string, ["Open", "Do not open"],
                                        message_type=gtk.MESSAGE_QUESTION,
                                        parent=root_window)
            response_id = dialog.run()
            if response_id == 1:  # Apply pressed
                open_as_state_machine_saved_state_as_separate_state_machine()
            elif response_id in [2, -4]:  # Cancel or Close pressed
                pass
            else:
                raise ValueError("Response id: {} is not considered".format(response_id))
            dialog.destroy()

        return True
    else:
        logger.warning("Multiple states can not be saved as state machine directly. Group them before.")
        return False


def refresh_libraries():
    library_manager.refresh_libraries()


def refresh_selected_state_machine(menubar):
    """Reloads the selected state machine.
    """

    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    selected_sm = state_machine_manager.state_machines[selected_sm_id]

    # check if the state machine is still running
    if not menubar.state_machine_execution_engine.finished_or_stopped:
        if selected_sm_id == state_machine_execution_engine.active_state_machine_id:
            if menubar.stopped_state_machine_to_proceed():
                pass  # state machine was stopped, proceeding reloading library
            else:
                return

    # check if the a dirty flag is still set
    all_tabs = menubar.states_editor_ctrl.tabs.values()
    all_tabs.extend(menubar.states_editor_ctrl.closed_tabs.values())
    dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                           tab_dict['source_code_view_is_dirty'] is True]
    if selected_sm.marked_dirty or dirty_source_editor:

        def on_message_dialog_response_signal(widget, response_id):
            if response_id == 1:
                menubar.refresh_state_machine_by_id(selected_sm_id)
            else:
                logger.debug("Refresh of selected state machine canceled")
            widget.destroy()

        message_string = "Are you sure you want to reload the currently selected state machine?\n\n" \
                         "The following elements have been modified and not saved. " \
                         "These changes will get lost:"
        message_string = "%s\n* State machine #%s and name '%s'" % (
            message_string, str(selected_sm_id), selected_sm.root_state.name)
        for ctrl in dirty_source_editor:
            if ctrl.model.state.get_state_machine().state_machine_id == selected_sm_id:
                message_string = "%s\n* Source code of state with name '%s' and path '%s'" % (
                    message_string, ctrl.model.state.name, ctrl.model.state.get_path())
        RAFCONButtonDialog(message_string, ["Reload anyway", "Cancel"], on_message_dialog_response_signal,
                           message_type=gtk.MESSAGE_WARNING, parent=menubar.get_root_window())
    else:
        menubar.refresh_state_machine_by_id(selected_sm_id)


def refresh_all(menubar=None, force=False):
    """Reloads all libraries and thus all state machines as well.
        :param menubar: the menubar where this method gets called from
        :param widget: the main widget
        :param data: optional data
    """
    if not menubar:
        error_no_menubar("refresh_all")
        return

    if force:
            menubar.refresh_libs_and_state_machines()
    else:

        # check if a state machine is still running
        if not menubar.state_machine_execution_engine.finished_or_stopped:
            if menubar.stopped_state_machine_to_proceed():
                pass  # state machine was stopped, proceeding reloading library
            else:
                return

        # check if the a dirty flag is still set
        all_tabs = menubar.states_editor_ctrl.tabs.values()
        all_tabs.extend(menubar.states_editor_ctrl.closed_tabs.values())
        dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                               tab_dict['source_code_view_is_dirty'] is True]
        if state_machine_manager.has_dirty_state_machine() or dirty_source_editor:

            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 1:
                    menubar.refresh_libs_and_state_machines()
                else:
                    logger.debug("Refresh canceled")
                widget.destroy()

            message_string = "Are you sure you want to reload the libraries and all state machines?\n\n" \
                             "The following elements have been modified and not saved. " \
                             "These changes will get lost:"
            for sm_id, sm in state_machine_manager.state_machines.iteritems():
                if sm.marked_dirty:
                    message_string = "%s\n* State machine #%s and name '%s'" % (
                        message_string, str(sm_id), sm.root_state.name)
            for ctrl in dirty_source_editor:
                message_string = "%s\n* Source code of state with name '%s' and path '%s'" % (
                    message_string, ctrl.model.state.name, ctrl.model.state.get_path())
            RAFCONButtonDialog(message_string, ["Reload anyway", "Cancel"], on_message_dialog_response_signal,
                               message_type=gtk.MESSAGE_WARNING, parent=menubar.get_root_window())
        else:
            menubar.refresh_libs_and_state_machines()


def error_no_menubar(method_name="unspecified"):
    logger.error("Method '{0}' not called from a menubar, behaviour not specified".format(method_name))


def delete_model(model, raise_exceptions=False):
    """Deletes a model of its state machine

    If the model is one of state, data flow or transition, it is tried to delete that model together with its
    data from the corresponding state machine.

    :param model: The model to delete
    :param bool raise_exceptions: Whether to raise exceptions or only log errors in case of failures
    :return: True if successful, False else
    """
    state_m = model.parent
    if state_m is None:
        msg = "Model has no parent from which it could be deleted from"
        if raise_exceptions:
            raise ValueError(msg)
        logger.error(msg)
        return False
    assert isinstance(state_m, StateModel)
    state = state_m.state
    core_element = model.core_element

    try:
        if core_element in state:
            state.remove(core_element)
            return True
        return False
    except (AttributeError, ValueError) as e:
        if raise_exceptions:
            raise
        logger.error("The model '{}' for core element '{}' could not be deleted: {}".format(model, core_element, e))
        return False


def delete_models(models, raise_exceptions=False):
    """Deletes all given models from their state machines

    Calls the :func:`delete_model` for all models given.

    :param models: A single model or a list of models to be deleted
    :return: The number of models that were successfully deleted
    """
    num_deleted = 0
    # If only one model is given, make a list out of it
    if not isinstance(models, list):
        models = [models]
    for model in models:
        if delete_model(model, raise_exceptions):
            num_deleted += 1
    return num_deleted


def delete_selected_elements(state_machine_m):
    if len(state_machine_m.selection.get_all()) > 0:
        delete_models(state_machine_m.selection.get_all())
        state_machine_m.selection.clear()
        return True


def paste_into_selected_state(state_machine_m):
    selection = state_machine_m.selection
    selected_states = selection.get_states()
    if len(selection) != 1 or len(selected_states) < 1:
        logger.error("Please select a single container state for pasting the clipboard")
        return

    # Note: in multi-selection case, a loop over all selected items is necessary instead of the 0 index
    target_state_m = selection.get_states()[0]
    global_clipboard.paste(target_state_m)


def selected_state_toggle_is_start_state():

    if rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model() is None:
        logger.warning("No state machine has been selected.")
        return False
    state_m_list = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if len(state_m_list) == 1 and isinstance(state_m_list[0], AbstractStateModel) and \
            not state_m_list[0].state.is_root_state:
        state_model = state_m_list[0]
        try:
            if not state_model.is_start:
                state_model.parent.state.start_state_id = state_model.state.state_id
                logger.debug("New start state '{0}'".format(state_model.state.name))
            else:
                state_model.parent.state.start_state_id = None
                logger.debug("Start state unset, no start state defined")
        except ValueError as e:
            logger.warn("Could no change start state: {0}".format(e))
        return True
    else:
        logger.warning("To toggle the is start state flag you have to select exact on state.")
        return False


def add_new_state(state_machine_m, state_type):
    """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

    Adds a new state only if the parent state (selected state) is a container state, and if the graphical editor or
    the state machine tree are in focus.
    """
    assert isinstance(state_machine_m, StateMachineModel)

    if state_type not in list(StateType):
        state_type = StateType.EXECUTION

    selected_state_models = state_machine_m.selection.get_states()
    if not selected_state_models or len(selected_state_models) != 1:
        logger.warn("Please select exactly one desired parent state, before adding a new state")
        return
    model = selected_state_models[0]

    if isinstance(model, StateModel):
        return gui_helper_state.add_state(model, state_type)
    if isinstance(model, (TransitionModel, DataFlowModel)) or \
            isinstance(model, (DataPortModel, OutcomeModel)) and isinstance(model.parent, ContainerStateModel):
        return gui_helper_state.add_state(model.parent, state_type)


def reduce_to_parent_states(models):
    models_to_remove = []
    for model in models:
        parent_m = model.parent
        while parent_m is not None:
            if parent_m in models:
                models_to_remove.append(model)
                break
            parent_m = parent_m.parent
    for model in models_to_remove:
        models.remove(model)
    return models


def insert_state(state, as_template=False):
    """Adds a State to the selected state

    :param state: the state which is inserted
    :param as_template:
    :return: boolean: success of the insertion
    """
    smm_m = rafcon.gui.singleton.state_machine_manager_model

    if not isinstance(state, State):
        logger.error("A state is needed to be insert not {0}".format(state))
        return False

    if not smm_m.selected_state_machine_id:
        logger.error("Please select a container state within a state machine first")
        return False

    selected_state_models = smm_m.state_machines[smm_m.selected_state_machine_id].selection.get_states()
    if len(selected_state_models) > 1:
        logger.error("Please select exactly one state for the insertion")
        return False

    if len(selected_state_models) == 0:
        logger.error("Please select a state for the insertion")
        return False

    current_state_m = selected_state_models[0]
    current_state = current_state_m.state
    if not isinstance(current_state, ContainerState):
        logger.error("States can only be inserted in container states")
        return False

    if not as_template:
        new_state_m = get_state_model_class_for_state(state)(state)
        gui_helper_state.gui_helper_meta_data.put_default_meta_on_state_m(new_state_m, current_state_m)
        new_state = state
    # If inserted as template, we have to extract the state_copy and load the meta data manually
    else:
        new_state_m = LibraryStateModel(state).state_copy
        new_state = state.state_copy

        gaphas_editor, _ = gui_helper_state.gui_helper_meta_data.get_y_axis_and_gaphas_editor_flag()
        previous_state_size = new_state_m.get_meta_data_editor(gaphas_editor)['size']
        gui_helper_state.gui_helper_meta_data.put_default_meta_on_state_m(new_state_m, current_state_m)

        gui_helper_state.prepare_state_m_for_insert_as(new_state_m, previous_state_size)

    current_state_m.expected_future_models.add(new_state_m)
    while new_state.state_id in current_state.states:
        new_state.change_state_id()

    current_state.add_state(new_state)

    return True


def add_state_by_drag_and_drop(state, data):
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    ctrl_path = ['state_machines_editor_ctrl', selected_sm_id]
    state_machine_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller_by_path(ctrl_path)
    state_machine_editor_ctrl.perform_drag_and_drop = True
    if insert_state(state, False):
        data.set_text(state.state_id)
    state_machine_editor_ctrl.perform_drag_and_drop = False


def add_data_port_to_selected_states(data_port_type, data_type=None):
    data_type = 'int' if data_type is None else data_type
    for state_m in rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states():
        # save name with generated data port id
        data_port_id = id_generator.generate_data_port_id(state_m.state.get_data_port_ids())
        if data_port_type == 'INPUT':
            name = 'input_' + str(data_port_id)
            try:
                state_m.state.add_input_data_port(name=name, data_type=data_type, data_port_id=data_port_id)
            except ValueError as e:
                logger.warn("The input data port couldn't be added: {0}".format(e))
        elif data_port_type == 'OUTPUT':
            name = 'output_' + str(data_port_id)
            try:
                state_m.state.add_output_data_port(name=name, data_type=data_type, data_port_id=data_port_id)
            except ValueError as e:
                logger.warn("The output data port couldn't be added: {0}".format(e))
        else:
            return
    return


def add_scoped_variable_to_selected_states(data_type=None):
    data_type = 'int' if data_type is None else data_type
    selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()

    if all([not isinstance(state_m.state, ContainerState) for state_m in selected_states]):
        logger.warn("The scoped variable couldn't be added to state of type {0}"
                    "".format(selected_states[0].state.__class__.__name__))
        return

    for state_m in selected_states:
        if isinstance(state_m.state, ContainerState):
            # save name with generated data port id
            data_port_id = id_generator.generate_data_port_id(state_m.state.get_data_port_ids())
            try:
                state_m.state.add_scoped_variable("scoped_{0}".format(data_port_id), data_type, 0)
            except ValueError as e:
                logger.warn("The scoped variable couldn't be added: {0}".format(e))
    return


def add_outcome_to_selected_states():
    for state_m in rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states():
        # save name with generated outcome id
        outcome_id = id_generator.generate_outcome_id(state_m.state.outcomes.keys())
        name = "outcome_" + str(outcome_id)
        try:
            state_m.state.add_outcome(name=name, outcome_id=outcome_id)
        except ValueError as e:
            logger.warn("The outcome couldn't be added: {0}".format(e))
    return


def change_state_type_with_error_handling_and_logger_messages(state_m, target_class):
    if not isinstance(state_m.state, target_class):
        logger.debug("Change type of State '{0}' from {1} to {2}".format(state_m.state.name,
                                                                         type(state_m.state).__name__,
                                                                         target_class.__name__))
        try:
            state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_state_machine_model(state_m)
            state_machine_m.selection.remove(state_m)
            new_state_m = gui_helper_state.change_state_type(state_m, target_class)
            state_machine_m.selection.set([new_state_m, ])
        except Exception as e:
            logger.exception("An error occurred while changing the state type")
    else:
        logger.info("State type of State '{0}' will not change because target_class: {1} == state_class: {2}"
                    "".format(state_m.state.name, type(state_m.state).__name__, target_class.__name__))


def substitute_selected_state_and_use_choice_dialog():
    selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1:
        StateSubstituteChooseLibraryDialog(rafcon.gui.singleton.library_manager_model,
                                           parent=rafcon.gui.singleton.main_window_controller.get_root_window())
        return True
    else:
        logger.warning("Substitute state needs exact one state to be selected.")
        return False


def substitute_selected_state(state, as_template=False):
    # print "substitute_selected_state", state, as_template
    assert isinstance(state, State)
    from rafcon.core.states.barrier_concurrency_state import DeciderState
    if isinstance(state, DeciderState):
        raise ValueError("State of type DeciderState can not be substituted.")

    smm_m = rafcon.gui.singleton.state_machine_manager_model

    if not smm_m.selected_state_machine_id:
        logger.error("Please select a container state within a state machine first")
        return False

    current_selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
    selected_state_models = current_selection.get_states()
    if len(selected_state_models) > 1:
        logger.error("Please select exactly one state for the substitution")
        return False

    if len(selected_state_models) == 0:
        logger.error("Please select a state for the substitution")
        return False

    current_state_m = selected_state_models[0]
    current_state = current_state_m.state
    current_state_name = current_state.name
    parent_state_m = current_state_m.parent
    parent_state = current_state.parent

    if not as_template:
        state_m = get_state_model_class_for_state(state)(state)
        gui_helper_state.substitute_state(parent_state_m.states[current_state.state_id], state_m)
        state.name = current_state_name
        return True
    # If inserted as template, we have to extract the state_copy and load the meta data manually
    else:
        assert isinstance(state, LibraryState)
        # print "as template1", parent_state_m.states[current_state.state_id].get_meta_data_editor()
        template_m = LibraryStateModel(state).state_copy
        # print "as template2", template_m
        gui_helper_state.substitute_state(parent_state_m.states[current_state.state_id], template_m)
        # template = template_m.state
        # template.change_state_id()
        # template.name = current_state_name

        return True


def substitute_selected_library_state_with_template():
    selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1 and isinstance(selected_states[0], LibraryStateModel):
        # print "start substitute library state with template"
        lib_state = LibraryState.from_dict(LibraryState.state_to_dict(selected_states[0].state))
        # lib_state_m = copy.deepcopy(selected_states[0].state)
        substitute_selected_state(lib_state, as_template=True)
        return True
    else:
        logger.warning("Substitute library state with template needs exact one library state to be selected.")
        return False


def group_selected_states_and_scoped_variables():
    logger.debug("try to group")
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    selected_state_m_list = sm_m.selection.get_states()
    selected_sv_m = [elem for elem in sm_m.selection.get_all() if isinstance(elem, ScopedVariableModel)]
    if selected_state_m_list and isinstance(selected_state_m_list[0].parent, StateModel) or selected_sv_m:
        # # check if all elements have the same parent or leave it to the parent
        # parent_list = []
        # for state_m in selected_state_m_list:
        #     parent_list.append(state_m.state)
        # for sv_m in selected_sv_m:
        #     parent_list.append(sv_m.scoped_variable.parent)
        # assert len(set(parent_list))
        logger.debug("do group selected states: {0} scoped variables: {1}".format(selected_state_m_list, selected_sv_m))
        # TODO remove un-select workaround (used to avoid wrong selections in gaphas and inconsistent selection)
        sm_m.selection.set([])
        gui_helper_state.group_states_and_scoped_variables(selected_state_m_list, selected_sv_m)


def ungroup_selected_state():
    logger.debug("try to ungroup")
    selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if len(selected_states) == 1 and isinstance(selected_states[0], ContainerStateModel) and \
            not selected_states[0].state.is_root_state:
        logger.debug("do ungroup")
        gui_helper_state.ungroup_state(selected_states[0])
