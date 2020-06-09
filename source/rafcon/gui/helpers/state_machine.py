# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""This module covers functionality which is state machine model related, e.g. use selection, dialogs ,storage and
   that are basically menu bar functions. Further the it holds methods that are not StateModel based and more generic.
   Additional this module holds methods that employing the state machine manager. Maybe this changes in future.
"""

from builtins import str
import copy
import time
import os
from gi.repository import Gtk

import rafcon.gui.helpers.state as gui_helper_state
import rafcon.gui.singleton

from rafcon.core import interface, id_generator
from rafcon.core.singleton import state_machine_manager, state_machine_execution_engine, library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import State, StateType
from rafcon.core.storage import storage
import rafcon.core.config

from rafcon.gui.helpers.text_formatting import format_default_folder_name
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.config import global_gui_config
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.controllers.state_substitute import StateSubstituteChooseLibraryDialog
from rafcon.gui.models import AbstractStateModel, StateModel, ContainerStateModel, LibraryStateModel, TransitionModel, \
    DataFlowModel, DataPortModel, ScopedVariableModel, OutcomeModel, StateMachineModel
from rafcon.gui.singleton import library_manager_model
from rafcon.gui.utils.dialog import RAFCONButtonDialog, RAFCONCheckBoxTableDialog
from rafcon.utils.filesystem import make_tarfile, copy_file_or_folder, create_path, make_file_executable
from rafcon.utils import log, storage_utils
import rafcon.gui.utils

logger = log.get_logger(__name__)

# TODO think about to generate a state machine manager helper to separate this functions from this module


def is_element_none_with_error_message(method_name, element_dict):
    missing_elements = [element_name for element_name, element in element_dict.items() if element is None]
    if missing_elements:
        logger.error("The following elements are missing to perform {0}: {1}".format(missing_elements))


def new_state_machine(*args):

    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    state_machines_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')

    logger.debug("Creating new state machine...")
    root_state = HierarchyState("root state")
    state_machine = StateMachine(root_state)
    state_machine_manager.add_state_machine(state_machine)

    # this is needed in order that the model is already there, when it is access via get_selected_state_machine_model()
    rafcon.gui.utils.wait_for_gui()
    state_machine_m = state_machine_manager_model.get_selected_state_machine_model()
    state_machine_m.selection.set(state_machine_m.root_state)

    editor_controller = state_machines_editor_ctrl.get_controller(state_machine.state_machine_id)
    editor_controller.view.editor.grab_focus()
    return state_machine


def open_state_machine(path=None, recent_opened_notification=False):
    """ Open a state machine from respective file system path

    :param str path: file system path to the state machine
    :param bool recent_opened_notification: flags that indicates that this call also should update recently open

    :rtype rafcon.core.state_machine.StateMachine
    :return: opened state machine
    """
    start_time = time.time()
    if path is None:
        if interface.open_folder_func is None:
            logger.error("No function defined for opening a folder")
            return
        load_path = interface.open_folder_func("Please choose the folder of the state machine")
        if load_path is None:
            return
    else:
        load_path = path

    if state_machine_manager.is_state_machine_open(load_path):
        logger.info("State machine already open. Select state machine instance from path {0}.".format(load_path))
        sm = state_machine_manager.get_open_state_machine_of_file_system_path(load_path)
        gui_helper_state.gui_singletons.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id
        return state_machine_manager.get_open_state_machine_of_file_system_path(load_path)

    state_machine = None
    try:
        state_machine = storage.load_state_machine_from_path(load_path)
        if not state_machine:
            return  # a corresponding exception has been handled with a proper error log in load_state_machine_from_path
        state_machine_manager.add_state_machine(state_machine)
        if recent_opened_notification:
            global_runtime_config.update_recently_opened_state_machines_with(state_machine)
        duration = time.time() - start_time
        stat = state_machine.root_state.get_states_statistics(0)
        logger.info("It took {0:.2}s to load {1} states with {2} hierarchy levels.".format(duration, stat[0], stat[1]))
    except Exception:
        logger.exception('Error while trying to open state machine')

    return state_machine


def open_library_state_separately():
    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    state_models = state_machine_manager_model.get_selected_state_machine_model().selection.states
    if not state_models:
        logger.info("Please select at least one library state to 'open library state separately'")
        return
    if not all([isinstance(state_m, LibraryStateModel) for state_m in state_models]):
        logger.warning("Please select only library states. "
                       "'Open library state separately' works only for library states.")
        return

    for state_m in state_models:
        try:
            path, _, _ = rafcon.gui.singleton.library_manager.get_os_path_to_library(state_m.state.library_path,
                                                                                     state_m.state.library_name)
            state_machine = open_state_machine(path)
            if state_machine is None:
                logger.warning('Library state {0} could not be open separately'.format(state_m.state))
        except Exception:
            logger.exception('Library state {0} could not be open separately'.format(state_m.state))


def save_state_machine(delete_old_state_machine=False, recent_opened_notification=False, as_copy=False, copy_path=None):
    """ Save selected state machine

     The function checks if states of the state machine has not stored script data abd triggers dialog windows to
     take user input how to continue (ignoring or storing this script changes).
     If the state machine file_system_path is None function save_state_machine_as is used to collect respective path and
     to store the state machine.
     The delete flag will remove all data in existing state machine folder (if plugins or feature use non-standard
     RAFCON files this data will be removed)

    :param bool delete_old_state_machine: Flag to delete existing state machine folder before storing current version
    :param bool recent_opened_notification: Flag to insert path of state machine into recent opened state machine paths
    :param bool as_copy: Store state machine as copy flag e.g. without assigning path to state_machine.file_system_path
    :return: True if the storing was successful, False if the storing process was canceled or stopped by condition fail
    :rtype bool:
    """

    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    states_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('states_editor_ctrl')

    state_machine_m = state_machine_manager_model.get_selected_state_machine_model()
    if state_machine_m is None:
        logger.warning("Can not 'save state machine' because no state machine is selected.")
        return False

    previous_path = state_machine_m.state_machine.file_system_path
    previous_marked_dirty = state_machine_m.state_machine.marked_dirty
    all_tabs = list(states_editor_ctrl.tabs.values())
    all_tabs.extend(states_editor_ctrl.closed_tabs.values())
    dirty_source_editor_ctrls = [tab_dict['controller'].get_controller('source_ctrl') for tab_dict in all_tabs if
                                 tab_dict['source_code_view_is_dirty'] is True and
                                 tab_dict['state_m'].state.get_state_machine().state_machine_id ==
                                 state_machine_m.state_machine.state_machine_id]

    for dirty_source_editor_ctrl in dirty_source_editor_ctrls:
        state = dirty_source_editor_ctrl.model.state
        message_string = "The source code of the state '{}' (path: {}) has not been applied yet and would " \
                         "therefore not be saved.\n\nDo you want to apply the changes now?" \
                         "".format(state.name, state.get_path())
        if global_gui_config.get_config_value("AUTO_APPLY_SOURCE_CODE_CHANGES", False):
            dirty_source_editor_ctrl.apply_clicked(None)
        else:
            dialog = RAFCONButtonDialog(message_string, ["Apply", "Ignore changes"],
                                        message_type=Gtk.MessageType.WARNING, parent=states_editor_ctrl.get_root_window())
            response_id = dialog.run()
            state = dirty_source_editor_ctrl.model.state
            if response_id == 1:  # Apply changes
                logger.debug("Applying source code changes of state '{}'".format(state.name))
                dirty_source_editor_ctrl.apply_clicked(None)

            elif response_id == 2:  # Ignore changes
                logger.debug("Ignoring source code changes of state '{}'".format(state.name))
            else:
                logger.warning("Response id: {} is not considered".format(response_id))
                return False
            dialog.destroy()

    save_path = state_machine_m.state_machine.file_system_path
    if not as_copy and save_path is None or as_copy and copy_path is None:
        if not save_state_machine_as(as_copy=as_copy):
            return False
        return True

    logger.debug("Saving state machine to {0}".format(save_path))

    state_machine_m = state_machine_manager_model.get_selected_state_machine_model()
    sm_path = state_machine_m.state_machine.file_system_path

    storage.save_state_machine_to_path(state_machine_m.state_machine, copy_path if as_copy else sm_path,
                                       delete_old_state_machine=delete_old_state_machine, as_copy=as_copy)
    if recent_opened_notification:
        global_runtime_config.update_recently_opened_state_machines_with(state_machine_m.state_machine)
    state_machine_m.store_meta_data(copy_path=copy_path if as_copy else None)
    logger.debug("Saved state machine and its meta data.")
    library_manager_model.state_machine_was_stored(state_machine_m, previous_path)
    return True


def save_state_machine_as(path=None, recent_opened_notification=False, as_copy=False):
    """ Store selected state machine to path

     If there is no handed path the interface dialog "create folder" is used to collect one. The state machine finally
     is stored by the save_state_machine function.

    :param str path: Path of state machine folder where selected state machine should be stored
    :param bool recent_opened_notification: Flag to insert path of state machine into recent opened state machine paths
    :param bool as_copy: Store state machine as copy flag e.g. without assigning path to state_machine.file_system_path
    :return: True if successfully stored, False if the storing process was canceled or stopped by condition fail
    :rtype bool:
    """

    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    selected_state_machine_model = state_machine_manager_model.get_selected_state_machine_model()
    if selected_state_machine_model is None:
        logger.warning("Can not 'save state machine as' because no state machine is selected.")
        return False

    if path is None:
        if interface.create_folder_func is None:
            logger.error("No function defined for creating a folder")
            return False
        folder_name = selected_state_machine_model.state_machine.root_state.name
        path = interface.create_folder_func("Please choose a root folder and a folder name for the state-machine. "
                                            "The default folder name is the name of the root state.",
                                            format_default_folder_name(folder_name))
        if path is None:
            logger.warning("No valid path specified")
            return False

    previous_path = selected_state_machine_model.state_machine.file_system_path
    if not as_copy:
        marked_dirty = selected_state_machine_model.state_machine.marked_dirty
        recent_opened_notification = recent_opened_notification and (not previous_path == path or marked_dirty)
        selected_state_machine_model.state_machine.file_system_path = path

    result = save_state_machine(delete_old_state_machine=True,
                                recent_opened_notification=recent_opened_notification,
                                as_copy=as_copy, copy_path=path)
    library_manager_model.state_machine_was_stored(selected_state_machine_model, previous_path)
    return result


def save_selected_state_as():
    """Save selected state as separate state machine

    :return True if successfully stored, False if the storing process was canceled or stopped by condition fail
    :rtype bool:
    :raises exceptions.ValueError: If dialog response ids are out of bounds
    """

    state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model
    selection = state_machine_manager_model.get_selected_state_machine_model().selection
    selected_state = selection.get_selected_state()
    state_machine_id = state_machine_manager_model.get_selected_state_machine_model().state_machine.state_machine_id
    if len(selection.states) == 1:
        state_m = copy.copy(selected_state)
        sm_m = StateMachineModel(StateMachine(root_state=state_m.state))
        sm_m.root_state = state_m
        path = interface.create_folder_func("Please choose a root folder and a folder name for the state-machine your "
                                            "state is saved in. The default folder name is the name of state.",
                                            format_default_folder_name(selected_state.state.name))
        if path:
            storage.save_state_machine_to_path(sm_m.state_machine, base_path=path)
            sm_m.store_meta_data()
        else:
            logger.warning("No valid path specified")
            return False

        def open_as_state_machine_saved_state_as_separate_state_machine():
            logger.debug("Open state machine.")
            try:
                open_state_machine(path=path, recent_opened_notification=True)
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
                                               message_type=Gtk.MessageType.QUESTION,
                                               parent=root_window,
                                               width=800, standalone=False)
            response_id = dialog.run()
            if response_id == 1:  # Apply pressed

                if overwrote_old_lib and dialog.list_store[2][0]:  # refresh all open state machine selected
                    logger.debug("Refresh all is triggered.")
                    refresh_all()
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
                                        message_type=Gtk.MessageType.QUESTION,
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


def is_state_machine_stopped_to_proceed(selected_sm_id=None, root_window=None):
    """ Check if state machine is stopped and in case request user by dialog how to proceed

     The function checks if a specific state machine or by default all state machines have stopped or finished
     execution. If a state machine is still running the user is ask by dialog window if those should be stopped or not.

    :param selected_sm_id: Specific state mine to check for
    :param root_window: Root window for dialog window
    :return:
    """
    # check if the/a state machine is still running
    if not state_machine_execution_engine.finished_or_stopped():
        if selected_sm_id is None or selected_sm_id == state_machine_manager.active_state_machine_id:

            message_string = "A state machine is still running. This state machine can only be refreshed" \
                             "when not longer running."
            dialog = RAFCONButtonDialog(message_string, ["Stop execution and refresh",
                                                         "Keep running and do not refresh"],
                                        message_type=Gtk.MessageType.QUESTION,
                                        parent=root_window)
            response_id = dialog.run()
            state_machine_stopped = False
            if response_id == 1:
                state_machine_execution_engine.stop()
                state_machine_stopped = True
            elif response_id == 2:
                logger.debug("State machine will stay running and no refresh will be performed!")
            dialog.destroy()

            return state_machine_stopped
    return True


def refresh_libraries():
    library_manager.refresh_libraries()


def replace_all_libraries_by_template(state_model):
    for s_id, child_state_model in state_model.states.items():
        if isinstance(child_state_model, LibraryStateModel):
            library_name = child_state_model.state.library_name
            library_path = child_state_model.state.library_path
            library_state = LibraryState(library_path, library_name, "0.1", library_name)
            gui_helper_state.substitute_state_as(child_state_model, library_state, True)
        if isinstance(child_state_model, ContainerStateModel):
            replace_all_libraries_by_template(child_state_model)


def save_all_libraries(target_path):
    for library_key, library_root_path in library_manager.library_root_paths.items():
        # lib_target_path = os.path.join(target_path, os.path.split(library_root_path)[1])
        lib_target_path = os.path.join(target_path, library_key)
        copy_file_or_folder(library_root_path, lib_target_path)


def save_library_config(target_path):
    config_path = os.path.join(target_path, "__generated__config")
    create_path(config_path)
    config_file_path = os.path.join(config_path, "core_config_generated.yaml")
    tmp_dict = rafcon.core.config.global_config.as_dict()
    new_config = rafcon.core.config.Config()

    # copy content
    for key, value in tmp_dict.items():
        new_config.set_config_value(key, value)

    # recreate library paths to be relative to config file
    new_library_paths_entry = {}
    for library_key, library_root_path in library_manager.library_root_paths.items():
        new_library_paths_entry[library_key] = os.path.relpath(os.path.join(target_path, library_key), config_path)
    new_config.set_config_value("LIBRARY_PATHS", new_library_paths_entry)

    # save config file to new location
    new_config.config_file_path = config_file_path
    new_config.save_configuration()
    return config_file_path


def generate_linux_launch_files(target_path, config_path, state_machine_path):
    launch_command = "{} -c {} -o {}".format(
        "/volume/software/common/packages/rafcon/latest/source/rafcon/gui/start.py",
        os.path.relpath(config_path, target_path),
        os.path.relpath(state_machine_path, target_path))
    she_bang = "#!/bin/bash\n\n"

    launch_file_with_env = os.path.join(target_path, "launch_rafcon_with_env_generated.sh")
    with open(launch_file_with_env, 'w') as file_pointer:
        file_pointer.write(she_bang)
        for key, value in os.environ.items():
            if key not in ["PWD", "BASH_FUNC_mc%%", "BASH_FUNC_module%%", "RAFCON_LIBRARY_PATH"]:
                file_pointer.write("export {}=\"{}\"\n".format(key, value))
        file_pointer.write("\n")
        file_pointer.write(launch_command)
    make_file_executable(launch_file_with_env)

    launch_file_without_env = os.path.join(target_path, "launch_rafcon_generated.sh")
    with open(launch_file_without_env, 'w') as file_pointer:
        file_pointer.write(she_bang)
        file_pointer.write(launch_command)
    make_file_executable(launch_file_without_env)


def bake_selected_state_machine(path=None):
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    if not selected_sm_id:
        logger.debug("Cannot bake state machine: No state machine selected!")
    else:
        logger.debug("Baking state machine ...")
    # selected_sm = state_machine_manager.state_machines[selected_sm_id]
    selected_sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[selected_sm_id]
    assert isinstance(selected_sm_m, StateMachineModel)
    root_state_m = selected_sm_m.root_state

    # generate path
    selected_state_machine_model = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    folder_name = format_default_folder_name(selected_state_machine_model.state_machine.root_state.name)
    if path is None:
        path = interface.create_folder_func("Please choose a root folder and a folder name for the state-machine. "
                                            "The default folder name is the name of the root state.", folder_name)
    if path is None:
        logger.warning("Baking canceled!")
        return False

    create_path(path)
    save_all_libraries(path)
    config_path = save_library_config(path)
    state_machine_path = os.path.join(path, "__generated__state_machine")
    generate_linux_launch_files(path, config_path, state_machine_path)

    save_state_machine_as(state_machine_path, as_copy=True)
    make_tarfile(path+".tar", path)
    logger.debug("Baking finished!")


def refresh_selected_state_machine():
    """Reloads the selected state machine.
    """

    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    selected_sm = state_machine_manager.state_machines[selected_sm_id]
    state_machines_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')
    states_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('states_editor_ctrl')

    if not is_state_machine_stopped_to_proceed(selected_sm_id, state_machines_editor_ctrl.get_root_window()):
        return

    # check if the a dirty flag is still set
    all_tabs = list(states_editor_ctrl.tabs.values())
    all_tabs.extend(states_editor_ctrl.closed_tabs.values())
    dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                           tab_dict['source_code_view_is_dirty'] is True]
    if selected_sm.marked_dirty or dirty_source_editor:

        message_string = "Are you sure you want to reload the currently selected state machine?\n\n" \
                         "The following elements have been modified and not saved. " \
                         "These changes will get lost:"
        message_string = "%s\n* State machine #%s and name '%s'" % (
            message_string, str(selected_sm_id), selected_sm.root_state.name)
        for ctrl in dirty_source_editor:
            if ctrl.model.state.get_state_machine().state_machine_id == selected_sm_id:
                message_string = "%s\n* Source code of state with name '%s' and path '%s'" % (
                    message_string, ctrl.model.state.name, ctrl.model.state.get_path())
        dialog = RAFCONButtonDialog(message_string, ["Reload anyway", "Cancel"],
                                    message_type=Gtk.MessageType.WARNING, parent=states_editor_ctrl.get_root_window())
        response_id = dialog.run()
        dialog.destroy()
        if response_id == 1:  # Reload anyway
            pass
        else:
            logger.debug("Refresh of selected state machine canceled")
            return

    library_manager.clean_loaded_libraries()
    refresh_libraries()
    states_editor_ctrl.close_pages_for_specific_sm_id(selected_sm_id)
    state_machines_editor_ctrl.refresh_state_machine_by_id(selected_sm_id)


def refresh_all(force=False):
    """Remove/close all libraries and state machines and reloads them freshly from the file system

    :param bool force: Force flag to avoid any checks
    """
    state_machines_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')
    states_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('states_editor_ctrl')

    if force:
        pass  # no checks direct refresh
    else:

        # check if a state machine is still running
        if not is_state_machine_stopped_to_proceed(root_window=states_editor_ctrl.get_root_window()):
            return

        # check if the a dirty flag is still set
        all_tabs = list(states_editor_ctrl.tabs.values())
        all_tabs.extend(states_editor_ctrl.closed_tabs.values())
        dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                               tab_dict['source_code_view_is_dirty'] is True]
        if state_machine_manager.has_dirty_state_machine() or dirty_source_editor:

            message_string = "Are you sure you want to reload the libraries and all state machines?\n\n" \
                             "The following elements have been modified and not saved. " \
                             "These changes will get lost:"
            for sm_id, sm in state_machine_manager.state_machines.items():
                if sm.marked_dirty:
                    message_string = "%s\n* State machine #%s and name '%s'" % (
                        message_string, str(sm_id), sm.root_state.name)
            for ctrl in dirty_source_editor:
                message_string = "%s\n* Source code of state with name '%s' and path '%s'" % (
                    message_string, ctrl.model.state.name, ctrl.model.state.get_path())
            dialog = RAFCONButtonDialog(message_string, ["Reload anyway", "Cancel"],
                                        message_type=Gtk.MessageType.WARNING, parent=states_editor_ctrl.get_root_window())
            response_id = dialog.run()
            dialog.destroy()
            if response_id == 1:  # Reload anyway
                pass
            else:
                logger.debug("Refresh canceled")
                return

    library_manager.clean_loaded_libraries()
    refresh_libraries()
    states_editor_ctrl.close_all_pages()
    state_machines_editor_ctrl.refresh_all_state_machines()


def delete_core_element_of_model(model, raise_exceptions=False, recursive=True, destroy=True, force=False):
    """Deletes respective core element of handed model of its state machine

    If the model is one of state, data flow or transition, it is tried to delete that model together with its
    data from the corresponding state machine.

    :param model: The model of respective core element to delete
    :param bool raise_exceptions: Whether to raise exceptions or only log errors in case of failures
    :param bool destroy: Access the destroy flag of the core remove methods
    :return: True if successful, False else
    """
    if isinstance(model, AbstractStateModel) and model.state.is_root_state:
        logger.warning("Deletion is not allowed. {0} is root state of state machine.".format(model.core_element))
        return False
    state_m = model.parent
    if state_m is None:
        msg = "Model has no parent from which it could be deleted from"
        if raise_exceptions:
            raise ValueError(msg)
        logger.error(msg)
        return False
    if is_selection_inside_of_library_state(selected_elements=[model]):
        logger.warning("Deletion is not allowed. Element {0} is inside of a library.".format(model.core_element))
        return False
    assert isinstance(state_m, StateModel)

    state = state_m.state
    core_element = model.core_element

    try:
        if core_element in state:
            state.remove(core_element, recursive=recursive, destroy=destroy, force=force)
            return True
        return False
    except (AttributeError, ValueError) as e:
        if raise_exceptions:
            raise
        logger.error("The model '{}' for core element '{}' could not be deleted: {}".format(model, core_element, e))
        return False


def delete_core_elements_of_models(models, raise_exceptions=True, recursive=True, destroy=True, force=False):
    """Deletes all respective core elements for the given models

    Calls the :func:`delete_core_element_of_model` for all given models.

    :param models: A single model or a list of models of respective core element to be deleted
    :param bool raise_exceptions: Whether to raise exceptions or log error messages in case of an error
    :param bool destroy:  Access the destroy flag of the core remove methods
    :return: The number of models that were successfully deleted
    """
    # If only one model is given, make a list out of it
    if not hasattr(models, '__iter__'):
        models = [models]
    return sum(delete_core_element_of_model(model, raise_exceptions, recursive=recursive, destroy=destroy, force=force)
               for model in models)


def is_selection_inside_of_library_state(state_machine_m=None, selected_elements=None):
    """ Check if handed or selected elements are inside of library state

    If no state machine model or selected_elements are handed the method is searching for the selected state machine and
    its selected elements. If selected_elements list is handed handed state machine model is ignored.

    :param rafcon.gui.models.state_machine.StateMachineModel state_machine_m: Optional state machine model
    :param list selected_elements: Optional model list that is considered to be selected
    :return: True if elements inside of library state
    """
    if state_machine_m is None:
        state_machine_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    if state_machine_m is None and selected_elements is None:
        return False
    selection_in_lib = []
    selected_elements = state_machine_m.selection.get_all() if selected_elements is None else selected_elements
    for model in selected_elements:
        # check if model is element of child state or the root state (or its scoped variables) of a LibraryState
        state_m = model if isinstance(model.core_element, State) else model.parent
        selection_in_lib.append(state_m.state.get_next_upper_library_root_state() is not None)
        # check if model is part of the shell (io-port or outcome) of a LibraryState
        if not isinstance(model.core_element, State) and isinstance(state_m, LibraryStateModel):
            selection_in_lib.append(True)
    if any(selection_in_lib):
        return True
    return False


def delete_selected_elements(state_machine_m):
    # avoid to delete any element inside of a library
    if is_selection_inside_of_library_state(state_machine_m):
        logger.warning("Deletion of elements inside of a library is not allowed.")
        return

    if len(state_machine_m.selection) > 0:
        delete_core_elements_of_models(state_machine_m.selection.get_all(), recursive=True, destroy=True)
        return True


def paste_into_selected_state(state_machine_m, cursor_position=None):
    """
    :param (float, float) cursor_position: the cursor position relative to the main window.
    """
    selection = state_machine_m.selection
    if len(selection.states) != 1:
        logger.warning("Please select a single container state for pasting the clipboard")
        return

    # Note: in multi-selection case, a loop over all selected items is necessary instead of the 0 index
    target_state_m = selection.get_selected_state()
    item_coordinates = None
    if cursor_position:
        from rafcon.gui.helpers.coordinates import main_window2graphical_editor
        from rafcon.gui.helpers.coordinates import graphical_editor2item
        gc_coordinates = main_window2graphical_editor(cursor_position)
        item_coordinates = graphical_editor2item(target_state_m, gc_coordinates)
    global_clipboard.paste(target_state_m, item_coordinates)


def selected_state_toggle_is_start_state():

    if rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model() is None:
        logger.warning("No state machine has been selected.")
        return False
    selection = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection
    selected_state_m = selection.get_selected_state()
    if len(selection.states) == 1 and not selected_state_m.state.is_root_state:
        if selected_state_m.state.get_next_upper_library_root_state() is not None:
            logger.warning("Toggle is start state is not performed because selected target state is inside of a "
                        "library state.")
            return False
        try:
            if not selected_state_m.is_start:
                selected_state_m.parent.state.start_state_id = selected_state_m.state.state_id
                logger.debug("New start state '{0}'".format(selected_state_m.state.name))
            else:
                selected_state_m.parent.state.start_state_id = None
                logger.debug("Start state unset, no start state defined")
        except ValueError as e:
            logger.warning("Could no change start state: {0}".format(e))
        return True
    else:
        logger.warning("To toggle the is start state flag you have to select exact on state.")
        return False


def add_new_state(state_machine_m, state_type, target_position=None):
    """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

    Adds a new state only if the parent state (selected state) is a container state, and if the graphical editor or
    the state machine tree are in focus.

    :param state_machine_m: the state machine model to add the state to
    :param state_type: the state type of the state to be added
    :param (float, float) target_position: The position, to add the state at, relative to the graphical editor.
    """
    assert isinstance(state_machine_m, StateMachineModel)

    if state_type not in list(StateType):
        state_type = StateType.EXECUTION

    if len(state_machine_m.selection.states) != 1:
        logger.warning("Please select exactly one desired parent state, before adding a new state")
        return
    state_m = state_machine_m.selection.get_selected_state()
    if is_selection_inside_of_library_state(selected_elements=[state_m]):
        logger.warning("Add new state is not performed because selected target state is inside of a library state.")
        return

    if isinstance(state_m, StateModel):
        rel_pos_to_state = None
        if target_position:
            from rafcon.gui.helpers.coordinates import graphical_editor2item
            rel_pos_to_state = graphical_editor2item(state_m, target_position)
        return gui_helper_state.add_state(state_m, state_type, rel_pos_to_state)
    else:
        logger.warning("Add new state is not performed because target state indication has to be a {1} not {0}"
                       "".format(state_m.__class__.__name__, StateModel.__name__))

    # TODO this code can not be reached -> recover again? -> e.g. feature select transition add's state to parent
    if isinstance(state_m, (TransitionModel, DataFlowModel)) or \
            isinstance(state_m, (DataPortModel, OutcomeModel)) and isinstance(state_m.parent, ContainerStateModel):
        return gui_helper_state.add_state(state_m.parent, state_type)


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


def insert_state_into_selected_state(state, as_template=False):
    """Adds a State to the selected state

    :param state: the state which is inserted
    :param as_template: If a state is a library state can be insert as template
    :return: boolean: success of the insertion
    """
    smm_m = rafcon.gui.singleton.state_machine_manager_model

    if not isinstance(state, State):
        logger.warning("A state is needed to be insert not {0}".format(state))
        return False

    if not smm_m.selected_state_machine_id:
        logger.warning("Please select a container state within a state machine first")
        return False

    selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
    if len(selection.states) > 1:
        logger.warning("Please select exactly one state for the insertion")
        return False

    if len(selection.states) == 0:
        logger.warning("Please select a state for the insertion")
        return False

    if is_selection_inside_of_library_state(selected_elements=[selection.get_selected_state()]):
        logger.warning("State is not insert because target state is inside of a library state.")
        return False

    gui_helper_state.insert_state_as(selection.get_selected_state(), state, as_template)

    return True


def add_state_by_drag_and_drop(state, data):
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    ctrl_path = ['state_machines_editor_ctrl', selected_sm_id]
    state_machine_editor_ctrl = rafcon.gui.singleton.main_window_controller.get_controller_by_path(ctrl_path)
    state_machine_editor_ctrl.perform_drag_and_drop = True
    if insert_state_into_selected_state(state, False):
        data.set_text(state.state_id, -1)
    state_machine_editor_ctrl.perform_drag_and_drop = False


def add_data_port_to_selected_states(data_port_type, data_type=None, selected_states=None):
    data_type = 'int' if data_type is None else data_type
    if selected_states is None:
        selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.states
    if is_selection_inside_of_library_state(selected_elements=selected_states):
        logger.warning("The data port couldn't be added because target state is inside of a library state.")
        return
    if all([isinstance(state_m.state, LibraryState) for state_m in selected_states]):
        logger.warning("The data port couldn't be added to state of type {0}"
                       "".format(LibraryState.__name__))
        return
    ids = {}
    for state_m in selected_states:
        # save name with generated data port id
        data_port_id = id_generator.generate_data_port_id(state_m.state.get_data_port_ids())
        if data_port_type == 'INPUT':
            name = 'input_' + str(data_port_id)
            try:
                state_m.state.add_input_data_port(name=name, data_type=data_type, data_port_id=data_port_id)
                ids[state_m.state] = data_port_id
            except ValueError as e:
                logger.warning("The input data port couldn't be added: {0}".format(e))
        elif data_port_type == 'OUTPUT':
            name = 'output_' + str(data_port_id)
            try:
                state_m.state.add_output_data_port(name=name, data_type=data_type, data_port_id=data_port_id)
                ids[state_m.state] = data_port_id
            except ValueError as e:
                logger.warning("The output data port couldn't be added: {0}".format(e))
        else:
            return
    return ids


def add_scoped_variable_to_selected_states(data_type=None, selected_states=None):
    data_type = 'int' if data_type is None else data_type
    if selected_states is None:
        selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.states
    if is_selection_inside_of_library_state(selected_elements=selected_states):
        logger.warning("The scoped variable couldn't be added because target state is inside of a library state.")
        return
    if all([not isinstance(state_m.state, ContainerState) for state_m in selected_states]):
        logger.warning("The scoped variable couldn't be added to state of type {0}"
                    "".format([state_m.state.__class__.__name__
                               for state_m in selected_states if not isinstance(state_m.state, ContainerState)]))
        return
    ids = {}
    for state_m in selected_states:
        if isinstance(state_m.state, ContainerState):
            # save name with generated data port id
            data_port_id = id_generator.generate_data_port_id(state_m.state.get_data_port_ids())
            try:
                state_m.state.add_scoped_variable("scoped_{0}".format(data_port_id), data_type, 0)
                ids[state_m.state] = data_port_id
            except ValueError as e:
                logger.warning("The scoped variable couldn't be added: {0}".format(e))

    return ids


def add_outcome_to_selected_states(selected_states=None):
    if selected_states is None:
        selected_states = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection.states
    if is_selection_inside_of_library_state(selected_elements=selected_states):
        logger.warning("The outcome couldn't be added because target state is inside of a library state.")
        return
    if all([isinstance(state_m.state, LibraryState) for state_m in selected_states]):
        logger.warning("The outcome couldn't be added to state of type {0}"
                       "".format(LibraryState.__name__))
        return
    ids = {}
    for state_m in selected_states:
        # save name with generated outcome id
        outcome_id = id_generator.generate_outcome_id(list(state_m.state.outcomes.keys()))
        name = "success_" + str(outcome_id)
        try:
            state_m.state.add_outcome(name=name, outcome_id=outcome_id)
            ids[state_m.state] = outcome_id
        except ValueError as e:
            logger.warning("The outcome couldn't be added: {0}".format(e))
    return ids


def change_state_type_with_error_handling_and_logger_messages(state_m, target_class):
    if is_selection_inside_of_library_state(selected_elements=[state_m]):
        logger.warning("Change state type is not performed because target state is inside of a library state.")
        return
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
    selection = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection
    selected_state_m = selection.get_selected_state()
    if len(selection.states) == 1 and not is_selection_inside_of_library_state(selected_elements=[selected_state_m]):
        # calculate position for dialog window
        root_window = rafcon.gui.singleton.main_window_controller.get_root_window()
        x, y = root_window.get_position()
        _width, _height = root_window.get_size()
        # print("x, y, width, height, bit_depth", x, y, width, height)
        pos = (x + _width/4, y + _height/6)
        StateSubstituteChooseLibraryDialog(rafcon.gui.singleton.library_manager_model, width=450, height=550, pos=pos,
                                           parent=root_window)
        return True
    else:
        logger.warning("Substitute state needs exact one state to be selected which is not inside of a library state.")
        return False


def substitute_selected_state(state, as_template=False, keep_name=False):
    """ Substitute the selected state with the handed state

    :param rafcon.core.states.state.State state: A state of any functional type that derives from State
    :param bool as_template: The flag determines if a handed the state of type LibraryState is insert as template
    :return:
    """
    # print("substitute_selected_state", state, as_template)
    assert isinstance(state, State)
    from rafcon.core.states.barrier_concurrency_state import DeciderState
    if isinstance(state, DeciderState):
        raise ValueError("State of type DeciderState can not be substituted.")

    smm_m = rafcon.gui.singleton.state_machine_manager_model
    if not smm_m.selected_state_machine_id:
        logger.error("Selected state machine can not be found, please select a state within a state machine first.")
        return False

    selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
    selected_state_m = selection.get_selected_state()
    if len(selection.states) != 1:
        logger.error("Please select exactly one state for the substitution")
        return False
    if is_selection_inside_of_library_state(selected_elements=[selected_state_m]):
        logger.warning("Substitute is not performed because target state is inside of a library state.")
        return

    gui_helper_state.substitute_state_as(selected_state_m, state, as_template, keep_name)

    return True


def substitute_selected_library_state_with_template(keep_name=True):
    selection = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection
    selected_state_m = selection.get_selected_state()
    if len(selection.states) == 1 and isinstance(selected_state_m, LibraryStateModel):
        # print("start substitute library state with template")
        # TODO optimize this to not generate one more library state and model
        lib_state = copy.deepcopy(selected_state_m.state)
        # lib_state_m = copy.deepcopy(selected_states[0].state)
        substitute_selected_state(lib_state, as_template=True, keep_name=keep_name)
        # TODO think about to use as return value the inserted state
        return True
    else:
        logger.warning("Substitute library state with template needs exact one library state to be selected.")
        return False


def group_selected_states_and_scoped_variables():
    logger.debug("try to group")
    if is_selection_inside_of_library_state():
        logger.error("Group is not performed because target elements are inside of a library state.")
        return
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    selection = sm_m.selection
    selected_states = list(selection.states)
    selected_scoped_vars = list(selection.scoped_variables)
    selected_state_m = selection.get_selected_state()
    if len(selected_states) > 0 and isinstance(selected_state_m.parent, StateModel) or len(selected_scoped_vars):
        # check if all elements have the same parent or leave it to the parent
        parent_list = []
        for state_m in selected_states:
            parent_list.append(state_m.state.parent)
        for sv_m in selected_scoped_vars:
            parent_list.append(sv_m.scoped_variable.parent)
        if not len(set(parent_list)) == 1:
            raise AttributeError("All elements that should be grouped have to have one direct parent state.")
        logger.debug("Group selected states: {0} scoped variables: {1}".format(selected_states, selected_scoped_vars))
        # TODO remove un-select workaround (used to avoid wrong selections in gaphas and inconsistent selection)
        sm_m.selection.clear()
        group_state = gui_helper_state.group_states_and_scoped_variables(selected_states, selected_scoped_vars)
        if group_state:
            sm_m.selection.add(sm_m.get_state_model_by_path(group_state.get_path()))
        return group_state


def ungroup_selected_state():
    logger.debug("try to ungroup")
    selection = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().selection
    selected_state_m = selection.get_selected_state()
    if len(selection.states) == 1 and isinstance(selected_state_m, ContainerStateModel) and \
            not selected_state_m.state.is_root_state:
        logger.debug("do ungroup")
        if is_selection_inside_of_library_state(selected_elements=[selected_state_m]):
            logger.warning("Ungroup is not performed because target state is inside of a library state.")
            return
        selection.remove(selected_state_m)
        return gui_helper_state.ungroup_state(selected_state_m)


def get_root_state_file_path(sm_file_system_path):
    if os.path.isdir(sm_file_system_path) and os.path.exists(os.path.join(sm_file_system_path, storage.STATEMACHINE_FILE)):
        try:
            sm_dict = storage.load_data_file(os.path.join(sm_file_system_path, storage.STATEMACHINE_FILE))
        except ValueError:
            return
        if 'root_state_id' not in sm_dict and 'root_state_storage_id' not in sm_dict:
            return
        root_state_folder = sm_dict['root_state_id'] if 'root_state_id' in sm_dict else sm_dict['root_state_storage_id']
        return os.path.join(sm_file_system_path, root_state_folder, storage.FILE_NAME_CORE_DATA)


def get_root_state_name_of_sm_file_system_path(file_system_path):
    root_state_file_path = get_root_state_file_path(sm_file_system_path=file_system_path)
    if root_state_file_path:
        state_dict = storage_utils.load_objects_from_json(root_state_file_path, as_dict=True)
        if 'name' in state_dict:
            return state_dict['name']
        return


def get_root_state_description_of_sm_file_system_path(file_system_path):
    root_state_file_path = get_root_state_file_path(sm_file_system_path=file_system_path)
    if root_state_file_path:
        state_dict = storage_utils.load_objects_from_json(root_state_file_path, as_dict=True)
        if 'description' in state_dict:
            return state_dict['description']
        return
