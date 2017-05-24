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

import gtk
import copy

from rafcon.core import interface, id_generator
from rafcon.core.singleton import state_machine_manager, library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.storage import storage
from rafcon.gui import singleton as gui_singletons
from rafcon.gui.controllers.state_substitute import StateSubstituteChooseLibraryDialog
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.models.library_state import LibraryStateModel
from rafcon.gui.utils.dialog import RAFCONButtonDialog, RAFCONCheckBoxTableDialog
from rafcon.utils import log

logger = log.get_logger(__name__)


def add_data_port_to_selected_states(data_port_type, data_type=None):
    data_type = 'int' if data_type is None else data_type
    for state_m in gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states():
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
    selected_states = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()

    if all([not isinstance(state_m, ContainerState) for state_m in selected_states]):
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
    for state_m in gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states():
        # save name with generated outcome id
        outcome_id = id_generator.generate_outcome_id(state_m.state.outcomes.keys())
        name = "outcome_" + str(outcome_id)
        try:
            state_m.state.add_outcome(name=name, outcome_id=outcome_id)
        except ValueError as e:
            logger.warn("The outcome couldn't be added: {0}".format(e))
    return


def substitute_selected_state():
    selected_states = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1:
        StateSubstituteChooseLibraryDialog(gui_singletons.library_manager_model,
                                           parent=gui_singletons.main_window_controller.get_root_window())
        return True
    else:
        logger.warning("Substitute state needs exact one state to be selected.")
        return False


def substitute_library_with_template():
    selected_states = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1 and isinstance(selected_states[0], LibraryStateModel):
        lib_state = LibraryState.from_dict(LibraryState.state_to_dict(selected_states[0].state))
        gui_helper_state_machine.substitute_state(lib_state, as_template=True)
        # TODO find out why the following generates a problem (e.g. lose of outcomes)
        # gui_helper_state_machine.substitute_state(selected_states[0].state, as_template=True)
        return True
    else:
        logger.warning("Substitute library state with template needs exact one library state to be selected.")
        return False


def save_selected_state_as():
    state_machine_manager_model = gui_singletons.state_machine_manager_model
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
                                               parent=gui_singletons.main_window_controller.view.get_top_widget(),
                                               width=800, standalone=False)
            response_id = dialog.run()
            if response_id == 1:  # Apply pressed

                if overwrote_old_lib and dialog.list_store[2][0]:  # refresh all open state machine selected
                    logger.debug("Refresh all is triggered.")
                    menu_bar_ctrl = gui_singletons.main_window_controller.get_controller('menu_bar_controller')
                    gui_helper_state_machine.refresh_all(menu_bar_ctrl)
                else:  # if not all was refreshed at least the libraries are refreshed
                    logger.debug("Library refresh is triggered.")
                    gui_helper_state_machine.refresh_libraries()

                if dialog.list_store[0][0]:  # Substitute saved state with Library selected
                    logger.debug("Substitute saved state with Library.")
                    if dialog.list_store[0][0] or dialog.list_store[0][1]:
                        gui_helper_state_machine.refresh_libraries()
                    state_machine_manager_model.selected_state_machine_id = state_machine_id
                    [library_path, library_name] = library_manager.get_library_path_and_name_for_os_path(path)
                    state = library_manager.get_library_instance(library_path, library_name)
                    try:
                        gui_helper_state_machine.substitute_state(state, as_template=False)
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
                                        parent=gui_singletons.main_window_controller.get_root_window())
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


def change_state_type(model, target_class):
    if not isinstance(model.state, target_class):
        state_name = model.state.name
        logger.debug("Change type of State '{0}' from {1} to {2}".format(state_name,
                                                                         type(model.state).__name__,
                                                                         target_class.__name__))
        try:
            if model.state.is_root_state:
                model.state.parent.change_root_state_type(target_class)
            else:
                model.state.parent.change_state_type(model.state, target_class)
        except Exception as e:
            logger.error("An error occurred while changing the state type: {0}".format(e))
    else:
        logger.debug("DON'T Change type of State '{0}' from {1} to {2}".format(model.state.name,
                                                                               type(model.state).__name__,
                                                                               target_class.__name__))
