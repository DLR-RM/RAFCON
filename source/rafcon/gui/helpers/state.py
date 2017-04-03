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

from rafcon.core import interface
from rafcon.core.singleton import state_machine_manager, library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.library_state import LibraryState
from rafcon.core.storage import storage
from rafcon.gui import singleton as gui_singletons
from rafcon.gui.controllers.state_substitute import StateSubstituteChooseLibraryDialog
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.models.library_state import LibraryStateModel
from rafcon.gui.models.container_state import ContainerStateModel
from rafcon.gui.models.signals import MetaSignalMsg, StateTypeChangeSignalMsg, ActionSignalMsg
from rafcon.gui.utils.dialog import RAFCONButtonDialog
from rafcon.utils import log

logger = log.get_logger(__name__)


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
    if selected_states and len(selected_states) == 1:
        state_m = copy.copy(selected_states[0])
        sm_m = StateMachineModel(StateMachine(root_state=state_m.state), state_machine_manager_model)
        sm_m.root_state = state_m
        path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
        if path:
            storage.save_state_machine_to_path(sm_m.state_machine, base_path=path, save_as=True)
            sm_m.store_meta_data()
        else:
            return False
        # check if state machine is in library path
        if library_manager.is_os_path_in_library_paths(path):
            # TODO use a check box dialog with three check boxes and an confirmation and cancel button

            # Library refresh dialog
            def on_message_dialog_response_signal(widget, response_id):

                if response_id == 1:
                    logger.debug("Library refresh is triggered.")
                    gui_helper_state_machine.refresh_libraries()
                elif response_id == 2:
                    logger.debug("Refresh all is triggered.")
                    gui_helper_state_machine.refresh_all(None)
                elif response_id in [3, -4]:
                    pass
                else:
                    logger.warning("Response id: {} is not considered".format(response_id))
                    return
                widget.destroy()

            message_string = "You stored your state machine in a path that is included into the library paths.\n\n"\
                             "Do you want to refresh the libraries or refresh libraries and state machines?"
            RAFCONButtonDialog(message_string, ["Refresh libraries", "Refresh everything", "Do nothing"],
                               on_message_dialog_response_signal,
                               message_type=gtk.MESSAGE_QUESTION,
                               parent=gui_singletons.main_window_controller.get_root_window())

            # Offer state substitution dialog
            def on_message_dialog_response_signal(widget, response_id):

                if response_id == 1:
                    logger.debug("Substitute saved state with Library.")
                    gui_helper_state_machine.refresh_libraries()
                    [library_path, library_name] = library_manager.get_library_path_and_name_for_os_path(path)
                    state = library_manager.get_library_instance(library_path, library_name)
                    gui_helper_state_machine.substitute_state(state, as_template=False)
                elif response_id in [2, -4]:
                    pass
                else:
                    logger.warning("Response id: {} is not considered".format(response_id))
                    return
                widget.destroy()

            message_string = "You stored your state machine in a path that is included into the library paths.\n\n"\
                             "Do you want to substitute the state you saved by this library?"
            RAFCONButtonDialog(message_string, ["Substitute", "Do nothing"],
                               on_message_dialog_response_signal,
                               message_type=gtk.MESSAGE_QUESTION,
                               parent=gui_singletons.main_window_controller.get_root_window())

        # Offer to open saved state machine dialog
        def on_message_dialog_response_signal(widget, response_id):

            if response_id == 1:
                logger.debug("Open state machine.")
                try:
                    state_machine = storage.load_state_machine_from_path(path)
                    state_machine_manager.add_state_machine(state_machine)
                except (ValueError, IOError) as e:
                    logger.error('Error while trying to open state machine: {0}'.format(e))
            elif response_id in [2, -4]:
                pass
            else:
                logger.warning("Response id: {} is not considered".format(response_id))
                return
            widget.destroy()

        message_string = "Should the newly created state machine be opened?"
        RAFCONButtonDialog(message_string, ["Open", "Do not open"], on_message_dialog_response_signal,
                           message_type=gtk.MESSAGE_QUESTION,
                           parent=gui_singletons.main_window_controller.get_root_window())
        return True
    else:
        logger.warning("Multiple states can not be saved as state machine directly. Group them before.")
        return False


def change_state_type(model, target_class):
    if target_class != type(model.state):
        state_name = model.state.name
        logger.debug("Change type of State '{0}' from {1} to {2}".format(state_name,
                                                                         type(model.state).__name__,
                                                                         target_class.__name__))
        try:
            model_change_state_type(model, target_class)
        except Exception as e:
            logger.error("An error occurred while changing the state type: {0}".format(e))
            raise
    else:
        logger.debug("DON'T Change type of State '{0}' from {1} to {2}".format(model.state.name,
                                                                               type(model.state).__name__,
                                                                               target_class.__name__))


def model_change_state_type(model, target_class):

    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    import rafcon.gui.singleton as gui_singletons
    from rafcon.utils.vividict import Vividict

    old_state = model.state
    old_state_m = model
    state_id = old_state.state_id
    is_root_state = old_state.is_root_state

    if is_root_state:

        # state_m = model
        state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(old_state_m)
        assert state_machine_m.root_state is old_state_m

        # Before the root state type is actually changed, we extract the information from the old state model and remove
        # the model from the selection
        old_state_m.unregister_observer(old_state_m)

        # print "emit change_root_state_type before msg: "
        old_state_m.action_signal.emit(ActionSignalMsg(action='change_root_state_type', origin='model',
                                                       action_root_m=state_machine_m,
                                                       affected_models=[old_state_m, ],
                                                       after=False,
                                                       args=[target_class]))
        old_state_m.unregister_observer(state_machine_m)
        logger.info("UNREGISTER OBSERVER")
        state_machine_m.selection.remove(old_state_m)

        # Extract child models of state, as they have to be applied to the new state model
        child_models = gui_helper_state_machine.extract_child_models_of_of_state(old_state_m, target_class)
        state_machine_m.change_root_state_type.__func__.child_models = child_models  # static variable of class method
        state_machine_m.suppress_new_root_state_model_one_time = True
        logger.info("FINISH BEFORE")
    else:

        action_root_m = old_state_m.parent
        assert isinstance(action_root_m, ContainerStateModel)

        # BEFORE
        state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(old_state_m)

        # Before the state type is actually changed, we extract the information from the old state model and remove
        # the model from the selection

        def list_dict_to_list(list_or_dict):
            if isinstance(list_or_dict, dict) and not isinstance(list_or_dict, Vividict):
                return list_or_dict.values()
            elif isinstance(list_or_dict, list):
                return list_or_dict
            else:
                return []

        # Extract child models of state, as they have to be applied to the new state model
        child_models = gui_helper_state_machine.extract_child_models_of_of_state(old_state_m, target_class)
        affected_models = [old_state_m, ]
        for list_or_dict in child_models.itervalues():
            affected_models.extend(list_dict_to_list(list_or_dict))
        old_state_m.action_signal.emit(ActionSignalMsg(action='change_state_type', origin='model',
                                                       action_root_m=action_root_m,
                                                       affected_models=affected_models,
                                                       after=False,
                                                       args=[model.state, target_class, ]))
        old_state_m.unregister_observer(old_state_m)
        # remove selection from StateMachineModel.selection -> find state machine model
        state_machine_m.selection.remove(old_state_m)

        action_root_m.change_state_type.__func__.child_models = child_models  # static variable of class method
        action_root_m.change_state_type.__func__.affected_models = affected_models

    # CORE
    new_state = e = None
    try:
        if is_root_state:
            new_state = state_machine_m.state_machine.change_root_state_type(target_class)
        else:
            new_state = old_state_m.parent.state.change_state_type(old_state, target_class)
    except Exception as e:
        raise

    # AFTER MODEL
    # After the state has been changed in the core, we create a new model for it with all information extracted
    # from the old state model
    if is_root_state:

        if new_state is None:
            logger.exception("Root state type change failed {0}".format(e))
        else:
            logger.info("start after TO STATE TYPE CHANGE")
            # Create a new state model based on the new state and apply the extracted child models
            child_models = state_machine_m.change_root_state_type.__func__.child_models
            new_state_m = gui_helper_state_machine.create_state_model_for_state(new_state, child_models)

            new_state_m.register_observer(state_machine_m)
            # state_machine_m.register_observer(state_machine_m)
            logger.info("ASSIGN after TO STATE TYPE CHANGE")
            state_machine_m.root_state = new_state_m
            logger.info("ASSIGNED after TO STATE TYPE CHANGE")
            old_state_m.state_type_changed_signal.emit(StateTypeChangeSignalMsg(new_state_m))

            # print "emit change_root_state_type after msg: "
            old_state_m.action_signal.emit(ActionSignalMsg(action='change_root_state_type', origin='model',
                                                           action_root_m=state_machine_m,
                                                           affected_models=[new_state_m, ],
                                                           after=True))

            state_machine_m.selection.add(new_state_m)
    else:
        # state_m = action_root_state_m.states[state_id]
        if new_state is None:
            logger.exception("Container state type change failed -> {0}".format(e))
        else:
            # Create a new state model based on the new state and apply the extracted child models
            child_models = action_root_m.change_state_type.__func__.child_models
            new_state_m = gui_helper_state_machine.create_state_model_for_state(new_state, child_models)
            # Set this state model (action_root_state_m) to be the parent of our new state model
            new_state_m.parent = action_root_m
            # Access states dict without causing a notifications. The dict is wrapped in a ObsMapWrapper object.
            action_root_m.states[state_id] = new_state_m
            action_root_m.check_is_start_state()

            affected_models = action_root_m.change_state_type.__func__.affected_models
            affected_models.append(new_state_m)
            old_state_m.state_type_changed_signal.emit(StateTypeChangeSignalMsg(new_state_m))
            old_state_m.action_signal.emit(ActionSignalMsg(action='change_state_type', origin='model',
                                                           action_root_m=action_root_m,
                                                           affected_models=affected_models,
                                                           after=True))

            state_machine_m.selection.add(new_state_m)
            # action_root_state_m.meta_signal.emit(MetaSignalMsg("state_type_change", "all", True))

        # del action_root_m.change_state_type.__func__.child_models
        del action_root_m.change_state_type.__func__.affected_models

    if is_root_state:
        state_machine_m._send_root_state_notification(state_machine_m.change_root_state_type.__func__.last_notification_model,
                                                      state_machine_m.change_root_state_type.__func__.last_notification_prop_name,
                                                      state_machine_m.change_root_state_type.__func__.last_notification_info)
