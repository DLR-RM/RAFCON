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
from rafcon.core.states.state import State
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.storage import storage
from rafcon.gui import singleton as gui_singletons
from rafcon.gui.controllers.state_substitute import StateSubstituteChooseLibraryDialog
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
import rafcon.gui.helpers.meta_data as gui_helper_meta_data
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.models.library_state import LibraryStateModel
from rafcon.gui.models.container_state import ContainerStateModel, AbstractStateModel, StateModel, ScopedVariableModel
from rafcon.gui.models.signals import MetaSignalMsg, StateTypeChangeSignalMsg, ActionSignalMsg, Notification
from rafcon.gui.utils.dialog import RAFCONButtonDialog
from rafcon.utils.vividict import Vividict
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
    for state_m in gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states():
        if not isinstance(state_m, ContainerStateModel):
            continue
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
                    substitute_selected_state(state, as_template=False)
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


def change_state_type(state_m, target_class):

    old_state = state_m.state
    old_state_m = state_m
    state_id = old_state.state_id
    is_root_state = old_state.is_root_state

    # TODO ??? maybe separate again into state machine function and state function in respective helper module
    if is_root_state:

        state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(old_state_m)
        assert state_machine_m.root_state is old_state_m



        # print "\n\nEMIT-BEFORE OLDSTATE\n\n"
        old_state_m.action_signal.emit(ActionSignalMsg(action='change_root_state_type', origin='model',
                                                       action_parent_m=state_machine_m,
                                                       affected_models=[old_state_m, ],
                                                       after=False,
                                                       kwargs={'target_class': target_class}))
        old_state_m.unregister_observer(old_state_m)
        old_state_m.unregister_observer(state_machine_m)
        # logger.info("UNREGISTER OBSERVER")

        # Before the root state type is actually changed, we extract the information from the old state model
        # Extract child models of state, as they have to be applied to the new state model
        child_models = gui_helper_state_machine.extract_child_models_of_of_state(old_state_m, target_class)
        state_machine_m.change_root_state_type.__func__.child_models = child_models  # static variable of class method
        state_machine_m.suppress_new_root_state_model_one_time = True
    else:

        action_parent_m = old_state_m.parent
        assert isinstance(action_parent_m, ContainerStateModel)
        state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(old_state_m)

        def list_dict_to_list(list_or_dict):
            if isinstance(list_or_dict, dict) and not isinstance(list_or_dict, Vividict):
                return list_or_dict.values()
            elif isinstance(list_or_dict, list):
                return list_or_dict
            else:
                return []

        # Before the state type is actually changed, we extract the information from the old state model
        # Extract child models of state, as they have to be applied to the new state model
        child_models = gui_helper_state_machine.extract_child_models_of_of_state(old_state_m, target_class)
        affected_models = [old_state_m, ]
        for list_or_dict in child_models.itervalues():
            affected_models.extend(list_dict_to_list(list_or_dict))
        # print "\n\nEMIT-BEFORE OLDSTATE\n\n"
        old_state_m.action_signal.emit(ActionSignalMsg(action='change_state_type', origin='model',
                                                       action_parent_m=action_parent_m,
                                                       affected_models=affected_models,
                                                       after=False,
                                                       kwargs={'state': old_state, 'target_class': target_class}))
        old_state_m.unregister_observer(old_state_m)
        # remove selection from StateMachineModel.selection -> find state machine model

        action_parent_m.change_state_type.__func__.child_models = child_models  # static variable of class method
        action_parent_m.change_state_type.__func__.affected_models = affected_models

    # CORE
    new_state = new_state_m = None
    try:
        if is_root_state:
            new_state = state_machine_m.state_machine.change_root_state_type(target_class)
        else:
            new_state = old_state_m.parent.state.change_state_type(old_state, target_class)
    except Exception:
        logger.exception("Root state type change failed" if is_root_state else "Container state type change failed")

    # AFTER MODEL
    # After the state has been changed in the core, we create a new model for it with all information extracted
    # from the old state model
    if is_root_state:
        if new_state:
            # logger.info("start after TO STATE TYPE CHANGE")
            # Create a new state model based on the new state and apply the extracted child models
            child_models = state_machine_m.change_root_state_type.__func__.child_models
            new_state_m = gui_helper_state_machine.create_state_model_for_state(new_state, child_models)

            new_state_m.register_observer(state_machine_m)
            # state_machine_m.register_observer(state_machine_m)
            state_machine_m.root_state = new_state_m
            # logger.info("ASSIGNED after TO STATE TYPE CHANGE")

            old_state_m.state_type_changed_signal.emit(StateTypeChangeSignalMsg(new_state_m))
            # print "\n\nEMIT-AFTER OLDSTATE\n\n"
            old_state_m.action_signal.emit(ActionSignalMsg(action='change_root_state_type', origin='model',
                                                           action_parent_m=state_machine_m,
                                                           affected_models=[new_state_m, ],
                                                           after=True))

        del state_machine_m.change_root_state_type.__func__.child_models

    else:
        if new_state:
            # Create a new state model based on the new state and apply the extracted child models
            child_models = action_parent_m.change_state_type.__func__.child_models
            new_state_m = gui_helper_state_machine.create_state_model_for_state(new_state, child_models)
            # Set this state model (action_root_state_m) to be the parent of our new state model
            new_state_m.parent = action_parent_m
            # Access states dict without causing a notifications. The dict is wrapped in a ObsMapWrapper object.
            action_parent_m.states[state_id] = new_state_m
            action_parent_m.check_is_start_state()

            affected_models = action_parent_m.change_state_type.__func__.affected_models
            affected_models.append(new_state_m)

            old_state_m.state_type_changed_signal.emit(StateTypeChangeSignalMsg(new_state_m))
            old_state_m.action_signal.emit(ActionSignalMsg(action='change_state_type', origin='model',
                                                           action_parent_m=action_parent_m,
                                                           affected_models=affected_models,
                                                           after=True))

        del action_parent_m.change_state_type.__func__.child_models
        del action_parent_m.change_state_type.__func__.affected_models

    if is_root_state:
        state_machine_m._send_root_state_notification(state_machine_m.change_root_state_type.__func__.last_notification_model,
                                                      state_machine_m.change_root_state_type.__func__.last_notification_prop_name,
                                                      state_machine_m.change_root_state_type.__func__.last_notification_info)
    return new_state_m


def change_state_type_with_error_handling_and_logger_messages(state_m, target_class):
    if not isinstance(state_m.state, target_class):
        logger.debug("Change type of State '{0}' from {1} to {2}".format(state_m.state.name,
                                                                         type(state_m.state).__name__,
                                                                         target_class.__name__))
        try:
            state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(state_m)
            state_machine_m.selection.remove(state_m)
            new_state_m = change_state_type(state_m, target_class)
            state_machine_m.selection.set([new_state_m, ])
        except Exception as e:
            logger.exception("An error occurred while changing the state type")
    else:
        logger.info("State type of State '{0}' will not change because target_class: {1} == state_class: {2}"
                    "".format(state_m.state.name, type(state_m.state).__name__, target_class.__name__))


def substitute_state(target_state_m, state_m_to_insert):

    state_to_insert = state_m_to_insert.state
    action_parent_m = target_state_m.parent
    old_state_m = target_state_m
    old_state = old_state_m.state
    state_id = old_state.state_id

    # BEFORE MODEL
    tmp_meta_data = {'transitions': {}, 'data_flows': {}, 'state': None}
    old_state_m = action_parent_m.states[state_id]
    # print "EMIT-BEFORE ON OLD_STATE ", state_id
    old_state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=[old_state_m, ], after=False,
                                                   kwargs={'state_id': state_id, 'state': state_to_insert}))
    related_transitions, related_data_flows = action_parent_m.state.related_linkage_state(state_id)
    tmp_meta_data['state'] = old_state_m.meta
    for t in related_transitions['external']['ingoing'] + related_transitions['external']['outgoing']:
        tmp_meta_data['transitions'][t.transition_id] = action_parent_m.get_transition_m(t.transition_id).meta
    for df in related_data_flows['external']['ingoing'] + related_data_flows['external']['outgoing']:
        tmp_meta_data['data_flows'][df.data_flow_id] = action_parent_m.get_data_flow_m(df.data_flow_id).meta
    action_parent_m.substitute_state.__func__.tmp_meta_data_storage = tmp_meta_data
    action_parent_m.substitute_state.__func__.old_state_m = old_state_m

    # CORE
    new_state = None
    try:
        new_state = action_parent_m.state.substitute_state(state_id, state_to_insert)
        # assert new_state.state_id is state_id
        assert new_state is state_to_insert
    except Exception:
        logger.exception("State substitution failed")

    if new_state:
        # AFTER MODEL
        new_state_m = action_parent_m.states[new_state.state_id]
        tmp_meta_data = action_parent_m.substitute_state.__func__.tmp_meta_data_storage
        old_state_m = action_parent_m.substitute_state.__func__.old_state_m
        changed_models = []
        new_state_m.meta = tmp_meta_data['state']
        changed_models.append(new_state_m)
        for t_id, t_meta in tmp_meta_data['transitions'].iteritems():
            if action_parent_m.get_transition_m(t_id) is not None:
                action_parent_m.get_transition_m(t_id).meta = t_meta
                changed_models.append(action_parent_m.get_transition_m(t_id))
            elif t_id in action_parent_m.state.substitute_state.__func__.re_create_io_going_t_ids:
                logger.warning("Transition model with id {0} to set meta data could not be found.".format(t_id))
        for df_id, df_meta in tmp_meta_data['data_flows'].iteritems():
            if action_parent_m.get_data_flow_m(df_id) is not None:
                action_parent_m.get_data_flow_m(df_id).meta = df_meta
                changed_models.append(action_parent_m.get_data_flow_m(df_id))
            elif df_id in action_parent_m.state.substitute_state.__func__.re_create_io_going_df_ids:
                logger.warning("Data flow model with id {0} to set meta data could not be found.".format(df_id))

        # TODO re-organize and use partly the expected_models pattern the next lines
        if isinstance(state_m_to_insert, ContainerStateModel):
            models_dict = {'state': new_state_m}
            for key in global_clipboard.Clipboard._container_state_unlimited:
                elems_list = getattr(state_m_to_insert, key)
                elems_list = elems_list.values() if hasattr(elems_list, 'keys') else elems_list
                models_dict[key] = {elem.core_element.core_element_id: elem for elem in elems_list}
            gui_helper_meta_data.scale_meta_data_according_state(models_dict)
            for key in global_clipboard._container_state_unlimited:
                elems_list = getattr(new_state_m, key)
                elems_list = elems_list.values() if hasattr(elems_list, 'keys') else elems_list
                for elem in elems_list:
                    elem.meta = models_dict[key][elem.core_element.core_element_id].meta

        notification = Notification(action_parent_m, "states", {'method_name': 'substitute_state'})
        action_parent_m.meta_signal.emit(MetaSignalMsg("substitute_state", "all", True, notification))
        msg = ActionSignalMsg(action='substitute_state', origin='model', action_parent_m=action_parent_m,
                              affected_models=changed_models, after=True)
        # print "EMIT-AFTER OLDSTATE", msg
        old_state_m.action_signal.emit(msg)

    del action_parent_m.substitute_state.__func__.tmp_meta_data_storage
    del action_parent_m.substitute_state.__func__.old_state_m


def substitute_selected_state(state, as_template=False):

    assert isinstance(state, State)
    from rafcon.core.states.barrier_concurrency_state import DeciderState
    if isinstance(state, DeciderState):
        raise ValueError("State of type DeciderState can not be substituted.")

    smm_m = gui_singletons.state_machine_manager_model

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
        if isinstance(state, ContainerState):
            state_m = ContainerStateModel(state)
        else:
            state_m = StateModel(state)
        substitute_state(parent_state_m.states[current_state.state_id], state_m)
        state.name = current_state_name
        return True
    # If inserted as template, we have to extract the state_copy and load the meta data manually
    else:
        assert isinstance(state, LibraryState)
        template = state.state_copy
        orig_state_id = template.state_id
        template.change_state_id()
        template.name = current_state_name
        if isinstance(state.state_copy, ContainerState):
            template_m = ContainerStateModel(state.state_copy)
            # load meta data TODO fix the following code and related code/functions to the 'template' True flag
            import os.path
            lib_os_path, _, _ = library_manager.get_os_path_to_library(state.library_path, state.library_name)
            root_state_path = os.path.join(lib_os_path, orig_state_id)

            def load_models_recursive(state_m, path):
                state_m.load_meta_data(path)
                if isinstance(state_m, ContainerStateModel):
                    for child_state_id, child_state_m in state_m.states.iteritems():
                        load_models_recursive(child_state_m, os.path.join(path, child_state_id))
            load_models_recursive(template_m, root_state_path)
        else:
            template_m = StateModel(state)

        substitute_state(parent_state_m.states[current_state.state_id], template_m)

        return True


def substitute_selected_state_and_use_choice_dialog():
    selected_states = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1:
        StateSubstituteChooseLibraryDialog(gui_singletons.library_manager_model,
                                           parent=gui_singletons.main_window_controller.get_root_window())
        return True
    else:
        logger.warning("Substitute state needs exact one state to be selected.")
        return False


def substitute_selected_library_state_with_template():
    selected_states = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if selected_states and len(selected_states) == 1 and isinstance(selected_states[0], LibraryStateModel):
        lib_state = LibraryState.from_dict(LibraryState.state_to_dict(selected_states[0].state))
        substitute_selected_state(lib_state, as_template=True)
        return True
    else:
        logger.warning("Substitute library state with template needs exact one library state to be selected.")
        return False


def group_states_and_scoped_variables(state_m_list, sv_m_list):

    state_ids = [state_m.state.state_id for state_m in state_m_list]
    sv_ids = [sv.scoped_variable.data_port_id for sv in sv_m_list]

    action_parent_m = state_m_list[0].parent if state_m_list else sv_m_list[0].parent

    assert isinstance(action_parent_m, ContainerStateModel)

    # BEFORE MODEL
    tmp_models_dict = {'transitions': {}, 'data_flows': {}, 'states': {}, 'scoped_variables': {}, 'state': None}
    related_transitions, related_data_flows = \
        action_parent_m.state.related_linkage_states_and_scoped_variables(state_ids, sv_ids)
    for state_id in state_ids:
        tmp_models_dict['states'][state_id] = action_parent_m.states[state_id]
    for sv_id in sv_ids:
        tmp_models_dict['scoped_variables'][sv_id] = action_parent_m.get_scoped_variable_m(sv_id)
    for t in related_transitions['enclosed']:
        tmp_models_dict['transitions'][t.transition_id] = action_parent_m.get_transition_m(t.transition_id)
    for df in related_data_flows['enclosed']:
        tmp_models_dict['data_flows'][df.data_flow_id] = action_parent_m.get_data_flow_m(df.data_flow_id)

    affected_models = []
    for elemets_dict in tmp_models_dict.itervalues():
        if isinstance(elemets_dict, dict):
            affected_models.extend(elemets_dict.itervalues())
        elif isinstance(elemets_dict, AbstractStateModel):
            affected_models.extend(elemets_dict)

    # print "EMIT-BEFORE ON ACTION PARENT"
    action_parent_m.action_signal.emit(ActionSignalMsg(action='group_states', origin='model',
                                                       action_parent_m=action_parent_m,
                                                       affected_models=affected_models, after=False,
                                                       kwargs={'state_ids': state_ids, 'scoped_variables': sv_ids}))

    action_parent_m.group_states.__func__.tmp_models_storage = tmp_models_dict
    action_parent_m.group_states.__func__.affected_models = affected_models

    # CORE
    new_state = None
    try:
        assert isinstance(action_parent_m.state, ContainerState)
        new_state_id = action_parent_m.state.group_states(state_ids, sv_ids)
        new_state = action_parent_m.state.states[new_state_id]
    except Exception:
        logger.exception("State ungroup failed")

    # AFTER MODEL
    if new_state:
        tmp_models_dict = action_parent_m.group_states.__func__.tmp_models_storage
        grouped_state_m = action_parent_m.states[new_state.state_id]
        tmp_models_dict['state'] = grouped_state_m
        # TODO re-organize and use partly the expected_models pattern the next lines
        if not gui_helper_meta_data.scale_meta_data_according_states(tmp_models_dict):
            del action_parent_m.group_states.__func__.tmp_models_storage
            return

        grouped_state_m.insert_meta_data_from_models_dict(tmp_models_dict)

        affected_models = action_parent_m.group_states.__func__.affected_models
        affected_models.append(grouped_state_m)
        # print "EMIT-AFTER ON ACTION PARENT"
        action_parent_m.action_signal.emit(ActionSignalMsg(action='group_states', origin='model',
                                                           action_parent_m=action_parent_m,
                                                           affected_models=affected_models, after=True))

    del action_parent_m.group_states.__func__.tmp_models_storage
    del action_parent_m.group_states.__func__.affected_models


def group_selected_states_and_scoped_variables():
    logger.debug("try to group")
    sm_m = gui_singletons.state_machine_manager_model.get_selected_state_machine_model()
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
        logger.debug("do group")

        group_states_and_scoped_variables(selected_state_m_list, selected_sv_m)


def ungroup_state(state_m):

    action_parent_m = state_m.parent
    state_id = state_m.state.state_id
    old_state_m = state_m

    # BEFORE MODEL
    tmp_models_dict = {'transitions': {}, 'data_flows': {}, 'states': {}, 'scoped_variables': {}, 'state': None}

    related_transitions, related_data_flows = action_parent_m.state.related_linkage_state(state_id)
    tmp_models_dict['state'] = action_parent_m.states[state_id]
    for s_id, s_m in action_parent_m.states[state_id].states.iteritems():
        tmp_models_dict['states'][s_id] = s_m
    for sv_m in action_parent_m.states[state_id].scoped_variables:
        tmp_models_dict['scoped_variables'][sv_m.scoped_variable.data_port_id] = sv_m
    for t in related_transitions['internal']['enclosed']:
        tmp_models_dict['transitions'][t.transition_id] = action_parent_m.states[state_id].get_transition_m(t.transition_id)
    for df in related_data_flows['internal']['enclosed']:
        tmp_models_dict['data_flows'][df.data_flow_id] = action_parent_m.states[state_id].get_data_flow_m(df.data_flow_id)
    affected_models = [action_parent_m.states[state_id], ]
    # print "EMIT-BEFORE ON OLD_STATE ", state_id
    old_state_m.action_signal.emit(ActionSignalMsg(action='ungroup_state', origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=affected_models, after=False,
                                                   kwargs={'state_id': state_id}))
    action_parent_m.ungroup_state.__func__.tmp_models_storage = tmp_models_dict
    action_parent_m.group_states.__func__.affected_models = affected_models

    # CORE
    e = None
    try:
        state_m.parent.state.ungroup_state(state_m.state.state_id)
    except Exception as e:
        logger.exception("State ungroup failed")

    # AFTER MODEL
    if e is None:
        tmp_models_dict = action_parent_m.ungroup_state.__func__.tmp_models_storage
        # TODO re-organize and use partly the expected_models pattern the next lines
        if not gui_helper_meta_data.offset_rel_pos_of_models_meta_data_according_parent_state(tmp_models_dict):
            del action_parent_m.ungroup_state.__func__.tmp_models_storage
            return

        # reduce tmp models by not applied state meta data
        tmp_models_dict.pop('state')

        # correct state element ids with new state element ids to set meta data on right state element
        tmp_models_dict['states'] = \
            {new_state_id: tmp_models_dict['states'][old_state_id]
             for old_state_id, new_state_id in action_parent_m.state.ungroup_state.__func__.state_id_dict.iteritems()}
        tmp_models_dict['scoped_variables'] = \
            {new_sv_id: tmp_models_dict['scoped_variables'][old_sv_id]
             for old_sv_id, new_sv_id in action_parent_m.state.ungroup_state.__func__.sv_id_dict.iteritems()}
        tmp_models_dict['transitions'] = \
            {new_t_id: tmp_models_dict['transitions'][old_t_id]
             for old_t_id, new_t_id in action_parent_m.state.ungroup_state.__func__.enclosed_t_id_dict.iteritems()}
        tmp_models_dict['data_flows'] = \
            {new_df_id: tmp_models_dict['data_flows'][old_df_id]
             for old_df_id, new_df_id in action_parent_m.state.ungroup_state.__func__.enclosed_df_id_dict.iteritems()}

        action_parent_m.insert_meta_data_from_models_dict(tmp_models_dict)

        affected_models = action_parent_m.group_states.__func__.affected_models
        for elemets_dict in tmp_models_dict.itervalues():
            affected_models.extend(elemets_dict.itervalues())
        # print "EMIT-AFTER ON OLD_STATE ", state_id
        old_state_m.action_signal.emit(ActionSignalMsg(action='ungroup_state', origin='model',
                                                       action_parent_m=action_parent_m,
                                                       affected_models=affected_models, after=True))

    del action_parent_m.ungroup_state.__func__.tmp_models_storage
    del action_parent_m.group_states.__func__.affected_models


def ungroup_selected_state():
    logger.debug("try to ungroup")
    state_m_list = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
    if len(state_m_list) == 1 and isinstance(state_m_list[0], ContainerStateModel) and \
            not state_m_list[0].state.is_root_state:
        logger.debug("do ungroup")
        ungroup_state(state_m_list[0])
