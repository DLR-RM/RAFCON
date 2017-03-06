# Copyright

import gtk

from rafcon.core.singleton import state_machine_manager
from rafcon.core import interface
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.state import State
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.core.states.state import StateType

from rafcon.gui.models import StateModel, AbstractStateModel, ContainerStateModel, TransitionModel, DataFlowModel
from rafcon.gui.models.data_port import DataPortModel
from rafcon.gui.models.outcome import OutcomeModel
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils.dialog import RAFCONButtonDialog
import rafcon.gui.singleton
from rafcon.utils import log


logger = log.get_logger(__name__)


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

    if path is None:
        if interface.create_folder_func is None:
            logger.error("No function defined for creating a folder")
            return False
        path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
        if path is None:
            return False
    menubar.model.get_selected_state_machine_model().state_machine.file_system_path = path
    save_state_machine(menubar=menubar, widget=widget, save_as=True, delete_old_state_machine=True)


def refresh_libraries():
    library_manager.refresh_libraries()


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


def add_pos(pos1, pos2):
    return pos1[0] + pos2[0], pos1[1] + pos2[1]


def subtract_pos(pos1, pos2):
    return pos1[0] - pos2[0], pos1[1] - pos2[1]


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


def add_state(container_state_m, state_type):
    """Add a state to a container state

    Adds a state of type state_type to the given container_state

    :param rafcon.gui.models.container_state.ContainerState container_state_m: A model of a container state to add
      the new state to
    :param rafcon.core.enums.StateType state_type: The type of state that should be added
    :return: True if successful, False else
    """
    if container_state_m is None:
        logger.error("Cannot add a state without a parent.")
        return False

    if not isinstance(container_state_m, StateModel) or \
            (isinstance(container_state_m, StateModel) and not isinstance(container_state_m, ContainerStateModel)):
        logger.error("Parent state must be a container, for example a Hierarchy State." + str(container_state_m))
        return False

    new_state = None
    if state_type == StateType.HIERARCHY:
        new_state = HierarchyState()
    elif state_type == StateType.EXECUTION:
        new_state = ExecutionState()
    elif state_type == StateType.BARRIER_CONCURRENCY:
        new_state = BarrierConcurrencyState()
    elif state_type == StateType.PREEMPTION_CONCURRENCY:
        new_state = PreemptiveConcurrencyState()

    if new_state is None:
        logger.error("Cannot create state of type {0}".format(state_type))
        return False

    container_state_m.state.add_state(new_state)
    return True


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
        return add_state(model, state_type)
    if isinstance(model, (TransitionModel, DataFlowModel)) or \
            isinstance(model, (DataPortModel, OutcomeModel)) and isinstance(model.parent, ContainerStateModel):
        return add_state(model.parent, state_type)


def create_new_state_from_state_with_type(source_state, target_state_class):
    """The function duplicates/transforms a state to a new state type. If the source state type and the new state
    type both are ContainerStates the new state will have not transitions to force the user to explicitly re-order
    the logical flow according the paradigm of the new state type.

    :param source_state: previous/original state that is to transform into a new state type (target_state_class)
    :param target_state_class: the final state class type
    :return:
    """

    current_state_is_container = isinstance(source_state, ContainerState)
    new_state_is_container = issubclass(target_state_class, ContainerState)

    if current_state_is_container and new_state_is_container:  # TRANSFORM from CONTAINER- TO CONTAINER-STATE

        # by default all transitions are left out if the new and original state are container states
        # -> because switch from Barrier, Preemptive or Hierarchy has always different rules
        state_transitions = {}
        state_start_state_id = None
        logger.info("Type change from %s to %s" % (type(source_state).__name__, target_state_class.__name__))

        # decider state is removed because it is unique for BarrierConcurrencyState
        if isinstance(source_state, BarrierConcurrencyState):
            source_state.remove_state(UNIQUE_DECIDER_STATE_ID, force=True)
            assert UNIQUE_DECIDER_STATE_ID not in source_state.states

        new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                       input_data_ports=source_state.input_data_ports,
                                       output_data_ports=source_state.output_data_ports,
                                       outcomes=source_state.outcomes, states=source_state.states,
                                       transitions=state_transitions, data_flows=source_state.data_flows,
                                       start_state_id=state_start_state_id,
                                       scoped_variables=source_state.scoped_variables)

    else:  # TRANSFORM from EXECUTION- TO CONTAINER-STATE or FROM CONTAINER- TO EXECUTION-STATE

        # in case the new state is an execution state remove of child states (for observable notifications)
        if current_state_is_container and issubclass(target_state_class, ExecutionState):
            for state_id in source_state.states.keys():
                source_state.remove_state(state_id=state_id)

        new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                       input_data_ports=source_state.input_data_ports,
                                       output_data_ports=source_state.output_data_ports,
                                       outcomes=source_state.outcomes)

    if source_state.description is not None and len(source_state.description) > 0:
        new_state.description = source_state.description

    return new_state


def extract_child_models_of_of_state(state_m, new_state_class):
    """Retrieve child models of state model

    The function stores model information like meta data of external (in the parent of the state) related
    transitions
    and data flows as well as StateModel-attributes of the original Models (of the original state) for operations
    on the newly generated models after core-operations. Additionally the function cares about selection issues.

    :param state_m: state model of which children are to be extracted from
    :param new_state_class: The type of the new class
    :return:
    """
    # check if root state and which type of state
    assert isinstance(state_m, StateModel)
    assert issubclass(new_state_class, State)
    orig_state = state_m.state  # only here to get the input parameter of the Core-function

    current_state_is_container = isinstance(orig_state, ContainerState)
    new_state_is_container = issubclass(new_state_class, ContainerState)

    # define which model references to hold for new state
    model_properties = ['meta', 'input_data_ports', 'output_data_ports', 'outcomes']
    if current_state_is_container and new_state_is_container:  # hold some additional references
        # transition are removed when changing the state type, thus do not copy them
        model_properties.extend(['states', 'data_flows', 'scoped_variables'])

    child_models = {}
    for prop_name in model_properties:
        child_models[prop_name] = state_m.__getattribute__(prop_name)

    return child_models


def create_state_model_for_state(new_state, state_element_models):
    """Create a new state model with the defined properties

    A state model is created for a state of the type of new_state. All child models in state_element_models (
    model list for port, connections and states) are added to the new model.

    :param new_state: The new state object with the correct type
    :param state_element_models: All state element and child state models of the original state model
    :return: New state model for new_state with all childs of state_element_models
    """
    from rafcon.gui.models.abstract_state import get_state_model_class_for_state
    state_m_class = get_state_model_class_for_state(new_state)
    new_state_m = state_m_class(new_state)

    # handle special case of BarrierConcurrencyState -> secure decider state model to not be overwritten
    if isinstance(new_state, BarrierConcurrencyState):
        decider_state_m = new_state_m.states[UNIQUE_DECIDER_STATE_ID]

    # by default all transitions are left out if the new and original state are container states
    # -> because Barrier, Preemptive or Hierarchy has always different rules
    if isinstance(state_element_models, ContainerStateModel):
        state_element_models['transitions'] = []

    # insert and link original state model attributes (child-models) into/with new state model (the new parent)
    for prop_name, value in state_element_models.iteritems():
        if prop_name == "states":
            # First, all automatically generated child states must be removed
            child_state_ids = [state_id for state_id in new_state_m.states]
            for child_state_id in child_state_ids:
                if child_state_id != UNIQUE_DECIDER_STATE_ID:
                    new_state_m.states[child_state_id].prepare_destruction()
                    del new_state_m.states[child_state_id]

            # Then, the old state models can be assigned
            new_state_m.__setattr__(prop_name, value)
            for state_m in new_state_m.states.itervalues():
                state_m.parent = new_state_m

            # Delete decider state model, if existing
            if UNIQUE_DECIDER_STATE_ID in new_state_m.states:
                del new_state_m.states[UNIQUE_DECIDER_STATE_ID]

        elif prop_name in ['outcomes', 'input_data_ports', 'output_data_ports', 'data_flows', 'scoped_variables']:
            # First, all automatically generated child elements must be removed
            for model in new_state_m.__getattribute__(prop_name):
                model.prepare_destruction()
            del new_state_m.__getattribute__(prop_name)[:]

            # Then, the old state element models can be assigned
            new_state_m.__setattr__(prop_name, value)
            for model in new_state_m.__getattribute__(prop_name):
                model.parent = new_state_m
        else:
            # Only the old meta data is left to be assigned
            new_state_m.__setattr__(prop_name, value)

    # handle special case of BarrierConcurrencyState -> re-insert decider state model
    if isinstance(new_state, BarrierConcurrencyState):
        decider_state_m.parent = new_state_m
        new_state_m.states[UNIQUE_DECIDER_STATE_ID] = decider_state_m
    if isinstance(new_state, ContainerState):
        new_state_m.check_is_start_state()

    return new_state_m


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


def get_root_state_model(state_m, library_root=False):
    """Get the root state for a given state model

    The method walks up the state tree from the given state model to find the root state model (which doesn't
    have a parent state). If the flag library_root is set to True, the root is defined as root of the library and
    not of the whole state machine.

    :param state_m: The state model to start from
    :param library_root: Flag to specify if the root of teh library is searched
    :return: The model of the root state (either of the state machine or the library)
    """
    while not state_m.state.is_root_state and (not library_root or not isinstance(state_m.state, LibraryState)):
        state_m = state_m.parent
    return state_m


def get_state_model_for_state(state):
    """Return the model for a given state

    The function looks up the state machine id for the given state and walks the state tree up until it find the
    model of the given state.

    :param state: The state of which the state model is searched
    :return: The model corresponding to state
    """
    assert isinstance(state, State)
    state_machine_id = state.get_state_machine().state_machine_id
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[state_machine_id]
    state_m = state_machine_m.root_state
    state_path = state.get_path()
    path_item_list = state_path.split('/')
    root_state_id = path_item_list.pop(0)
    assert state_m.state.state_id == root_state_id
    while len(path_item_list) > 0:
        state_id = path_item_list.pop(0)
        if isinstance(state_m.state, LibraryState):
            return state_m  # There are no models for states within library states, yes
        state_m = state_m.states[state_id]
    assert state == state_m.state  # Final check
    return state_m


def get_state_machine_model_for_state(state):
    """Return the state machine model containing the given state

    :param state: The state of which the state machine model is searched
    :return: The state machine model containing the state
    """
    assert isinstance(state, State)
    state_machine_id = state.get_state_machine().state_machine_id
    state_machine_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[state_machine_id]
    return state_machine_m


def substitute_state(state, as_template=False):

    assert isinstance(state, State)

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
        parent_state.substitute_state(current_state.state_id, state)
        state.name = current_state_name
        return True
    # If inserted as template, we have to extract the state_copy and load the meta data manually
    else:
        template = state.state_copy
        orig_state_id = template.state_id
        template.change_state_id()
        template.name = current_state_name
        parent_state.substitute_state(current_state.state_id, template)

        # load meta data TODO fix the following code and related code/functions to the 'template' True flag
        # from os.path import join
        # lib_os_path, _, _ = library_manager.get_os_path_to_library(state.library_path, state.library_name)
        # root_state_path = join(lib_os_path, orig_state_id)
        # template_m = parent_state_m.states[template.state_id]
        # template_m.load_meta_data(root_state_path)
        # # parent_state_m.meta_signal.emit(MetaSignalMsg("substitute_state", "all", True))
        # # Causes the template to be resized
        # template_m.temp['gui']['editor']['template'] = True
        # from rafcon.gui.models.signals import Notification
        # notification = Notification(parent_state_m, "states", {'method_name': 'substitute_state'})
        # parent_state_m.meta_signal.emit(MetaSignalMsg("substitute_state", "all", True, notification))
        return True


def insert_state(state, as_template=False):
    """Adds a State to the selected state

    :param state: the state which is inserted
    :param as_template:
    :return: boolean: success of the insertion
    """
    smm_m = rafcon.gui.singleton.state_machine_manager_model

    if state is None:
        logger.error("Please select a library state")
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
        current_state.add_state(state)
        return True
    # If inserted as template, we have to extract the state_copy and load the meta data manually
    else:
        template = state.state_copy
        orig_state_id = template.state_id
        template.change_state_id()
        current_state.add_state(template)

        # reset the parent of all ports (logical + data ports)
        # as in the setter function the parent is reset it can be used here
        template.input_data_ports = template.input_data_ports
        template.output_data_ports = template.output_data_ports
        template.outcomes = template.outcomes

        # load meta data
        from os.path import join
        lib_os_path, _, _ = library_manager.get_os_path_to_library(state.library_path, state.library_name)
        root_state_path = join(lib_os_path, orig_state_id)
        template_m = current_state_m.states[template.state_id]
        template_m.load_meta_data(root_state_path)
        # Causes the template to be resized
        template_m.temp['gui']['editor']['template'] = True
        return True


def insert_self_transition_meta_data(state_m, t_id, origin='graphical_editor', combined_action=False):

    try:
        gaphas_editor = global_gui_config.get_config_value('GAPHAS_EDITOR', True)
        y_axis_mirror = 1 if gaphas_editor else -1
        state_meta = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)

        if 'rel_pos' not in state_meta or 'size' not in state_meta:
            return

        transition_m = state_m.parent.get_transition_m(t_id)
        margin = min(state_meta['size']) / 10.
        first_point_x = state_meta['rel_pos'][0] + state_meta['size'][0] + margin
        first_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin
        second_point_x = state_meta['rel_pos'][0] - margin
        second_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin

        waypoints = [(first_point_x, first_point_y), (second_point_x, second_point_y)]
        transition_m.set_meta_data_editor('waypoints', waypoints, from_gaphas=gaphas_editor)

        if combined_action:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='append_to_last_change'))
        else:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='viapoint_position'))
    except TypeError:
        # meta data generation currently only supported for OpenGL editor
        pass


def scale_meta_data_according_state(meta_data):
    """ The full meta data of state elements is scaled (reduced) according the area used indicated by the state
    meta data.

    :param meta_data: dict that hold lists of meta data with state attribute consistent keys
    :return:
    """
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    old_parent_rel_pos = meta_data['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']

    for state_m in meta_data['states'].itervalues():
        old_rel_pos = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        state_m.set_meta_data_editor('rel_pos', add_pos(old_rel_pos, old_parent_rel_pos), from_gaphas=gaphas_editor)

    for scoped_variable_m in meta_data['scoped_variables'].itervalues():
        if not gaphas_editor:
            old_rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            scoped_variable_m.set_meta_data_editor('inner_rel_pos', add_pos(old_rel_pos, old_parent_rel_pos), False)

    connection_models = meta_data['transitions'].values() + meta_data['data_flows'].values()
    for connection_m in connection_models:
        old_waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        new_waypoints = []
        for waypoint in old_waypoints:
            new_waypoints.append(add_pos(waypoint, old_parent_rel_pos))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)

    return True


def scale_meta_data_according_states(meta_data):
    """ The full meta data of state elements is scaled (enlarged) according the area used indicated by the states
    meta data.

    :param meta_data: dict that hold lists of meta data with state attribute consistent keys
    :return:
    """
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1 if gaphas_editor else -1

    state_m = meta_data['state']
    parent_state_m = meta_data['state'].parent
    parent_size = parent_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']

    # Determine outer coordinates of elements that are to be grouped
    # Use borders of the parent state as initial coordinates
    left = parent_size[0]
    right = 0.0
    top = parent_size[1]
    bottom = 0.0
    margin = min(parent_size) / 20.

    # Update outer coordinates regarding all states
    for child_state_m in meta_data['states'].itervalues():
        rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        size = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
        left = min(rel_pos[0], left)
        right = max(rel_pos[0] + size[0], right)
        top = min(y_axis_mirror * rel_pos[1], top)
        bottom = max(y_axis_mirror * rel_pos[1] + size[1], bottom)

    # Update outer coordinates regarding all scoped variables
    if not gaphas_editor:
        for scoped_variable_m in meta_data['scoped_variables'].itervalues():
            rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            left = min(rel_pos[0], left)
            right = max(rel_pos[0], right)
            top = min(y_axis_mirror * rel_pos[1], top)
            bottom = max(y_axis_mirror * rel_pos[1], bottom)

    # Update outer coordinates regarding all connections (waypoints)
    connection_models = meta_data['transitions'].values() + meta_data['data_flows'].values()
    for connection_m in connection_models:
        waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        for waypoint in waypoints:
            left = min(waypoint[0], left)
            right = max(waypoint[0], right)
            top = min(y_axis_mirror * waypoint[1], top)
            bottom = max(y_axis_mirror * waypoint[1], bottom)

    # Add margin and ensure that the upper left corner is within the state
    rel_pos = max(left - margin, 0), y_axis_mirror * max(top - margin, 0)
    # Add margin and ensure that the lower right corner is within the state
    size = (min(right - left + 2 * margin, parent_size[0] - rel_pos[0]),
            min(bottom - top + 2 * margin, parent_size[1] - y_axis_mirror * rel_pos[1]))

    # Set size and position of new container state
    state_m.set_meta_data_editor('rel_pos', rel_pos, from_gaphas=gaphas_editor)
    state_m.set_meta_data_editor('size', size, from_gaphas=gaphas_editor)

    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in meta_data['states'].itervalues():
        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        new_rel_pos = subtract_pos(old_rel_pos, rel_pos)
        child_state_m.set_meta_data_editor('rel_pos', new_rel_pos, from_gaphas=gaphas_editor)

    # Do the same for scoped variable
    if not gaphas_editor:
        for scoped_variable_m in meta_data['scoped_variables'].itervalues():
            old_rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            new_rel_pos = subtract_pos(old_rel_pos, rel_pos)
            scoped_variable_m.set_meta_data_editor('inner_rel_pos', new_rel_pos, from_gaphas=gaphas_editor)

    # Do the same for all connections (transitions and data flows)
    for connection_m in connection_models:
        old_waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        new_waypoints = []
        for waypoint in old_waypoints:
            new_waypoints.append(subtract_pos(waypoint, rel_pos))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)

    return True
