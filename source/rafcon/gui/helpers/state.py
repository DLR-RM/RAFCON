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

from builtins import str
from rafcon.core.states.state import State, StateType
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.state_elements.logical_port import Income
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.gui import singleton as gui_singletons

import rafcon.gui.helpers.meta_data as gui_helper_meta_data
from rafcon.gui.models import ContainerStateModel, AbstractStateModel, StateModel, StateMachineModel, \
    LibraryStateModel, get_state_model_class_for_state
from rafcon.gui.models.signals import ActionSignalMsg, MetaSignalMsg
from rafcon.utils.vividict import Vividict
from rafcon.utils import log

state_type_to_state_class_dict = {StateType.EXECUTION: ExecutionState, StateType.HIERARCHY: HierarchyState,
                                  StateType.BARRIER_CONCURRENCY: BarrierConcurrencyState,
                                  StateType.PREEMPTION_CONCURRENCY: PreemptiveConcurrencyState}

logger = log.get_logger(__name__)


def negative_check_for_model_in_expected_future_models(target_state_m, model, msg, delete=True, with_logger=None):
    """ Checks if the expected future models list/set includes still a specific model

    Return False if the handed model is still in and also creates a warning message as feedback.

    :param StateModel target_state_m: The state model which expected_future_models attribute should be checked
    :param Model model: Model to check for.
    :param str msg: Message for the logger if a model is still in.
    :param bool delete: Flag to delete respective model from list/set.
    :param with_logger: A optional logger to use in case of logging messages
    :rtype: bool
    :return: True if empty and False if still model in set/list
    """
    if with_logger is None:
        with_logger = logger
    # check that the model in the list expected_future_model was used
    if model in target_state_m.expected_future_models:
        with_logger.warning("{0} -> still in is: {1} Please inform the developer how to reproduce this."
                            "".format(msg, model))
        if delete:
            # TODO think about to destroy this models
            target_state_m.expected_future_models.remove(model)
        return False
    return True


def check_expected_future_model_list_is_empty(target_state_m, msg, delete=True, with_logger=None):
    """ Checks if the expected future models list/set is empty

    Return False if there are still elements in and also creates a warning message as feedback.

    :param StateModel target_state_m: The state model which expected_future_models attribute should be checked
    :param str msg: Message for the logger if a model is still in.
    :param bool delete: Flag to delete respective model from list/set.
    :param with_logger: A optional logger to use in case of logging messages
    :rtype: bool
    :return: True if empty and False if still model in set/list
    """
    if with_logger is None:
        with_logger = logger
    # check that the model in the list expected_future_model was used
    if target_state_m.expected_future_models:
        with_logger.warning("{0} -> still in are: {1} Please inform the developer how to reproduce this."
                            "".format(msg, target_state_m.expected_future_models))
        if delete:
            # TODO think about to destroy this models
            target_state_m.expected_future_models.clear()
        return False
    return True


def update_models_recursively(state_m, expected=True):
    """ If a state model is reused the model depth maybe is to low. Therefore this method checks if all 
    library state models are created with reliable depth
    
    :param bool expected: Define newly generated library models as expected or triggers logger warnings if False
    """

    assert isinstance(state_m, AbstractStateModel)

    if isinstance(state_m, LibraryStateModel):
        if not state_m.state_copy_initialized:
            if not expected:
                logger.warning("State {0} generates unexpected missing state copy models.".format(state_m))
            state_m.recursive_generate_models(load_meta_data=False)
            import rafcon.gui.helpers.meta_data as gui_helper_meta_data
            gui_helper_meta_data.scale_library_content(state_m)

    if isinstance(state_m, ContainerStateModel):
        for child_state_m in state_m.states.values():
            update_models_recursively(child_state_m, expected)


def add_state(container_state_m, state_type, add_position=None):
    """Add a state to a container state

    Adds a state of type state_type to the given container_state

    :param rafcon.gui.models.container_state.ContainerState container_state_m: A model of a container state to add
      the new state to
    :param rafcon.core.enums.StateType state_type: The type of state that should be added
    :param (float, float) add_position: The position, to add the state at, relative to the container_state_m in item coordinates.
    :return: True if successful, False else
    """
    if container_state_m is None:
        logger.error("Cannot add a state without a parent.")
        return False

    if not isinstance(container_state_m, StateModel) or \
            (isinstance(container_state_m, StateModel) and not isinstance(container_state_m, ContainerStateModel)):
        logger.error("Parent state must be a container, for example a Hierarchy State." + str(container_state_m))
        return False

    state_class = state_type_to_state_class_dict.get(state_type, None)

    if state_class is None:
        logger.error("Cannot create state of type {0}".format(state_type))
        return False

    new_state = state_class()
    from rafcon.gui.models.abstract_state import get_state_model_class_for_state
    new_state_m = get_state_model_class_for_state(new_state)(new_state)
    gui_helper_meta_data.put_default_meta_on_state_m(new_state_m, container_state_m)
    if add_position is not None:
        new_state_m.set_meta_data_editor('rel_pos', add_position)
    container_state_m.expected_future_models.add(new_state_m)
    container_state_m.state.add_state(new_state)
    return True


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

        # separate state-elements from source state
        data_flows = dict(source_state.data_flows)
        source_state.data_flows = {}
        input_data_ports = dict(source_state.input_data_ports)
        output_data_ports = dict(source_state.output_data_ports)
        scoped_variables = dict(source_state.scoped_variables)
        income = source_state.income
        outcomes = dict(source_state.outcomes)
        source_state.input_data_ports = {}
        source_state.output_data_ports = {}
        source_state.scoped_variables = {}
        source_state.transitions = {}  # before remove of outcomes related transitions should be gone
        source_state.income = Income()
        source_state.outcomes = {}
        states = dict(source_state.states)
        # TODO check why next line can not be performed
        # source_state.states = {}

        new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                       input_data_ports=input_data_ports,
                                       output_data_ports=output_data_ports,
                                       scoped_variables=scoped_variables,
                                       income=income,
                                       outcomes=outcomes,
                                       transitions=state_transitions,
                                       data_flows=data_flows,
                                       states=states,
                                       start_state_id=state_start_state_id)

    else:  # TRANSFORM from EXECUTION- TO CONTAINER-STATE or FROM CONTAINER- TO EXECUTION-STATE

        # in case the new state is an execution state remove of child states (for observable notifications)
        if current_state_is_container and issubclass(target_state_class, ExecutionState):
            if isinstance(source_state, BarrierConcurrencyState):
                source_state.remove_state(UNIQUE_DECIDER_STATE_ID, force=True)
                assert UNIQUE_DECIDER_STATE_ID not in source_state.states
            for state_id in list(source_state.states.keys()):
                source_state.remove_state(state_id)

        # separate state-elements from source state
        input_data_ports = dict(source_state.input_data_ports)
        output_data_ports = dict(source_state.output_data_ports)
        income = source_state.income
        outcomes = dict(source_state.outcomes)
        source_state.input_data_ports = {}
        source_state.output_data_ports = {}
        source_state.income = Income()
        source_state.outcomes = {}

        new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                       input_data_ports=input_data_ports,
                                       output_data_ports=output_data_ports,
                                       income=income, outcomes=outcomes)

    if source_state.description is not None and len(source_state.description) > 0:
        new_state.description = source_state.description
    new_state.semantic_data = Vividict(source_state.semantic_data)
    return new_state


def extract_child_models_of_state(state_m, new_state_class):
    """Retrieve child models of state model

    The function extracts the child state and state element models of the given state model into a dict. It only
    extracts those properties that are required for a state of type `new_state_class`. Transitions are always left out.

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
    required_model_properties = ['input_data_ports', 'output_data_ports', 'outcomes', 'income']
    obsolete_model_properties = []
    if current_state_is_container and new_state_is_container:  # hold some additional references
        # transition are removed when changing the state type, thus do not copy them
        required_model_properties.extend(['states', 'data_flows', 'scoped_variables'])
        obsolete_model_properties.append('transitions')
    elif current_state_is_container:
        obsolete_model_properties.extend(['states', 'transitions', 'data_flows', 'scoped_variables'])

    def get_element_list(state_m, prop_name):
        if prop_name == 'income':
            return [state_m.income]
        wrapper = getattr(state_m, prop_name)
        # ._obj is needed as gaphas wraps observable lists and dicts into a gaphas.support.ObsWrapper
        list_or_dict = wrapper._obj
        if isinstance(list_or_dict, list):
            return list_or_dict[:]  # copy list
        return list(list_or_dict.values())  # dict

    required_child_models = {}
    for prop_name in required_model_properties:
        required_child_models[prop_name] = get_element_list(state_m, prop_name)
    obsolete_child_models = {}
    for prop_name in obsolete_model_properties:
        obsolete_child_models[prop_name] = get_element_list(state_m, prop_name)

    # Special handling of BarrierState, which includes the DeciderState that always becomes obsolete
    if isinstance(state_m, ContainerStateModel):
        decider_state_m = state_m.states.get(UNIQUE_DECIDER_STATE_ID, None)
        if decider_state_m:
            if new_state_is_container:
                required_child_models['states'].remove(decider_state_m)
                obsolete_child_models['states'] = [decider_state_m]

    return required_child_models, obsolete_child_models


def create_state_model_for_state(new_state, meta, state_element_models):
    """Create a new state model with the defined properties

    A state model is created for a state of the type of new_state. All child models in state_element_models (
    model list for port, connections and states) are added to the new model.

    :param StateModel new_state: The new state object with the correct type
    :param Vividict meta: Meta data for the state model
    :param list state_element_models: All state element and child state models of the original state model
    :return: New state model for new_state with all childs of state_element_models
    """
    from rafcon.gui.models.abstract_state import get_state_model_class_for_state
    state_m_class = get_state_model_class_for_state(new_state)
    new_state_m = state_m_class(new_state, meta=meta, load_meta_data=False, expected_future_models=state_element_models)
    error_msg = "New state has not re-used all handed expected future models."
    check_expected_future_model_list_is_empty(new_state_m, msg=error_msg)

    return new_state_m


def change_state_type(state_m, target_class):

    old_state = state_m.state
    old_state_m = state_m
    state_id = old_state.state_id
    is_root_state = old_state.is_root_state
    state_machine_m = gui_singletons.state_machine_manager_model.get_state_machine_model(old_state_m)

    # Before the state type is actually changed, we extract the information from the old state model, to apply it
    # later on to the new state model
    required_child_models, obsolete_child_models = extract_child_models_of_state(old_state_m, target_class)
    old_state_meta = old_state_m.meta
    # By convention, the first element within the affected models list is the root model that has been affected
    affected_models = [old_state_m]
    state_element_models = []
    obsolete_state_element_models = []
    for state_elements in required_child_models.values():
        affected_models.extend(state_elements)
        state_element_models.extend(state_elements)
    for state_elements in obsolete_child_models.values():
        affected_models.extend(state_elements)
        obsolete_state_element_models.extend(state_elements)

    # TODO ??? maybe separate again into state machine function and state function in respective helper module
    if is_root_state:
        assert isinstance(state_machine_m, StateMachineModel)
        assert state_machine_m.root_state is old_state_m
        old_state_m.action_signal.emit(ActionSignalMsg(action='change_root_state_type', origin='model',
                                                       action_parent_m=state_machine_m,
                                                       affected_models=affected_models,
                                                       after=False,
                                                       kwargs={'target_class': target_class}))
        old_state_m.unregister_observer(state_machine_m)
        state_machine_m.suppress_new_root_state_model_one_time = True
    else:
        parent_state_m = old_state_m.parent
        assert isinstance(parent_state_m, ContainerStateModel)

        old_state_m.action_signal.emit(ActionSignalMsg(action='change_state_type', origin='model',
                                                       action_parent_m=parent_state_m,
                                                       affected_models=affected_models,
                                                       after=False,
                                                       kwargs={'state': old_state, 'target_class': target_class}))
    old_state_m.unregister_observer(old_state_m)

    # CORE
    new_state = new_state_m = e = None
    try:
        if is_root_state:
            new_state = state_machine_m.state_machine.change_root_state_type(target_class)
        else:
            new_state = old_state_m.parent.state.change_state_type(old_state, target_class)
    except Exception as e:
        logger.exception("Root state type change failed" if is_root_state else "Container state type change failed")

    # AFTER MODEL
    # After the state has been changed in the core, we create a new model for it with all information extracted
    # from the old state model
    if new_state:
        if old_state.__class__.__name__ in new_state.name:
            new_state.name = old_state.name.replace(old_state.__class__.__name__, new_state.__class__.__name__)

        # Create a new state model based on the new state and apply the extracted child models
        new_state_m = create_state_model_for_state(new_state, old_state_meta, state_element_models)
        # By convention, tha last model within the affected model list, is the newly created model
        affected_models.append(new_state_m)

    if is_root_state:
        action_type = 'change_root_state_type'
        action_parent_m = state_machine_m
        affected_models = [new_state_m, ]

        if new_state_m:
            new_state_m.register_observer(state_machine_m)
            state_machine_m.root_state = new_state_m
    else:
        action_type = 'change_state_type'
        action_parent_m = parent_state_m

        if new_state_m:
            new_state_m.parent = parent_state_m
            # Access states dict without causing a notifications. The dict is wrapped in a ObsMapWrapper object.
            parent_state_m.states[state_id] = new_state_m
            parent_state_m.update_child_is_start()

    # Destroy all states and state elements (core and models) that are no longer required
    old_state.destroy(recursive=False)
    # Temporarily re-register to prevent KeyError: prepare_destruction calls unregister_observer
    old_state_m.register_observer(old_state_m)
    old_state_m.prepare_destruction(recursive=False)
    for state_element_m in obsolete_state_element_models:
        if isinstance(state_element_m, AbstractStateModel):
            if state_element_m.core_element:
                state_element_m.core_element.destroy(recursive=True)
            else:
                logger.verbose("Multiple calls of destroy {0}".format(state_element_m))
        state_element_m.prepare_destruction()

    old_state_m.action_signal.emit(ActionSignalMsg(action=action_type, origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=affected_models,
                                                   after=True, result=e))

    if is_root_state:
        suppressed_notification_parameters = state_machine_m.change_root_state_type.__func__.suppressed_notification_parameters
        state_machine_m.change_root_state_type.__func__.suppressed_notification_parameters = None
        state_machine_m._send_root_state_notification(*suppressed_notification_parameters)
    return new_state_m


def prepare_state_m_for_insert_as(state_m_to_insert, previous_state_size):
    """Prepares and scales the meta data to fit into actual size of the state."""
    # TODO check how much code is duplicated or could be reused for library fit functionality meta data helper
    # TODO DO REFACTORING !!! and move maybe the hole method to meta data and rename it
    if isinstance(state_m_to_insert, AbstractStateModel) and \
            not gui_helper_meta_data.model_has_empty_meta(state_m_to_insert):

        if isinstance(state_m_to_insert, ContainerStateModel):
            # print("TARGET1", state_m_to_insert.state.state_element_attrs)
            models_dict = {'state': state_m_to_insert}

            for state_element_key in state_m_to_insert.state.state_element_attrs:
                if state_element_key == "income":
                    continue
                state_element_list = getattr(state_m_to_insert, state_element_key)
                # Some models are hold in a gtkmvc3.support.wrappers.ObsListWrapper, not a list
                if hasattr(state_element_list, 'keys'):
                    state_element_list = state_element_list.values()
                models_dict[state_element_key] = {elem.core_element.core_element_id: elem for elem in state_element_list}

            resize_factor = gui_helper_meta_data.scale_meta_data_according_state(models_dict, as_template=True)
            gui_helper_meta_data.resize_income_of_state_m(state_m_to_insert, (resize_factor, resize_factor))

        elif isinstance(state_m_to_insert, StateModel):
            # print("TARGET2", state_m_to_insert.state.state_element_attrs)

            if previous_state_size:
                current_size = state_m_to_insert.get_meta_data_editor()['size']
                factor = gui_helper_meta_data.divide_two_vectors(current_size, previous_state_size)
                state_m_to_insert.set_meta_data_editor('size', previous_state_size)
                factor = (min(*factor), min(*factor))
                gui_helper_meta_data.resize_state_meta(state_m_to_insert, factor)
            else:
                logger.debug("For insert as template of {0} no resize of state meta data is performed because "
                             "the meta data has empty fields.".format(state_m_to_insert))

        # library state is not resize because its ports became resized indirectly -> see was resized flag
        elif not isinstance(state_m_to_insert, LibraryStateModel):
            raise TypeError("For insert as template of {0} no resize of state meta data is performed because "
                            "state model type is not ContainerStateModel or StateModel".format(state_m_to_insert))
    else:
        logger.info("For insert as template of {0} no resize of state meta data is performed because the meta data has "
                    "empty fields.".format(state_m_to_insert))


def insert_state_as(target_state_m, state, as_template):
    """ Add a state into a target state

    In case the state to be insert is a LibraryState it can be chosen to be insert as template.

    :param rafcon.gui.models.container_state.ContainerStateModel target_state_m: State model of the target state
    :param rafcon.core.states.State state: State to be insert as template or not
    :param bool as_template: The flag determines if a handed state of type LibraryState is insert as template
    :return:
    """

    if not isinstance(target_state_m, ContainerStateModel) or \
            not isinstance(target_state_m.state, ContainerState):
        logger.error("States can only be inserted in container states")
        return False

    state_m = get_state_model_class_for_state(state)(state)
    if not as_template:
        gui_helper_meta_data.put_default_meta_on_state_m(state_m, target_state_m)

    # If inserted as template, we have to extract the state_copy and respective model
    else:
        assert isinstance(state, LibraryState)
        old_lib_state_m = state_m
        state_m = state_m.state_copy

        previous_state_size = state_m.get_meta_data_editor()['size']
        gui_helper_meta_data.put_default_meta_on_state_m(state_m, target_state_m)
        # TODO check if the not as template case maybe has to be run with the prepare call
        prepare_state_m_for_insert_as(state_m, previous_state_size)

        old_lib_state_m.prepare_destruction(recursive=False)

    # explicit secure that there is no state_id conflict within target state child states
    while state_m.state.state_id in target_state_m.state.states:
        state_m.state.change_state_id()

    target_state_m.expected_future_models.add(state_m)
    target_state_m.state.add_state(state_m.state)

    # secure possible missing models to be generated
    update_models_recursively(state_m, expected=False)


def substitute_state(target_state_m, state_m_to_insert, as_template=False):
    """ Substitutes the target state

    Both, the state to be replaced (the target state) and the state to be inserted (the new state) are passed via
    parameters.
    The new state adapts the size and position of the target state.
    State elements of the new state are resized but kepp their proportion.

    :param rafcon.gui.models.container_state.AbstractStateModel target_state_m: State Model of state to be substituted
    :param rafcon.gui.models.container_state.StateModel state_m_to_insert: State Model of state to be inserted
    :return:
    """
    # print("substitute_state")

    state_to_insert = state_m_to_insert.state
    action_parent_m = target_state_m.parent
    old_state_m = target_state_m
    old_state = old_state_m.state
    state_id = old_state.state_id

    # BEFORE MODEL
    tmp_meta_data = {'transitions': {}, 'data_flows': {}, 'state': None}
    old_state_m = action_parent_m.states[state_id]
    # print("EMIT-BEFORE ON OLD_STATE ", state_id)
    old_state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=[old_state_m, ], after=False,
                                                   kwargs={'state_id': state_id, 'state': state_to_insert}))
    related_transitions, related_data_flows = action_parent_m.state.get_connections_for_state(state_id)
    tmp_meta_data['state'] = old_state_m.meta
    # print("old state meta", old_state_m.meta)
    external_t = related_transitions['external']
    for t in external_t['ingoing'] + external_t['outgoing'] + external_t['self']:
        tmp_meta_data['transitions'][t.transition_id] = action_parent_m.get_transition_m(t.transition_id).meta
    external_df = related_data_flows['external']
    for df in external_df['ingoing'] + external_df['outgoing'] + external_df['self']:
        tmp_meta_data['data_flows'][df.data_flow_id] = action_parent_m.get_data_flow_m(df.data_flow_id).meta
    action_parent_m.substitute_state.__func__.tmp_meta_data_storage = tmp_meta_data
    action_parent_m.substitute_state.__func__.old_state_m = old_state_m

    # put old state size and rel_pos onto new state
    previous_state_size = state_m_to_insert.get_meta_data_editor()['size']
    state_m_to_insert.set_meta_data_editor('size', old_state_m.get_meta_data_editor()['size'])
    state_m_to_insert.set_meta_data_editor('rel_pos', old_state_m.get_meta_data_editor()['rel_pos'])
    # scale the meta data according new size
    prepare_state_m_for_insert_as(state_m_to_insert, previous_state_size)

    # CORE
    new_state = e = None
    # print("state to insert", state_to_insert)
    try:
        # if as_template:  # TODO remove this work around if the models are loaded correctly
        #     # the following enforce the creation of a new model (in needed depth) and transfer of meta data
        #     import rafcon.gui.action
        #     meta_dict = rafcon.gui.action.get_state_element_meta(state_m_to_insert)
        #     new_state = action_parent_m.state.substitute_state(state_id, state_to_insert)
        #     sm_m = action_parent_m.get_state_machine_m()
        #     rafcon.gui.action.insert_state_meta_data(meta_dict, sm_m.get_state_model_by_path(new_state.get_path()))
        # else:
        action_parent_m.expected_future_models.add(state_m_to_insert)
        new_state = action_parent_m.state.substitute_state(state_id, state_to_insert)
        # assert new_state.state_id is state_id
        assert new_state is state_to_insert
    except Exception as e:
        logger.exception("State substitution failed")

    if new_state:
        # AFTER MODEL
        # print("AFTER MODEL", new_state)
        new_state_m = action_parent_m.states[new_state.state_id]
        update_models_recursively(state_m=new_state_m)
        tmp_meta_data = action_parent_m.substitute_state.__func__.tmp_meta_data_storage
        old_state_m = action_parent_m.substitute_state.__func__.old_state_m
        changed_models = []
        new_state_m.meta = tmp_meta_data['state']
        changed_models.append(new_state_m)
        for t_id, t_meta in tmp_meta_data['transitions'].items():
            if action_parent_m.get_transition_m(t_id) is not None:
                action_parent_m.get_transition_m(t_id).meta = t_meta
                changed_models.append(action_parent_m.get_transition_m(t_id))
            elif t_id in action_parent_m.state.substitute_state.__func__.re_create_io_going_t_ids:
                logger.warning("Transition model with id {0} to set meta data could not be found.".format(t_id))
        for df_id, df_meta in tmp_meta_data['data_flows'].items():
            if action_parent_m.get_data_flow_m(df_id) is not None:
                action_parent_m.get_data_flow_m(df_id).meta = df_meta
                changed_models.append(action_parent_m.get_data_flow_m(df_id))
            elif df_id in action_parent_m.state.substitute_state.__func__.re_create_io_going_df_ids:
                logger.warning("Data flow model with id {0} to set meta data could not be found.".format(df_id))

        msg = ActionSignalMsg(action='substitute_state', origin='model', action_parent_m=action_parent_m,
                              affected_models=changed_models, after=True, result=e)
        # print("EMIT-AFTER OLDSTATE", msg)
        old_state_m.action_signal.emit(msg)

    del action_parent_m.substitute_state.__func__.tmp_meta_data_storage
    del action_parent_m.substitute_state.__func__.old_state_m


def substitute_state_as(target_state_m, state, as_template, keep_name=False):
    """ Substitute a target state with a handed state

    The method generates a state model for the state to be inserted and use function substitute_state to finally
    substitute the state.
    In case the state to be inserted is a LibraryState it can be chosen to be inserted as template.
    It can be chosen that the inserted state keeps the name of the target state.

    :param rafcon.gui.models.state.AbstractStateModel target_state_m: State model of the state to be substituted
    :param rafcon.core.states.State state: State to be inserted
    :param bool as_template: The flag determines if a handed state of type LibraryState is insert as template
    :param bool keep_name: The flag to keep the name of the target state
    :return:
    """

    state_m = get_state_model_class_for_state(state)(state)
    # If inserted as template, we have to extract the state_copy and model otherwise keep original name
    if as_template:
        assert isinstance(state_m, LibraryStateModel)
        state_m = state_m.state_copy
        state_m.state.parent = None

    if keep_name:
        state_m.state.name = target_state_m.state.name

    assert target_state_m.parent.states[target_state_m.state.state_id] is target_state_m
    substitute_state(target_state_m, state_m, as_template)


def group_states_and_scoped_variables(state_m_list, sv_m_list):

    state_ids = [state_m.state.state_id for state_m in state_m_list]
    sv_ids = [sv.scoped_variable.data_port_id for sv in sv_m_list]

    action_parent_m = state_m_list[0].parent if state_m_list else sv_m_list[0].parent

    assert isinstance(action_parent_m, ContainerStateModel)

    # BEFORE MODEL
    tmp_models_dict = {'transitions': {}, 'data_flows': {}, 'states': {}, 'scoped_variables': {}, 'state': None,
                       'input_data_ports': {}, 'output_data_ports': {}}
    related_transitions, related_data_flows = \
        action_parent_m.state.get_connections_for_state_and_scoped_variables(state_ids, sv_ids)
    for state_id in state_ids:
        tmp_models_dict['states'][state_id] = action_parent_m.states[state_id]
    for sv_id in sv_ids:
        tmp_models_dict['scoped_variables'][sv_id] = action_parent_m.get_scoped_variable_m(sv_id)
    for t in related_transitions['enclosed']:
        tmp_models_dict['transitions'][t.transition_id] = action_parent_m.get_transition_m(t.transition_id)
    for df in related_data_flows['enclosed']:
        tmp_models_dict['data_flows'][df.data_flow_id] = action_parent_m.get_data_flow_m(df.data_flow_id)

    affected_models = []
    for elements_dict in tmp_models_dict.values():
        if isinstance(elements_dict, dict):
            affected_models.extend(elements_dict.values())
        elif isinstance(elements_dict, AbstractStateModel):
            affected_models.extend(elements_dict)

    # print("EMIT-BEFORE ON ACTION PARENT")
    action_parent_m.action_signal.emit(ActionSignalMsg(action='group_states', origin='model',
                                                       action_parent_m=action_parent_m,
                                                       affected_models=affected_models, after=False,
                                                       kwargs={'state_ids': state_ids, 'scoped_variables': sv_ids}))

    action_parent_m.group_states.__func__.tmp_models_storage = tmp_models_dict
    action_parent_m.group_states.__func__.affected_models = affected_models

    error_msg = "Group action has not started with empty expected future models list."
    check_expected_future_model_list_is_empty(action_parent_m, msg=error_msg)
    for key in ['states', 'scoped_variables', 'transitions', 'data_flows']:
        for model in tmp_models_dict[key].values():
            action_parent_m.expected_future_models.add(model)

    # CORE
    new_state = e = None
    try:
        assert isinstance(action_parent_m.state, ContainerState)
        new_state = action_parent_m.state.group_states(state_ids, sv_ids)
    except Exception as e2:
        e = e2
        logger.exception("State group failed")

    # AFTER MODEL
    if new_state:
        tmp_models_dict = action_parent_m.group_states.__func__.tmp_models_storage
        grouped_state_m = action_parent_m.states[new_state.state_id]
        tmp_models_dict['state'] = grouped_state_m

        # if models are left over check if the model remove methods have eaten your models because destroy flag was True
        error_msg = "Group action has not re-used all models of grouped elements."
        check_expected_future_model_list_is_empty(action_parent_m, msg=error_msg)
        if not gui_helper_meta_data.scale_meta_data_according_states(tmp_models_dict):
            logger.error("Meta data adaptation for group states failed.")
        else:
            # at the moment this is only used to check and generate error logger messages in case
            grouped_state_m.insert_meta_data_from_models_dict(tmp_models_dict, logger.error)

        affected_models = action_parent_m.group_states.__func__.affected_models
        # print("EMIT-AFTER ON ACTION PARENT")
        affected_models.append(grouped_state_m)

    action_parent_m.action_signal.emit(ActionSignalMsg(action='group_states', origin='model',
                                                       action_parent_m=action_parent_m,
                                                       affected_models=affected_models, after=True, result=e))

    del action_parent_m.group_states.__func__.tmp_models_storage
    del action_parent_m.group_states.__func__.affected_models

    return new_state


def ungroup_state(state_m):

    action_parent_m = state_m.parent
    state_id = state_m.state.state_id
    old_state_m = state_m

    # BEFORE MODEL
    tmp_models_dict = {'transitions': {}, 'data_flows': {}, 'states': {}, 'scoped_variables': {}, 'state': None,
                       'input_data_ports': {}, 'output_data_ports': {}}

    related_transitions, related_data_flows = action_parent_m.state.get_connections_for_state(state_id)
    tmp_models_dict['state'] = action_parent_m.states[state_id]
    for s_id, s_m in action_parent_m.states[state_id].states.items():
        tmp_models_dict['states'][s_id] = s_m
    for sv_m in action_parent_m.states[state_id].scoped_variables:
        tmp_models_dict['scoped_variables'][sv_m.scoped_variable.data_port_id] = sv_m
    for t in related_transitions['internal']['enclosed']:
        tmp_models_dict['transitions'][t.transition_id] = action_parent_m.states[state_id].get_transition_m(t.transition_id)
    for df in related_data_flows['internal']['enclosed']:
        tmp_models_dict['data_flows'][df.data_flow_id] = action_parent_m.states[state_id].get_data_flow_m(df.data_flow_id)
    affected_models = [action_parent_m.states[state_id], ]
    # print("EMIT-BEFORE ON OLD_STATE ", state_id)
    old_state_m.action_signal.emit(ActionSignalMsg(action='ungroup_state', origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=affected_models, after=False,
                                                   kwargs={'state_id': state_id}))
    action_parent_m.ungroup_state.__func__.tmp_models_storage = tmp_models_dict
    action_parent_m.ungroup_state.__func__.affected_models = affected_models
    # print("ungroup", id(old_state_m), [id(m) for m in tmp_models_dict['states']])

    error_msg = "Un-Group action has not started with empty expected future models list."
    check_expected_future_model_list_is_empty(action_parent_m, msg=error_msg)
    for key in ['states']:  # , 'scoped_variables', 'transitions', 'data_flows']:
        for m in tmp_models_dict[key].values():
            if not m.state.state_id == UNIQUE_DECIDER_STATE_ID:
                action_parent_m.expected_future_models.add(m)

    # CORE
    e = None
    try:
        state_m.parent.state.ungroup_state(state_m.state.state_id)
    except Exception as e2:
        e = e2
        logger.exception("State ungroup failed")

    error_msg = "Un-Group action has not re-used all models of grouped elements."
    check_expected_future_model_list_is_empty(action_parent_m, msg=error_msg)

    # AFTER MODEL
    if e is None:
        tmp_models_dict = action_parent_m.ungroup_state.__func__.tmp_models_storage
        # TODO re-organize and use partly the expected_models pattern the next lines
        # TODO -> when transitions/data flows only hold references onto respective logical/data ports
        if not gui_helper_meta_data.offset_rel_pos_of_models_meta_data_according_parent_state(tmp_models_dict):
            logger.error("Meta data adaptation for group states failed.")
        else:
            # reduce tmp models by not applied state meta data
            tmp_models_dict.pop('state')

            # correct state element ids with new state element ids to set meta data on right state element
            tmp_models_dict['states'] = \
                {new_state_id: tmp_models_dict['states'][old_state_id]
                 for old_state_id, new_state_id in action_parent_m.state.ungroup_state.__func__.state_id_dict.items()}
            tmp_models_dict['scoped_variables'] = \
                {new_sv_id: tmp_models_dict['scoped_variables'][old_sv_id]
                 for old_sv_id, new_sv_id in action_parent_m.state.ungroup_state.__func__.sv_id_dict.items()}
            tmp_models_dict['transitions'] = \
                {new_t_id: tmp_models_dict['transitions'][old_t_id]
                 for old_t_id, new_t_id in action_parent_m.state.ungroup_state.__func__.enclosed_t_id_dict.items()}
            tmp_models_dict['data_flows'] = \
                {new_df_id: tmp_models_dict['data_flows'][old_df_id]
                 for old_df_id, new_df_id in action_parent_m.state.ungroup_state.__func__.enclosed_df_id_dict.items()}

            action_parent_m.insert_meta_data_from_models_dict(tmp_models_dict, logger.info)

        affected_models = action_parent_m.ungroup_state.__func__.affected_models
        for elemets_dict in tmp_models_dict.values():
            affected_models.extend(elemets_dict.values())

    old_state_m.action_signal.emit(ActionSignalMsg(action='ungroup_state', origin='model',
                                                   action_parent_m=action_parent_m,
                                                   affected_models=affected_models, after=True, result=e))

    old_state_m.prepare_destruction(recursive=True)
    # print("prepare destruction finished")
    del action_parent_m.ungroup_state.__func__.tmp_models_storage
    del action_parent_m.ungroup_state.__func__.affected_models
    # print("## ungroup finished")
    return old_state_m


def toggle_show_content_flag_of_library_state_model(state_m):
    if not isinstance(state_m, LibraryStateModel):
        logger.warning("The show content is only available for LibraryStateModel instances and can not be toggled for"
                       "{0}".format(state_m))
        return

    if state_m.state.get_next_upper_library_root_state() is not None:
        logger.warning("Can not change show content of library state that is not uppermost library state.")
        return

    state_m.meta['gui']['show_content'] = False if state_m.meta['gui']['show_content'] else True
    msg = MetaSignalMsg(origin='state_overview', change='show_content', affects_children=False)
    state_m.meta_signal.emit(msg)
