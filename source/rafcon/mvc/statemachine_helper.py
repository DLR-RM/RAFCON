from rafcon.utils import log
from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID

logger = log.get_logger(__name__)

from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.statemachine.enums import StateType
from rafcon.mvc.models import StateModel, ContainerStateModel, TransitionModel, DataFlowModel
from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel
import rafcon.mvc.singleton


class StateMachineHelper():
    @staticmethod
    def delete_model(model, raise_exceptions=False):
        """Deletes a model of its state machine

        If the model is one of state, data flow or transition, it is tried to delete that model together with its
        data from the corresponding state machine.
        :param model: The model to delete
        :return: True if successful, False else
        """
        container_m = model.parent
        if container_m is None:
            return False
        assert isinstance(container_m, StateModel)
        if isinstance(model, StateModel):
            state_id = model.state.state_id
            try:
                if state_id in container_m.state.states:
                    container_m.state.remove_state(state_id)
                    return True
            except AttributeError as e:
                if not raise_exceptions:
                    logger.error("The state with the ID {0} and the name {1} could not be deleted: {2}".format(
                        state_id, model.state.name, e.message))
                else:
                    raise

        elif isinstance(model, TransitionModel):
            transition_id = model.transition.transition_id
            try:
                if transition_id in container_m.state.transitions:
                    container_m.state.remove_transition(transition_id)
                    return True
            except AttributeError as e:
                if not raise_exceptions:
                    logger.error("The transition with the ID {0} could not be deleted: {1}".format(
                        transition_id, e.message))
                else:
                    raise

        elif isinstance(model, DataFlowModel):
            data_flow_id = model.data_flow.data_flow_id
            try:
                if data_flow_id in container_m.state.data_flows:
                    container_m.state.remove_data_flow(data_flow_id)
                    return True
            except AttributeError as e:
                if not raise_exceptions:
                    logger.error("The data flow with the ID {0} could not be deleted: {1}".format(
                        data_flow_id, e.message))
                else:
                    raise

        elif isinstance(model, ScopedVariableModel):
            scoped_variable_id = model.scoped_variable.data_port_id
            try:
                if scoped_variable_id in container_m.state.scoped_variables:
                    container_m.state.remove_scoped_variable(scoped_variable_id)
                    return True
            except AttributeError as e:
                if not raise_exceptions:
                    logger.error("The scoped variable with the ID {0} could not be deleted: {1}".format(
                        scoped_variable_id, e.message))
                else:
                    raise

        elif isinstance(model, DataPortModel):
            port_id = model.data_port.data_port_id
            try:
                if port_id in container_m.state.input_data_ports:
                    container_m.state.remove_input_data_port(port_id)
                    return True
                elif port_id in container_m.state.output_data_ports:
                    container_m.state.remove_output_data_port(port_id)
                    return True
            except AttributeError as e:
                if not raise_exceptions:
                    logger.error("The data port with the ID {0} could not be deleted: {1}".format(
                        port_id, e.message))
                else:
                    raise

        return False

    @staticmethod
    def delete_models(models, raise_exceptions=False):
        """Deletes all given models from their state machines

        Calls the :func:`StateMachineHelper.delete_model` for all models given.
        :param models: A single model or a list of models to be deleted
        :return: The number of models that were successfully deleted
        """
        num_deleted = 0
        # If only one model is given, make a list out of it
        if not isinstance(models, list):
            models = [models]
        for model in models:
            if StateMachineHelper.delete_model(model, raise_exceptions):
                num_deleted += 1
        return num_deleted

    @staticmethod
    def add_state(container_state_m, state_type):
        """Add a state to a container state

        Adds a state of type state_type to the given container_state
        :param rafcon.mvc.models.container_state.ContainerState container_state: A model of a container state to add the new
        state to
        :param rafcon.statemachine.enums.StateType state_type: The type of state that should be added
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

    @staticmethod
    def duplicate_state_with_other_state_type(source_state, target_state_class):
        """ The function duplicates/transforms a state to a new state type. If the source state type and the new state
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
            logger.info("Type change from %s to %s" % (type(source_state), target_state_class))

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
                                           scoped_variables=source_state.scoped_variables,
                                           v_checker=source_state.v_checker)
        else:  # TRANSFORM from EXECUTION- TO CONTAINER-STATE or FROM CONTAINER- TO EXECUTION-STATE
            new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                           input_data_ports=source_state.input_data_ports,
                                           output_data_ports=source_state.output_data_ports,
                                           outcomes=source_state.outcomes)

        if source_state.description is not None and len(source_state.description) > 0:
            new_state.description = source_state.description

        return new_state

    @staticmethod
    def run_before_model_functionality_of_change_state_type(orig_state_m, new_state_class):
        """ The function  stores model information like meta data of external (in the parent of the state) related transitions
        and data flows as well as StateModel-attributes of the original Models (of the original state) for operations
        on the newly generated models after core-operations. Additionally the function cares about selection issues.
        :param orig_state_m: state model of state which state type (class-type) should be changed to new_state_class
        :param new_state_class:
        :return:
        """
        # check if root state and which type of state
        assert isinstance(orig_state_m, StateModel)
        assert issubclass(new_state_class, State)
        orig_state = orig_state_m.state  # only here to get the input parameter of the Core-function

        is_root_state = orig_state.is_root_state

        current_state_is_container = isinstance(orig_state, ContainerState)
        new_state_is_container = new_state_class in [HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState]

        # remove selection from StateMachineModel.selection -> find state machine model
        state_machine_m = rafcon.mvc.singleton.state_machine_manager_model.get_sm_m_for_state_model(orig_state_m)
        state_machine_m.selection.remove(orig_state_m)

        state_id = orig_state_m.state.state_id

        # store old meta information of related linkage
        orig_model_linkage_meta_data = {'transitions': {}, 'data_flows': {}}
        if not is_root_state:
            parent_m = orig_state_m.parent
            assert isinstance(parent_m.state, ContainerState)

            for transition_m in parent_m.transitions:
                transition = transition_m.transition
                if transition.from_state == state_id or transition.to_state == state_id:
                    orig_model_linkage_meta_data['transitions'][transition.transition_id] = transition_m.meta
            for data_flow_m in parent_m.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    orig_model_linkage_meta_data['data_flows'][data_flow.data_flow_id] = data_flow_m.meta

        # define which model references to hold for new state
        refs_from_model = ['meta', 'input_data_ports', 'output_data_ports']
        if current_state_is_container and new_state_is_container:  # hold some additional references
            refs_from_model.extend(['states', 'transitions', 'data_flows', 'scoped_variables'])

        orig_model_property_refs = {}
        for prop_name in refs_from_model:
            orig_model_property_refs[prop_name] = orig_state_m.__getattribute__(prop_name)

        return state_machine_m, orig_model_property_refs, orig_model_linkage_meta_data

    @staticmethod
    def run_core_functionality_of_change_state_type(orig_state, new_state_class):
        """ The function performs the core functionality of the state type change.
        It differs between general state machine root_state changes and state type changes of n-level of child states.
        The original state get substituted into the duplicated new type of state using the function
        duplicate_state_with_other_state_type.
        If the state is not a root_state external linkage (data flows and transitions) is maintained by storing
        original linkage elements and reconstruction of those after substitution of the state.

        :param orig_state: state which state type (class-type) should be changed to new_state_class
        :param new_state_class: the new type of state-class to which orig_state_m.state will be transformed
        :return:
        """

        assert isinstance(orig_state, State)
        assert issubclass(new_state_class, State)

        is_root_state = orig_state.is_root_state

        if not is_root_state:  # PARENT IS CONTAINER STATE
            # has parent state
            parent_state = orig_state.parent
            assert isinstance(parent_state, ContainerState)

            # remember related external transitions and data flows
            connected_transitions = []
            connected_data_flows = []
            for t_id, transition in orig_state.parent.transitions.iteritems():
                if transition.from_state == orig_state.state_id or transition.to_state == orig_state.state_id:
                    connected_transitions.append(transition)

            for df_id, data_flow in orig_state.parent.data_flows.iteritems():
                if data_flow.from_state == orig_state.state_id or data_flow.to_state == orig_state.state_id:
                    connected_data_flows.append(data_flow)

            # create new state from new type
            new_state = StateMachineHelper.duplicate_state_with_other_state_type(orig_state, new_state_class)

            # substitute old state with new state
            # new_state.parent = parent_state  # TODO remove - unnecessary as long done in add_state
            if isinstance(orig_state, ContainerState) and isinstance(new_state, ExecutionState):
                parent_state.remove_state(orig_state.state_id, recursive_deletion=True, force=True)
            else:
                parent_state.remove_state(orig_state.state_id, recursive_deletion=False, force=True)
            parent_state.add_state(new_state)

            # reconstruct related external transitions and data flows
            # -> skip transitions if the PARENT state ist Barrier- or PreemptiveConcurrencyState
            if not (isinstance(parent_state, BarrierConcurrencyState) or
                    isinstance(parent_state, PreemptiveConcurrencyState)):
                for t in connected_transitions:
                    parent_state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)

            for df in connected_data_flows:
                parent_state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

        else:  # PARENT IS STATE MACHINE
            # create new state from new type
            new_state = StateMachineHelper.duplicate_state_with_other_state_type(orig_state, new_state_class)

            # substitute original root state with new state
            from rafcon.statemachine.singleton import state_machine_manager
            sm_id = orig_state.get_sm_for_state().state_machine_id
            state_machine_manager.state_machines[sm_id].root_state = new_state

    @staticmethod
    def run_after_model_functionality_of_change_state_type(orig_state_m,
                                                           state_machine_m,
                                                           orig_model_property_refs,
                                                           orig_model_linkage_meta_data):
        """Inserts meta of external and related transition and data flow models and reconstructs the linking
        (to the parent model) of elements (data flows, transitions and states and so on) in the original state model in
        the new state model.

        :param orig_state_m: state model of state which state type (class-type) should be changed to new_state_class
        :param state_machine_m: state machine model in which state machine the type change happens
        :param orig_model_property_refs: references on properties (child models) of the original model
        :param orig_model_linkage_meta_data: meta data of external linkage elements in the parent state
        :return:
        """
        # find the parent of original and new state model
        is_root_state = orig_state_m.state.is_root_state

        if not is_root_state:
            parent_m = orig_state_m.parent
            assert isinstance(parent_m.state, ContainerState)
        else:
            parent_m = state_machine_m

        # get new StateModel
        if not is_root_state:
            # CONTAINER STATE MODEL CASE
            new_state_m = parent_m.states[orig_state_m.state.state_id]
            # new_state_m.parent = parent_m  # TODO remove - sollte schon passiert sein oder?
        else:  # STATE MACHINE MODEL CASE
            new_state_m = parent_m.root_state
            rafcon.mvc.singleton.state_machine_manager_model.selected_state_machine_id = state_machine_m.state_machine.state_machine_id

        # handle special case of BarrierConcurrencyState -> secure decider state model to not be overwritten
        if isinstance(new_state_m.state, BarrierConcurrencyState):
            decider_state_m = new_state_m.states[UNIQUE_DECIDER_STATE_ID]

        # by default all transitions are left out if the new and original state are container states
        # -> because Barrier, Preemptive or Hierarchy has always different rules
        if isinstance(orig_state_m.state, ContainerState) and isinstance(new_state_m.state, ContainerState):
            for i in range(len(orig_model_property_refs['transitions'])):
                orig_model_property_refs['transitions'].pop()

        # insert and link original state model attributes (child-models) into/with new state model (the new parent)
        for prop_name, value in orig_model_property_refs.iteritems():
            # look_out: all model properties get overwritten here
            new_state_m.__setattr__(prop_name, value)
            # Set the parent of all child models to the new state model
            if prop_name == "states":
                for state_m in new_state_m.states.itervalues():
                    state_m.parent = new_state_m
            if prop_name in ['input_data_ports', 'output_data_ports', 'transitions', 'data_flows', 'scoped_variables']:
                for model in new_state_m.__getattribute__(prop_name):
                    model.parent = new_state_m

        # handle special case of BarrierConcurrencyState -> re-insert decider state model
        if isinstance(new_state_m.state, BarrierConcurrencyState):
            decider_state_m.parent = new_state_m
            new_state_m.states[decider_state_m.state.state_id] = decider_state_m

        # If there is a parent,
        # -> Insert Meta-Data of original external related transition and data flow models into the new models
        if not is_root_state:
            if not (isinstance(new_state_m.parent.state, PreemptiveConcurrencyState) or
                    isinstance(new_state_m.parent.state, BarrierConcurrencyState)):
                for t_id, t_meta in orig_model_linkage_meta_data['transitions'].iteritems():
                    parent_m.get_transition_m(t_id).meta = t_meta

            for df_id, df_meta in orig_model_linkage_meta_data['data_flows'].iteritems():
                parent_m.get_data_flow_m(df_id).meta = df_meta

        return new_state_m

    @staticmethod
    def change_state_type(orig_state_m, new_state_class):
        """Change the type of the given state

        The function transforms a original state represented by its model orig_state_m into a different StateType
        new_state_class. This is done in three steps:

        # run_before_model_functionality_of_change_state_type -> necessary model-operation/storing before core changes
        # run_core_functionality_of_change_state_type -> generation of core objects, so reduction or extension
        of original child-elements of the state
        # run_after_model_functionality_of_change_state_type -> necessary model-operation/inserts after core changes

        The core functionality is also performed by duplicate_state_with_other_state_type.

        :param orig_state_m: state model of state which state type (class-type) should be changed to new_state_class
        :param new_state_class: the new type of state-class to which orig_state_m.state will be transformed
        :return:
        """

        [state_machine_m, orig_model_property_refs, orig_model_linkage_meta_data] = \
            StateMachineHelper.run_before_model_functionality_of_change_state_type(orig_state_m, new_state_class)

        StateMachineHelper.run_core_functionality_of_change_state_type(orig_state_m.state, new_state_class)

        new_state_m = StateMachineHelper.run_after_model_functionality_of_change_state_type(orig_state_m,
                                                                                        state_machine_m,
                                                                                        orig_model_property_refs,
                                                                                        orig_model_linkage_meta_data)

        # TODO: different types of states have different constraints, e. g. barrier concurrency states can only have
        # one outcome. Shell the restrictions be checked here?

        # TODO: check all references, are there references remaining to the old state? are the references to all
        # models correct?

        return new_state_m

    @staticmethod
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

    @staticmethod
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

    @staticmethod
    def get_state_model_for_state(state):
        """Return the model for a given state

        The function looks up the state machine id for the given state and walks the state tree up until it find the
        model of the given state.
        :param state: The state of which the state model is searched
        :return: The model corresponding to state
        """
        assert isinstance(state, State)
        from rafcon.statemachine.singleton import state_machine_manager
        state_machine_id = state.get_sm_for_state().state_machine_id
        state_machine_m = rafcon.mvc.singleton.state_machine_manager_model.state_machines[state_machine_id]
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

    @staticmethod
    def get_state_machine_model_for_state(state):
        """Return the state machine model containing the given state

        :param state: The state of which the state machine model is searched
        :return: The state machine model containing the state
        """
        assert isinstance(state, State)
        from rafcon.statemachine.singleton import state_machine_manager
        state_machine_id = state.get_sm_for_state().state_machine_id
        state_machine_m = rafcon.mvc.singleton.state_machine_manager_model.state_machines[state_machine_id]
        return state_machine_m
