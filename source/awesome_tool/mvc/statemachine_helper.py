from awesome_tool.utils import log
from awesome_tool.statemachine.enums import UNIQUE_DECIDER_STATE_ID

logger = log.get_logger(__name__)

from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.states.library_state import LibraryState
from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from awesome_tool.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from awesome_tool.statemachine.enums import StateType
from awesome_tool.mvc.models import StateModel, ContainerStateModel, TransitionModel, DataFlowModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel
from awesome_tool.mvc.singleton import state_machine_manager_model


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
        :param awesome_tool.mvc.models.container_state.ContainerState container_state: A model of a container state to add the new
        state to
        :param awesome_tool.statemachine.enums.StateType state_type: The type of state that should be added
        :return: True if successful, False else
        """
        if container_state_m is None:
            logger.error("Cannot add a state without a parent.")
            return False
        if not isinstance(container_state_m, StateModel) or \
                (isinstance(container_state_m, StateModel) and not isinstance(container_state_m, ContainerStateModel)):
            logger.error("Parent state must be a container, for example a Hierarchy State.")
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
        current_state_is_container = isinstance(source_state, ContainerState)
        new_state_is_container = target_state_class in [HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState]

        if current_state_is_container and new_state_is_container:
            assert isinstance(source_state, ContainerState)

            # if new_state_class in [BarrierConcurrencyState, PreemptiveConcurrencyState]:
            for t_id in source_state.transitions.keys():
                source_state.remove_transition(t_id)
            state_transitions = {}
            state_start_state_id = None
            new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                           input_data_ports=source_state.input_data_ports,
                                           output_data_ports=source_state.output_data_ports,
                                           outcomes=source_state.outcomes, states=source_state.states,
                                           transitions=state_transitions, data_flows=source_state.data_flows,
                                           start_state_id=state_start_state_id,
                                           scoped_variables=source_state.scoped_variables,
                                           v_checker=source_state.v_checker,
                                           path=source_state.script.path, filename=source_state.script.filename,
                                           check_path=False)
        else:
            if hasattr(source_state, "states"):
                # remove old child states
                for child_state_id in source_state.states.keys():
                    source_state.remove_state(child_state_id, force=True)
            new_state = target_state_class(name=source_state.name, state_id=source_state.state_id,
                                           input_data_ports=source_state.input_data_ports,
                                           output_data_ports=source_state.output_data_ports,
                                           outcomes=source_state.outcomes)  # , path=state.script.path, filename=state.script.filename,
                                        # check_path=False)

        new_state._used_data_port_ids = source_state._used_data_port_ids
        new_state._used_outcome_ids = source_state._used_outcome_ids

        if source_state.description is not None and len(source_state.description) > 0:
            new_state.description = source_state.description

        return new_state

    @staticmethod
    def save_data_of_old_state(old_state_m, new_state_class):
        # BEFORE MODEL
        assert isinstance(old_state_m, StateModel)
        assert issubclass(new_state_class, State)
        old_state = old_state_m.state  # only here to get the input parameter of the Core-function

        is_root_state = old_state_m.parent is None

        current_state_is_container = isinstance(old_state, ContainerState)
        new_state_is_container = new_state_class in [HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState]

        # remove selection from StateMachineModel.selection -> find state machine model
        state_machine_m = state_machine_manager_model.get_sm_m_for_state_model(old_state_m)
        state_machine_m.selection.remove(old_state_m)

        state_id = old_state_m.state.state_id

        # store old meta information of linkage
        old_model_properties_meta_data = {'transitions': {}, 'data_flows': {}}

        if not is_root_state:
            parent_m = old_state_m.parent
            assert isinstance(parent_m.state, ContainerState)

            for transition_m in parent_m.transitions:
                transition = transition_m.transition
                if transition.from_state == state_id or transition.to_state == state_id:
                    old_model_properties_meta_data['transitions'][transition.transition_id] = transition_m.meta
            for data_flow_m in parent_m.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    old_model_properties_meta_data['data_flows'][data_flow.data_flow_id] = data_flow_m.meta

        copy_from_model = ['meta', 'input_data_ports', 'output_data_ports']
        if current_state_is_container and new_state_is_container:
            copy_from_model.extend(['states', 'transitions', 'data_flows', 'scoped_variables'])

        old_model_properties = {}
        for prop_name in copy_from_model:
            old_model_properties[prop_name] = old_state_m.__getattribute__(prop_name)

        return state_machine_m, old_model_properties, old_model_properties_meta_data

    @staticmethod
    def do_core_functionality(state, new_state_class):

        # CORE FUNCTIONALITY
        assert isinstance(state, State)
        assert issubclass(new_state_class, State)

        is_root_state = state.parent is None

        # CONTAINER STATE
        if not is_root_state:
            # has parent state
            parent_state = state.parent
            assert isinstance(parent_state, ContainerState)

            # remember related external transitions and data flows
            connected_transitions = []
            connected_data_flows = []
            for t_id, transition in state.parent.transitions.iteritems():
                if transition.from_state == state.state_id or transition.to_state == state.state_id:
                    connected_transitions.append(transition)

            for df_id, data_flow in state.parent.data_flows.iteritems():
                if data_flow.from_state == state.state_id or data_flow.to_state == state.state_id:
                    connected_data_flows.append(data_flow)

            # create new state from new type
            new_state = StateMachineHelper.duplicate_state_with_other_state_type(state, new_state_class)

            # substitute old state with new state
            new_state.parent = parent_state
            parent_state.remove_state(state.state_id, recursive_deletion=False, force=True)
            parent_state.add_state(new_state)

            # re-implement external transitions and data flows
            if not (isinstance(parent_state, BarrierConcurrencyState) or
                        isinstance(parent_state, PreemptiveConcurrencyState)):
                for t in connected_transitions:
                    parent_state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)

            for df in connected_data_flows:
                parent_state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

        # STATE MACHINE
        else:
            # create new state from new type
            new_state = StateMachineHelper.duplicate_state_with_other_state_type(state, new_state_class)

            # substitute old root state with new state
            from awesome_tool.statemachine.singleton import state_machine_manager
            sm_id = state_machine_manager.get_sm_id_for_state(state)
            state_machine_manager.state_machines[sm_id].root_state = new_state

    @staticmethod
    def load_data_from_old_state_into_new_state(new_state_m, state_machine_m, old_model_properties, old_model_properties_meta_data):

        is_root_state = new_state_m.parent is None

        if not is_root_state:
            parent_m = new_state_m.parent
            assert isinstance(parent_m.state, ContainerState)
        else:
            parent_m = state_machine_m

        def link_models_to_new_model(target_state_model, source_state_model_properties):
            # link old models to new state model (the new parent)
            for prop_name, value in source_state_model_properties.iteritems():
                # look_out: all model properties get overwritten here
                target_state_model.__setattr__(prop_name, value)
                # Set the parent of all child models to the new state model
                if prop_name == "states":
                    for state_m in new_state_m.states.itervalues():
                        state_m.parent = target_state_model
                if prop_name in ['input_data_ports', 'output_data_ports', 'transitions', 'data_flows', 'scoped_variables']:
                    for model in target_state_model.__getattribute__(prop_name):
                        model.parent = target_state_model

        state_id = new_state_m.state.state_id
        # CONTAINER STATE MODEL CASE
        if not is_root_state:
            # get new StateModel
            new_state_m = parent_m.states[state_id]
            new_state_m.parent = parent_m  # sollte schon passiert sein oder?
        else:  # STATE MACHINE MODEL CASE
            new_state_m = parent_m.root_state
            state_machine_manager_model.selected_state_machine_id = state_machine_m.state_machine.state_machine_id

        if isinstance(new_state_m.state, BarrierConcurrencyState):
            decider_state_m = new_state_m.states[UNIQUE_DECIDER_STATE_ID]

        # insert again meta data of external related linkage elements
        link_models_to_new_model(new_state_m, old_model_properties)

        if isinstance(new_state_m.state, BarrierConcurrencyState):
            decider_state_m.parent = new_state_m
            new_state_m.states[decider_state_m.state.state_id] = decider_state_m

        # CONTAINER STATE MODEL CASE
        if not is_root_state:
            # skip meta data for preemptive concurrency states and barrier concurrency states
            if not (isinstance(new_state_m.state, PreemptiveConcurrencyState) or
                    isinstance(new_state_m.state, BarrierConcurrencyState)):
                for t_id, t_meta in old_model_properties_meta_data['transitions'].iteritems():
                    parent_m.get_transition_model(t_id).meta = t_meta

            for df_id, df_meta in old_model_properties_meta_data['data_flows'].iteritems():
                parent_m.get_data_flow_model(df_id).meta = df_meta

        return new_state_m

    @staticmethod
    def change_state_type(old_state_m, new_state_class):

        [state_machine_m, old_model_properties, old_model_properties_meta_data] = \
            StateMachineHelper.save_data_of_old_state(old_state_m, new_state_class)

        StateMachineHelper.do_core_functionality(old_state_m.state, new_state_class)

        new_state_m = StateMachineHelper.load_data_from_old_state_into_new_state(old_state_m,
                                                                                 state_machine_m,
                                                                                 old_model_properties,
                                                                                 old_model_properties_meta_data)

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
        while state_m.parent is not None and (not library_root or not isinstance(state_m.state, LibraryState)):
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
        from awesome_tool.statemachine.singleton import state_machine_manager
        state_machine_id = state_machine_manager.get_sm_id_for_state(state)
        state_machine_m = state_machine_manager_model.state_machines[state_machine_id]
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
        from awesome_tool.statemachine.singleton import state_machine_manager
        state_machine_id = state_machine_manager.get_sm_id_for_state(state)
        state_machine_m = state_machine_manager_model.state_machines[state_machine_id]
        return state_machine_m
