from awesome_tool.utils import log

logger = log.get_logger(__name__)

from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.states.container_state import ContainerState
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
    def delete_model(model):
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
                logger.error("The state with the ID {0} and the name {1} could not be deleted: {2}".format(
                    state_id, model.state.name, e.message))

        elif isinstance(model, TransitionModel):
            transition_id = model.transition.transition_id
            try:
                if transition_id in container_m.state.transitions:
                    container_m.state.remove_transition(transition_id)
                    return True
            except AttributeError as e:
                logger.error("The transition with the ID {0} could not be deleted: {1}".format(
                    transition_id, e.message))

        elif isinstance(model, DataFlowModel):
            data_flow_id = model.data_flow.data_flow_id
            try:
                if data_flow_id in container_m.state.data_flows:
                    container_m.state.remove_data_flow(data_flow_id)
                    return True
            except AttributeError as e:
                logger.error("The data flow with the ID {0} could not be deleted: {1}".format(
                    data_flow_id, e.message))

        elif isinstance(model, ScopedVariableModel):
            scoped_variable_id = model.scoped_variable.data_port_id
            try:
                if scoped_variable_id in container_m.state.scoped_variables:
                    container_m.state.remove_scoped_variable(scoped_variable_id)
                    return True
            except AttributeError as e:
                logger.error("The scoped variable with the ID {0} could not be deleted: {1}".format(
                    scoped_variable_id, e.message))

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
                logger.error("The data port with the ID {0} could not be deleted: {1}".format(
                    port_id, e.message))

        return False

    @staticmethod
    def delete_models(models):
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
            if StateMachineHelper.delete_model(model):
                num_deleted += 1
        return num_deleted

    @staticmethod
    def add_state(container_state, state_type):
        """Add a state to a container state

        Adds a state of type state_type to the given container_state
        :param awesome_tool.mvc.models.container_state.ContainerState container_state: A model of a container state to add the new
        state to
        :param awesome_tool.statemachine.enums.StateType state_type: The type of state that should be added
        :return: True if successful, False else
        """
        if container_state is None:
            logger.error("Cannot add a state without a parent.")
            return False
        if not isinstance(container_state, StateModel) or \
                (isinstance(container_state, StateModel) and not isinstance(container_state, ContainerStateModel)):
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

        container_state.state.add_state(new_state)
        return True

    @staticmethod
    def get_data_port_model(state_m, data_port_id):
        """Searches and returns the model of a data port of a given state

        The method searches a port with the given id in the data ports of the given state model. If the state model
        is a container state, not only the input and output data ports are looked at, but also the scoped variables.
        :param state_m: The state model to search the data port in
        :param data_port_id: The data port id to be searched
        :return: The model of the data port or None if it is not found
        """
        def find_port_in_list(data_port_list):
            """Helper method to search for a port within a given list

            :param data_port_list: The list to search the data port in
            :return: The model of teh data port or None if it is not found
            """
            for port_m in data_port_list:
                if port_m.data_port.data_port_id == data_port_id:
                    return port_m
            return None

        if isinstance(state_m, ContainerStateModel):
            for scoped_var_m in state_m.scoped_variables:
                if scoped_var_m.scoped_variable.data_port_id == data_port_id:
                    return scoped_var_m
        if isinstance(state_m, StateModel):
            port_m = find_port_in_list(state_m.input_data_ports)
            if port_m is not None:
                return port_m
            port_m = find_port_in_list(state_m.output_data_ports)
            if port_m is not None:
                return port_m
        return None

    @staticmethod
    def get_transition_model(state_m, transition_id):
        """Searches and return the transition model with the given in the given container state model
        :param state_m: The state model to search the transition in
        :param transition_id: The transition id to be searched
        :return: The model of the transition or None if it is not found
        """
        if isinstance(state_m, ContainerStateModel):
            for transition_m in state_m.transitions:
                if transition_m.transition.transition_id == transition_id:
                    return transition_m
        return None

    @staticmethod
    def get_data_flow_model(state_m, data_flow_id):
        """Searches and return the data flow model with the given in the given container state model
        :param state_m: The state model to search the transition in
        :param data_flow_id: The data flow id to be searched
        :return: The model of the data flow or None if it is not found
        """
        if isinstance(state_m, ContainerStateModel):
            for data_flow_m in state_m.data_flows:
                if data_flow_m.data_flow.data_flow_id == data_flow_id:
                    return data_flow_m
        return None

    @staticmethod
    def change_state_type(state_m, new_state_class):

        assert isinstance(state_m, StateModel)
        assert issubclass(new_state_class, State)
        current_state_is_container = isinstance(state_m, ContainerStateModel)
        new_state_is_container = new_state_class in [HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState]

        is_root_state = state_m.parent is None

        state = state_m.state

        root_state_m = StateMachineHelper.get_root_state_model(state_m)
        for state_machine_m in state_machine_manager_model.state_machines.itervalues():
            if state_machine_m.root_state is root_state_m:
                break

        if not is_root_state:
            parent_m = state_m.parent
            parent = state.parent
            assert isinstance(parent, ContainerState)
        else:
            parent_m = state_machine_m

        state_machine_m.selection.remove(state_m)

        name = state.name
        state_id = state.state_id
        description = state.description
        inputs = state.input_data_ports
        outputs = state.output_data_ports
        outcomes = state.outcomes
        used_data_port_ids = state._used_data_port_ids
        used_outcome_ids = state._used_outcome_ids

        script = state.script

        copy_from_model = ['meta', 'input_data_ports', 'output_data_ports']

        if not is_root_state:
            connected_transitions = []
            for transition_m in parent_m.transitions:
                transition = transition_m.transition
                if transition.from_state == state_id or transition.to_state == state_id:
                    connected_transitions.append({"transition": transition, "meta": transition_m.meta})
            connected_data_flows = []
            for data_flow_m in parent_m.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    connected_data_flows.append({"data_flow": data_flow, "meta": data_flow_m.meta})

        if current_state_is_container and new_state_is_container:
            assert isinstance(state, ContainerState)
            start_state_id = state.start_state_id
            states = state.states
            scoped_vars = state.scoped_variables
            transitions = state.transitions
            data_flows = state.data_flows
            v_checker = state.v_checker

            copy_from_model.extend(['states', 'transitions', 'data_flows', 'scoped_variables'])

            new_state = new_state_class(name=name, state_id=state_id,
                                        input_data_ports=inputs, output_data_ports=outputs,
                                        outcomes=outcomes, states=states,
                                        transitions=transitions, data_flows=data_flows,
                                        start_state_id=start_state_id, scoped_variables=scoped_vars,
                                        v_checker=v_checker,
                                        path=script.path, filename=script.filename,
                                        check_path=False)
        else:
            new_state = new_state_class(name=name, state_id=state_id,
                                        input_data_ports=inputs, output_data_ports=outputs,
                                        outcomes=outcomes, path=script.path, filename=script.filename,
                                        check_path=False)

        new_state._used_data_port_ids = used_data_port_ids
        new_state._used_outcome_ids = used_outcome_ids

        if not is_root_state:
            new_state.parent = parent

        if description is not None and len(description) > 0:
            new_state.description = description

        model_properties = {}
        for prop_name in copy_from_model:
            model_properties[prop_name] = state_m.__getattribute__(prop_name)

        if not is_root_state:
            parent.remove_state(state_id, recursive_deletion=False)
            parent.add_state(new_state)

            new_state_m = parent_m.states[state_id]
            new_state_m.parent = parent_m
        else:
            parent_m.state_machine.root_state = new_state
            new_state_m = parent_m.root_state
            #parent_m.selection.set(state_m)

        for prop_name, value in model_properties.iteritems():
            new_state_m.__setattr__(prop_name, value)
            # Set the parent of all child models to the new state model
            if prop_name == "states":
                for state_m in new_state_m.states.itervalues():
                    state_m.parent = new_state_m
            if prop_name in ['input_data_ports', 'output_data_ports', 'transitions', 'data_flows', 'scoped_variables']:
                for model in new_state_m.__getattribute__(prop_name):
                    model.parent = new_state_m

        if not is_root_state:
            for transition_data in connected_transitions:
                t = transition_data["transition"]
                parent.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)
                transition_m = StateMachineHelper.get_transition_model(parent_m, t.transition_id)
                transition_m.meta = transition_data["meta"]

            for data_flow_data in connected_data_flows:
                d = data_flow_data["data_flow"]
                parent.add_data_flow(d.from_state, d.from_key, d.to_state, d.to_key, d.data_flow_id)
                data_flow_m = StateMachineHelper.get_data_flow_model(parent_m, d.data_flow_id)
                data_flow_m.meta = data_flow_data["meta"]


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
    def get_root_state_model(state_m):
        while state_m.parent is not None:
            state_m = state_m.parent
        return state_m



