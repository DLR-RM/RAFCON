from awesome_tool.utils import log

logger = log.get_logger(__name__)

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from awesome_tool.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from awesome_tool.statemachine.enums import StateType
from awesome_tool.mvc.models import StateModel, ContainerStateModel, TransitionModel, DataFlowModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel


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