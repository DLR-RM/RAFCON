from utils import log

logger = log.get_logger(__name__)

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from statemachine.enums import StateType
from mvc.models import StateModel, ContainerStateModel, TransitionModel, DataFlowModel


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
        if isinstance(model, StateModel):
            if container_m is not None:
                try:
                    container_m.state.remove_state(model.state.state_id)
                    return True
                except AttributeError as e:
                    logger.error("The state with the ID {0} and the name {1} could not be deleted: {2}".format(
                        model.state.state_id, model.state.name, e.message))

        elif isinstance(model, TransitionModel):
            try:
                container_m.state.remove_transition(model.transition.transition_id)
                return True
            except AttributeError as e:
                logger.error("The transition with the ID {0} could not be deleted: {1}".format(
                    model.transition.transition_id, e.message))

        elif isinstance(model, DataFlowModel):
            try:
                container_m.state.remove_data_flow(model.data_flow.data_flow_id)
                return True
            except AttributeError as e:
                logger.error("The data flow with the ID {0} could not be deleted: {1}".format(
                    model.data_flow.data_flow_id, e.message))

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
        :param mvc.models.container_state.ContainerState container_state: A model of a container state to add the new
        state to
        :param statemachine.enums.StateType state_type: The type of state that should be added
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