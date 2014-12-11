import gobject
from statemachine.states.container_state import ContainerState
from mvc.models.state import StateModel
from gtk import ListStore


class ContainerStateModel(StateModel):
    """This model class manages a ContainerState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a container state).

    :param ContainerState container_state: The container state to be managed
     """

    container_state = None
    states = []
    transitions = []
    data_flows = []

    transition_list_store = ListStore(gobject.TYPE_PYOBJECT)  # Actually TransitionModel, but this is not supported
    # by GTK

    __observables__ = ("container_state", "states", "transitions", "data_flows")

    def __init__(self, container_state):
        """Constructor
        """

        assert isinstance(container_state, ContainerState)

        StateModel.__init__(self, container_state)

        self.container_state = container_state

        # Create model for each child class
        states = container_state.states
        for state in states:
            # Create hierarchy
            if isinstance(state, ContainerState):
                self.states.append(ContainerState(state))
            else:
                self.states.append(StateModel(state))

        for transition in container_state.transitions:
            self.transitions.append(transition)
            self.transition_list_store.append([transition])

        for data_flow in container_state.data_flows:
            self.data_flows.append(data_flow)

