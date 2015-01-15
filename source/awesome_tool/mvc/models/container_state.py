import gobject
from statemachine.states.container_state import ContainerState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.state import State
from mvc.models.state import StateModel
from mvc.models.transition import TransitionModel
from mvc.models.data_flow import DataFlowModel
from gtk import ListStore

from utils import log
logger = log.get_logger(__name__)


class ContainerStateModel(StateModel):
    """This model class manages a ContainerState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a container state).

    :param ContainerState container_state: The container state to be managed
     """

    #container_state = None
    states = []
    transitions = []
    data_flows = []

    transition_list_store = ListStore(gobject.TYPE_PYOBJECT)  # Actually Transition, but this is not supported by GTK
    data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)  # Actually DataFlow, but this is not supported by GTK

    __observables__ = ("states", "transitions", "data_flows")

    def __init__(self, container_state, parent=None, meta=None):
        """Constructor
        """

        assert isinstance(container_state, ContainerState)

        StateModel.__init__(self, container_state, parent, meta)
        self.states = {}
        self.transitions = []
        self.data_flows = []

        #self.state = container_state

        # Create model for each child class
        states = container_state.states
        for state in states.itervalues():
            # Create hierarchy
            if isinstance(state, ContainerState):
                self.states[state.state_id] = ContainerStateModel(state, parent=self)
            # elif isinstance(state, HierarchyState):
            #     self.states.append(ContainerState(state))
            elif isinstance(state, State):
                self.states[state.state_id] = StateModel(state, parent=self)
            else:
                logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))
                logger.error(state)


        for transition in container_state.transitions.itervalues():
            self.transitions.append(TransitionModel(transition, self))
            self.transition_list_store.append([transition])

        for data_flow in container_state.data_flows.itervalues():
            self.data_flows.append(DataFlowModel(data_flow, self))
            self.data_flow_list_store.append([data_flow])
