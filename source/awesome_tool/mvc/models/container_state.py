import gobject
from statemachine.states.container_state import ContainerState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.state import State
from mvc.models.state import StateModel
from mvc.models.transition import TransitionModel
from mvc.models.data_flow import DataFlowModel
from mvc.models.data_port import DataPortModel
from gtk import ListStore
import gtk
from gtkmvc import Observer

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
    input_data_ports = []

    transition_list_store = ListStore(gobject.TYPE_PYOBJECT)  # Actually Transition, but this is not supported by GTK
    data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)  # Actually DataFlow, but this is not supported by GTK
    input_data_port_list_store = ListStore(gobject.TYPE_PYOBJECT)
    #state_input_data_ports = {}

    __observables__ = ("states", "transitions", "data_flows", "input_data_ports")
    #__observables__ = ("states", "transitions", "data_flows", "input_data_ports", "state_input_data_ports")

    def __init__(self, container_state, parent=None, meta=None):
        """Constructor
        """

        assert isinstance(container_state, ContainerState)

        StateModel.__init__(self, container_state, parent, meta)
        self.states = {}
        self.transitions = []
        self.data_flows = []
        self.input_data_ports = []
        #self.state_input_data_ports = self.state.input_data_ports

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
            self.data_flows.append(DataFlowModel(data_flow))
            self.data_flow_list_store.append([data_flow])

        self.update_input_data_port_list_store()

    def update_input_data_port_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))
            tmp.append([input_data_port])

        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, self.comparemethod)
        tms.sort_column_changed()
        self.input_data_port_list_store = tms

    def comparemethod(self, treemodel, iter1, iter2, user_data=None):
        path1 = treemodel.get_path(iter1)[0]
        path2 = treemodel.get_path(iter2)[0]
        name1 = treemodel[path1][0].name
        name2 = treemodel[path2][0].name
        name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
        name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
        if name1_as_bits == name2_as_bits:
            return 0
        elif name1_as_bits > name2_as_bits:
            return 1
        else:
            return -1
