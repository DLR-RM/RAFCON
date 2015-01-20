import gobject
from statemachine.states.container_state import ContainerState
from statemachine.states.state import State
from mvc.models.state import StateModel
from mvc.models.transition import TransitionModel
from mvc.models.data_flow import DataFlowModel
from mvc.models.data_port import DataPortModel
from mvc.models.scoped_variable import ScopedVariableModel
from gtk import ListStore
from gtkmvc import ModelMT
import gtk

from utils import log
logger = log.get_logger(__name__)


class ContainerStateModel(StateModel):
    """This model class manages a ContainerState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a container state).

    :param ContainerState container_state: The container state to be managed
     """

    states = {}
    transitions = []
    data_flows = []
    input_data_ports = []
    output_data_ports = []
    scoped_variables = []

    __observables__ = ("states", "transitions", "data_flows", "input_data_ports",
                       "output_data_ports", "scoped_variables")

    def __init__(self, container_state, parent=None, meta=None):
        """Constructor
        """
        assert isinstance(container_state, ContainerState)
        #ContainerState.__init__(self, container_state, parent, meta)
        StateModel.__init__(self, container_state, parent, meta)

        self.container_state = container_state

        self.states = {}
        self.transitions = []
        self.data_flows = []
        self.input_data_ports = []
        self.output_data_ports = []
        self.scoped_variables = []
        # Actually Transition, but this is not supported by GTK
        self.transition_list_store = ListStore(gobject.TYPE_PYOBJECT)
        # Actually DataFlow, but this is not supported by
        self.data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.input_data_port_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.output_data_port_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.scoped_variables_list_store = ListStore(gobject.TYPE_PYOBJECT)

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
            print "DataFlows", self.data_flows
            self.data_flow_list_store.append([data_flow])

        # this class is an observer of its own properties:
        self.register_observer(self)
        self.update_input_data_port_list_store()
        self.update_output_data_port_list_store()
        self.update_scoped_variables_list_store()

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
        tmp = tms
        self.input_data_port_list_store.clear()
        for elem in tmp:
            self.input_data_port_list_store.append(elem)

    def update_output_data_port_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))
            tmp.append([output_data_port])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, self.comparemethod)
        tms.sort_column_changed()
        tmp = tms
        self.output_data_port_list_store.clear()
        for elem in tmp:
            self.output_data_port_list_store.append(elem)

    def update_scoped_variables_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        self.scoped_variables = []
        for scoped_variable in self.container_state.scoped_variables.itervalues():
            self.scoped_variables.append(ScopedVariableModel(scoped_variable, self))
            tmp.append([scoped_variable])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, self.comparemethod)
        tms.sort_column_changed()
        tmp = tms
        self.scoped_variables_list_store.clear()
        for elem in tmp:
            self.scoped_variables_list_store.append(elem)

    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, name, info):
        if self is not model:
            if hasattr(info, 'before') and info['before']:
                self.states._notify_method_before(self.state, "state_change", (model,), info)
            elif hasattr(info, 'after') and info['after']:
                self.states._notify_method_after(self.state, "state_change", None, (model,), info)
        if self.parent is not None:
            self.parent.model_changed(model, name, info)

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

    @ModelMT.observe("state", after=True)
    def update_child_models(self, model, name, info):
        #print info

        model_list = []
        data_list = []
        model_name = ""
        model_class = None
        if "transition" in info.method_name:
            model_list = self.transitions
            data_list = self.state.transitions
            model_name = "transition"
            model_class = TransitionModel
        elif "data_flow" in info.method_name:
            model_list = self.data_flows
            data_list = self.state.data_flows
            model_name = "data_flow"
            model_class = DataFlowModel

        if model_name is not "":
            #print "before", model_list
            if "add" in info.method_name:
                #print "add", model_name
                self.add_missing_model(model_list, data_list, model_name, model_class)
            elif "remove" in info.method_name:
                #print "remove", model_name
                self.remove_additional_model(model_list, data_list, model_name)
            print "after", model_list

    def add_missing_model(self, model_list, data_list, model_name, model_class):
        for data in data_list.itervalues():
            found = False
            for model in model_list:
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                model_list.append(model_class(data, self))
                return

    def remove_additional_model(self, model_list, data_list, model_name):
        for model in model_list:
            for data in data_list.itervalues():
                if data is getattr(model, model_name):
                    model_list.remove(model)
                    return
