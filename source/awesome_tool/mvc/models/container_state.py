import gobject
from gtk import ListStore
from gtkmvc import ModelMT
import gtk

from statemachine.states.container_state import ContainerState
from statemachine.states.state import State
from mvc.models.state import StateModel
import mvc.models
from mvc.models.transition import TransitionModel
from mvc.models.data_flow import DataFlowModel
from mvc.models.scoped_variable import ScopedVariableModel
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
    scoped_variables = []

    __observables__ = ("states", "transitions", "data_flows", "scoped_variables")

    def __init__(self, container_state, parent=None, meta=None):
        """Constructor
        """
        assert isinstance(container_state, ContainerState)
        #ContainerState.__init__(self, container_state, parent, meta)
        StateModel.__init__(self, container_state, parent, meta)

        self.states = {}
        self.transitions = []
        self.data_flows = []
        self.scoped_variables = []
        # Actually Transition, but this is not supported by GTK
        self.transition_list_store = ListStore(gobject.TYPE_PYOBJECT)
        # Actually DataFlow, but this is not supported by
        self.data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.scoped_variables_list_store = ListStore(gobject.TYPE_PYOBJECT)

        # Create model for each child class
        states = container_state.states
        for state in states.itervalues():
            # Create hierarchy
            model_class = self.state_to_state_model(state)
            if model_class is not None:
                self.states[state.state_id] = model_class(state, parent=self)
            # if isinstance(state, ContainerState):
            #     self.states[state.state_id] = ContainerStateModel(state, parent=self)
            # # elif isinstance(state, HierarchyState):
            # #     self.states.append(ContainerState(state))
            # elif isinstance(state, State):
            #     self.states[state.state_id] = StateModel(state, parent=self)
            else:
                logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))
                logger.error(state)

        for transition in container_state.transitions.itervalues():
            self.transitions.append(TransitionModel(transition, self))
            self.transition_list_store.append([transition])

        for data_flow in container_state.data_flows.itervalues():
            self.data_flows.append(DataFlowModel(data_flow, self))
            self.data_flow_list_store.append([data_flow])

        # this class is an observer of its own properties:
        self.register_observer(self)
        self.update_scoped_variables_list_store_and_models()

    def update_scoped_variables_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        for scoped_variable in self.container_state.scoped_variables.itervalues():
            tmp.append([scoped_variable])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, mvc.models.state.dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.scoped_variables_list_store.clear()
        for elem in tmp:
            self.scoped_variables_list_store.append(elem)

    def update_scoped_variables_models(self):
        self.scoped_variables = []
        for scoped_variable in self.container_state.scoped_variables.itervalues():
            self.scoped_variables.append(ScopedVariableModel(scoped_variable, self))

    def update_scoped_variables_list_store_and_models(self):
        self.update_scoped_variables_models()
        self.update_scoped_variables_list_store()


    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, name, info):
        """ This method is always triggered when the state model changes

            It basically triggers its own parent, and the list of its state models
        """
        if self is not model:
            if hasattr(info, 'before') and info['before']:
                self.states._notify_method_before(self.state, "state_change", (model,), info)
            elif hasattr(info, 'after') and info['after']:
                self.states._notify_method_after(self.state, "state_change", None, (model,), info)
        if self.parent is not None:
            self.parent.model_changed(model, name, info)

    @ModelMT.observe("state", after=True)
    def update_child_models(self, _, name, info):
        """ This method is always triggered when the state model changes

            It keeps the following models/model-lists consistent:
            transition models
            data-flow models
            state models
            scoped variable models
        """
        #TODO: scoped variables
        model_list = None

        def get_model_info(model):
            model_list = None
            data_list = None
            model_name = ""
            model_class = None
            model_key = None
            if model == "transition":
                model_list = self.transitions
                data_list = self.state.transitions
                model_name = "transition"
                model_class = TransitionModel
            elif model ==  "data_flow":
                model_list = self.data_flows
                data_list = self.state.data_flows
                model_name = "data_flow"
                model_class = DataFlowModel
            elif model == "state":
                model_list = self.states
                data_list = self.state.states
                model_name = "state"
                # Defer state type from class type (Execution, Hierarchy, ...)
                model_class = self.state_to_state_model(info.args[1])
                model_key = "state_id"
            return model_list, data_list, model_name, model_class, model_key

        if "transition" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = get_model_info("transition")
        elif "data_flow" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = get_model_info("data_flow")
        elif "state" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = get_model_info("state")

        if model_list is not None:
            if "add" in info.method_name:
                self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
            elif "remove" in info.method_name:
                self.remove_additional_model(model_list, data_list, model_name, model_key)

        if info.method_name in ("remove_state", "remove_outcome"):
            (model_list, data_list, model_name, _, model_key) = get_model_info("transition")
            while True:
                num_transitions = len(self.transitions)
                self.remove_additional_model(model_list, data_list, model_name, model_key)
                if len(self.transitions) == num_transitions:
                    break

        if info.method_name in ("remove_state", "remove_scoped_variable", "remove_input_data_port",
                                "remove_output_data_port"):
            (model_list, data_list, model_name, _, model_key) = get_model_info("data_flow")
            while True:
                num_data_flows = len(self.data_flows)
                self.remove_additional_model(model_list, data_list, model_name, model_key)
                if len(self.data_flows) == num_data_flows:
                    break

    @staticmethod
    def state_to_state_model(state):
        if isinstance(state, ContainerState):
            return ContainerStateModel
        elif isinstance(state, State):
            return StateModel
        else:
            return None
