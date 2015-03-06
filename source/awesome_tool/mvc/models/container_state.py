import gobject
from gtk import ListStore
from gtkmvc import ModelMT
import gtk

from statemachine.states.container_state import ContainerState
from statemachine.states.state import State
from statemachine.data_flow import DataFlow
from statemachine.transition import Transition
from statemachine.states.state import DataPort
from statemachine.scope import ScopedVariable
from mvc.models.state import StateModel
import mvc.models
from mvc.models.transition import TransitionModel
from mvc.models.data_flow import DataFlowModel
from mvc.models.data_port import DataPortModel
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
        StateModel.__init__(self, container_state, parent, meta)

        self.states = {}
        self.transitions = []
        self.data_flows = []
        self.scoped_variables = []
        # Actually Transition, but this is not supported by GTK
        self.transition_list_store = ListStore(gobject.TYPE_PYOBJECT)
        # Actually DataFlow, but this is not supported by
        self.data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.scoped_variables_list_store = ListStore(str, str, str, int)

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
        self.reload_scoped_variables_list_store_and_models()

    def reload_scoped_variables_list_store(self):
        """Reloads the scoped variable list store from the data port models
        """
        tmp = ListStore(str, str, str, int)
        for sv_model in self.scoped_variables:
            tmp.append([sv_model.scoped_variable.name, sv_model.scoped_variable.data_type,
                        sv_model.scoped_variable.default_value, sv_model.scoped_variable.data_port_id])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, mvc.models.state.dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.scoped_variables_list_store.clear()
        for elem in tmp:
            self.scoped_variables_list_store.append(elem)

    def reload_scoped_variables_models(self):
        """Reloads the scoped variable models directly from the the state
        """
        self.scoped_variables = []
        for scoped_variable in self.state.scoped_variables.itervalues():
            self.scoped_variables.append(ScopedVariableModel(scoped_variable, self))

    def reload_scoped_variables_list_store_and_models(self):
        """Reloads the scoped variable list store and models
        """
        self.reload_scoped_variables_models()
        self.reload_scoped_variables_list_store()

    def is_element_of_self(self, instance):

        #print info.method_name, info
        if isinstance(instance, DataFlow) and instance.data_flow_id in self.state.data_flows:
            return True
        if isinstance(instance, Transition) and instance.transition_id in self.state.transitions:
            return True
        if isinstance(instance, ScopedVariable) and instance.data_port_id in self.state.scoped_variables:
            return True
        return StateModel.is_element_of_self(self, instance)

    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, name, info):
        """ This method is always triggered when the state model changes

            It basically triggers its own parent, and the list of its state models
        """
        if self is not model and not self.is_element_of_self(info.instance):
            if hasattr(info, 'before') and info['before']:
                self.states._notify_method_before(self.state, "state_change", (model,), info)
            elif hasattr(info, 'after') and info['after']:
                self.states._notify_method_after(self.state, "state_change", None, (model,), info)

        # TODO delete # prints if there is no bug ... latest 15th of march
        if hasattr(info, 'before') and info['before']:

            if "modify_data_flow" in info.method_name and self is model or \
                    isinstance(info.instance, DataFlow) and info.instance.data_flow_id in self.state.data_flows:
                # print "NOTIFY DATA_FLOWS before"
                self.data_flows._notify_method_before(info.instance, "data_flow_change", (model,), info)
            if "modify_transition" in info.method_name and self is model or \
                    isinstance(info.instance, Transition) and info.instance.transition_id in self.state.transitions:
                # print "NOTIFY TRANSITIONS Before"
                self.transitions._notify_method_before(info.instance, "transition_change", (model,), info)
            if "modify_scoped_variable" in info.method_name and self is model or \
                    isinstance(info.instance, ScopedVariable) and info.instance.data_port_id in self.state.scoped_variables:
                # print "NOTIFY before SCOPED_VARIABLES"
                self.scoped_variables._notify_method_before(info.instance, "scoped_variable_change", (model,), info)

        elif hasattr(info, 'after') and info['after']:
            if "modify_data_flow" in info.method_name and self is model or \
                    isinstance(info.instance, DataFlow) and info.instance.data_flow_id in self.state.data_flows:
                # print "NOTIFY DATA_FLOWS after"
                self.data_flows._notify_method_after(info.instance, "data_flow_change", None, (model,), info)
            if "modify_transition" in info.method_name and self is model or \
                    isinstance(info.instance, Transition) and info.instance.transition_id in self.state.transitions:
                # print "NOTIFY TRANSITIONS after"
                self.transitions._notify_method_after(info.instance, "transition_change", None, (model,), info)
            if "modify_scoped_variable" in info.method_name and self is model or \
                    isinstance(info.instance, ScopedVariable) and info.instance.data_port_id in self.state.scoped_variables:
                # print "NOTIFY after SCOPED_VARIABLES"
                self.scoped_variables._notify_method_after(info.instance, "scoped_variable_change", None, (model,), info)

        StateModel.model_changed(self, model, name, info)

    @ModelMT.observe("state", after=True)
    def update_child_models(self, _, name, info):
        """ This method is always triggered when the state model changes

            It keeps the following models/model-lists consistent:
            transition models
            data-flow models
            state models
            scoped variable models
        """

        if info.method_name == 'start_state':
            c_state_m = None
            if self.state.start_state:
                c_state_m = self.states[self.state.start_state]
                c_state_m.is_start = True if c_state_m.parent is None or \
                                             c_state_m.state.state_id == c_state_m.parent.state.start_state else False
            # remove other state_models is_start flag
            state_list = filter(lambda state_model: state_model.is_start and state_model is not c_state_m, self.states.values())
            if len(state_list) > 1:
                logger.warning("There are more then one start state.")
            for state_model in state_list:
                logger.debug("State %s %s is no longer start state" % (state_model.state.state_id,
                                                                       state_model.state.name))
                state_model.is_start = False

        model_list = None
        
        # TODO to lower computation load only called if reasonable
        # if not info.method_name in ['add_data_flow', 'remove_data_flow',
        #                             'add_transition', 'remove_transition',
        #                             'add_scoped_variable', 'remove_scoped_variable']:  # container_state-functions
        #     StateModel.update_models(self, _, name, info)
        StateModel.update_models(self, _, name, info)

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
            elif model == "data_flow":
                model_list = self.data_flows
                data_list = self.state.data_flows
                model_name = "data_flow"
                model_class = DataFlowModel
            elif model == "scoped_variable":
                model_list = self.scoped_variables
                data_list = self.state.scoped_variables
                model_name = "scoped_variable"
                model_class = ScopedVariableModel
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
        elif "scoped_variable" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = get_model_info("scoped_variable")

        if model_list is not None:
            if "add" in info.method_name:
                self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
            elif "remove" in info.method_name:
                self.remove_additional_model(model_list, data_list, model_name, model_key)

    @staticmethod
    def state_to_state_model(state):
        if isinstance(state, ContainerState):
            return ContainerStateModel
        elif isinstance(state, State):
            return StateModel
        else:
            return None

    # ---------------------------------------- storage functions ---------------------------------------------
    def load_meta_data_for_state(self):
        #logger.debug("load recursively graphics file from yaml for state model of state %s" % self.state.name)
        StateModel.load_meta_data_for_state(self)
        for state_key, state in self.states.iteritems():
            state.load_meta_data_for_state()

    def store_meta_data_for_state(self):
        #logger.debug("store recursively graphics file to yaml for state model of state %s" % self.state.name)
        StateModel.store_meta_data_for_state(self)
        for state_key, state in self.states.iteritems():
            state.store_meta_data_for_state()
