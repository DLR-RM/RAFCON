import gobject
from gtk import ListStore
from gtkmvc import ModelMT
import gtk

from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.data_flow import DataFlow
from awesome_tool.statemachine.transition import Transition
from awesome_tool.statemachine.data_port import DataPort
from awesome_tool.statemachine.scope import ScopedVariable
from awesome_tool.mvc.models.state import StateModel
import awesome_tool.mvc.models
from awesome_tool.mvc.models.transition import TransitionModel
from awesome_tool.mvc.models.data_flow import DataFlowModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel
from awesome_tool.utils import log

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

        # Create model for each child class
        states = container_state.states
        for state in states.itervalues():
            # Create hierarchy
            model_class = self.state_to_state_model(state)
            if model_class is not None:
                self.states[state.state_id] = model_class(state, parent=self)
            # if isinstance(state, ContainerState):
            # self.states[state.state_id] = ContainerStateModel(state, parent=self)
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
        self.reload_scoped_variables_models()

    def reload_scoped_variables_models(self):
        """Reloads the scoped variable models directly from the the state
        """
        self.scoped_variables = []
        for scoped_variable in self.state.scoped_variables.itervalues():
            self.scoped_variables.append(ScopedVariableModel(scoped_variable, self))


    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        """This method notifies the model lists and the parent state about changes

        The method is called each time, the model is changed. This happens, when the state itself changes or one of
        its children (states, transitions, data flows) changes. Changes of the children cannot be observed directly,
        therefore children notify their parent about their changes by calling this method.
        This method then checks, what has been changed by looking at the model that is passed to it. In the following it
        notifies the list in which the change happened about the change.
        E.g. one child state changes its name. The model of that state observes itself and notifies the parent (
        i.e. this state model) about the change by calling this method with the information about the change. This
        method recognizes that the model is of type StateModel and therefore triggers a notify on the list of state
        models.
        "_notify_method_before" is used as trigger method when the changing function is entered and
        "_notify_method_after" is used when the changing function returns. This changing function in the example
        would be the setter of the property name.
        :param model: The model that was changed
        :param prop_name: The property that was changed
        :param info: Information about the change (e.g. the name of the changing function)
        """

        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may relay on the updated models
        self.update_child_models(model, prop_name, info)

        changed_list = None
        cause = None
        # If the change happened in a child state, notify the list of all child states
        if (isinstance(model, StateModel) and model is not self) or (  # The state was changed directly
                not isinstance(model, StateModel) and model.parent is not self):  # One of the member models was changed
            changed_list = self.states
            cause = 'state_change'
        # If the change happened in one of the transitions, notify the list of all transitions
        elif isinstance(model, TransitionModel) and model.parent is self:
            changed_list = self.transitions
            cause = 'transition_change'
        # If the change happened in one of the data flows, notify the list of all data flows
        elif isinstance(model, DataFlowModel) and model.parent is self:
            changed_list = self.data_flows
            cause = 'data_flow_change'
        # If the change happened in one of the scoped variables, notify the list of all scoped variables
        elif isinstance(model, ScopedVariableModel) and model.parent is self:
            changed_list = self.scoped_variables
            cause = 'scoped_variable_change'

        if not (cause is None or changed_list is None):
            if hasattr(info, 'before') and info['before']:
                changed_list._notify_method_before(info.instance, cause, (model,), info)
            elif hasattr(info, 'after') and info['after']:
                changed_list._notify_method_after(info.instance, cause, None, (model,), info)

        # Finally call the method of the base class, to forward changes in ports and outcomes
        StateModel.model_changed(self, model, prop_name, info)

    def update_child_models(self, _, name, info):
        """ This method is always triggered when the state model changes

            It keeps the following models/model-lists consistent:
            transition models
            data-flow models
            state models
            scoped variable models
        """

        # Update is_start flag in child states if the start state has changed (eventually)
        if info.method_name in ['start_state_id', 'add_transition', 'remove_transition']:
            start_state_id = self.state.start_state_id
            for state_id, state_m in self.states.iteritems():
                if state_m.is_start != (state_id == start_state_id):
                    state_m.is_start = (state_id == start_state_id)

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
        # logger.debug("load recursively graphics file from yaml for state model of state %s" % self.state.name)
        StateModel.load_meta_data_for_state(self)
        for state_key, state in self.states.iteritems():
            state.load_meta_data_for_state()

    def store_meta_data_for_state(self):
        # logger.debug("store recursively graphics file to yaml for state model of state %s" % self.state.name)
        StateModel.store_meta_data_for_state(self)
        for state_key, state in self.states.iteritems():
            state.store_meta_data_for_state()

    def copy_meta_data_from_state_model(self, source_state):
        StateModel.copy_meta_data_from_state_model(self, source_state)
        for state_key, state in self.states.iteritems():
            state.copy_meta_data_from_state_model(source_state.states[state_key])
