from copy import deepcopy
from os.path import join

from gtkmvc import ModelMT

from rafcon.statemachine.states.container_state import ContainerState

from rafcon.mvc.models.state import StateModel
from rafcon.mvc.models.transition import TransitionModel
from rafcon.mvc.models.data_flow import DataFlowModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel

from rafcon.mvc.models.abstract_state import state_to_state_model, StateTypeChangeSignalMsg

from rafcon.utils import log
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

    def __init__(self, container_state, parent=None, meta=None, load_meta_data=True):
        """Constructor
        """
        assert isinstance(container_state, ContainerState)
        StateModel.__init__(self, container_state, parent, meta)

        self.states = {}
        self.transitions = []
        self.data_flows = []
        self.scoped_variables = []

        # Create model for each child class
        states = container_state.states
        for state in states.itervalues():
            # Create hierarchy
            model_class = state_to_state_model(state)
            if model_class is not None:
                self.states[state.state_id] = model_class(state, parent=self)
            else:
                logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))

        for transition in container_state.transitions.itervalues():
            self.transitions.append(TransitionModel(transition, self))

        for data_flow in container_state.data_flows.itervalues():
            self.data_flows.append(DataFlowModel(data_flow, self))

        for scoped_variable in self.state.scoped_variables.itervalues():
            self.scoped_variables.append(ScopedVariableModel(scoped_variable, self))

        if load_meta_data:
            self.load_meta_data()

        # this class is an observer of its own properties:
        self.register_observer(self)

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
        if info.method_name == 'change_state_type':  # Handled in method 'change_state_type'
            return

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
                model_class = state_to_state_model(info.args[1])
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

    @ModelMT.observe("state", after=True, before=True)
    def change_state_type(self, model, prop_name, info):
        if info.method_name != 'change_state_type':
            return
        from rafcon.mvc import statemachine_helper

        old_state = info.args[1]
        new_state_class = info.args[2]
        state_id = old_state.state_id
        state_m = self.states[state_id]

        # Before the state type is actually changed, we extract the information from the old state model and remove
        # the model from the selection
        if hasattr(info, 'before') and info['before']:
            # remove selection from StateMachineModel.selection -> find state machine model
            from rafcon.mvc.singleton import state_machine_manager_model
            state_machine_m = state_machine_manager_model.get_sm_m_for_state_model(state_m)
            state_machine_m.selection.remove(state_m)

            # Extract child models of state, as they have to be applied to the new state model
            child_models = statemachine_helper.extract_child_models_of_of_state(state_m, new_state_class)
            self.change_state_type.__func__.child_models = child_models  # static variable of class method

        # After the state has been changed in the core, we create a new model for it with all information extracted
        # from the old state model
        else:  # after
            # The new state is returned by the core state class method 'change_state_type'
            new_state = info.result
            # Create a new state model based on the new state and apply the extracted child models
            child_models = self.change_state_type.__func__.child_models
            new_state_m = statemachine_helper.create_state_model_for_state(new_state, child_models)
            # Set this state model (self) to be the parent of our new state model
            new_state_m.parent = self
            # Access states dict without causing a notifications. The dict is wrapped in a ObsMapWrapper object.
            self.states._obj.__setitem__(state_id, new_state_m)

            state_m.state_type_changed_signal.emit(StateTypeChangeSignalMsg(new_state_m))

        self.model_changed(model, prop_name, info)

    def get_scoped_variable_m(self, data_port_id):
        """Returns the scoped variable model for the given data port id

        :param data_port_id: The data port id to search for
        :return: The model of the scoped variable with the given id
        """
        for scoped_variable_m in self.scoped_variables:
            if scoped_variable_m.scoped_variable.data_port_id == data_port_id:
                return scoped_variable_m
        return None

    def get_data_port_m(self, data_port_id):
        """Searches and returns the model of a data port of a given state

        The method searches a port with the given id in the data ports of the given state model. If the state model
        is a container state, not only the input and output data ports are looked at, but also the scoped variables.

        :param data_port_id: The data port id to be searched
        :return: The model of the data port or None if it is not found
        """

        for scoped_var_m in self.scoped_variables:
            if scoped_var_m.scoped_variable.data_port_id == data_port_id:
                return scoped_var_m

        return StateModel.get_data_port_m(self, data_port_id)

    def get_transition_m(self, transition_id):
        """Searches and return the transition model with the given in the given container state model

        :param transition_id: The transition id to be searched
        :return: The model of the transition or None if it is not found
        """
        for transition_m in self.transitions:
            if transition_m.transition.transition_id == transition_id:
                return transition_m
        return None

    def get_data_flow_m(self, data_flow_id):
        """Searches and return the data flow model with the given in the given container state model

        :param data_flow_id: The data flow id to be searched
        :return: The model of the data flow or None if it is not found
        """
        for data_flow_m in self.data_flows:
            if data_flow_m.data_flow.data_flow_id == data_flow_id:
                return data_flow_m
        return None

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of container states from filesystem

        Recursively loads meta data of child states.
        """
        super(ContainerStateModel, self).load_meta_data(path)
        for state_key, state_m in self.states.iteritems():
            child_path = None if not path else join(path, state_key)
            state_m.load_meta_data(child_path)

    def store_meta_data(self):
        """Store meta data of container states to the filesystem

        Recursively stores meta data of child states.
        """
        super(ContainerStateModel, self).store_meta_data()
        for state_key, state in self.states.iteritems():
            state.store_meta_data()

    def copy_meta_data_from_state_m(self, source_state_m):
        """Dismiss current meta data and copy meta data from given state model

        In addition to the state model method, also the meta data of container states is copied. Then, the meta data
        of child states are recursively copied.

        :param source_state_m: State model to load the meta data from
        """
        for scoped_variable_m in self.data_flows:
            source_scoped_variable_m = source_state_m.get_scoped_variable_m(
                scoped_variable_m.scoped_variable.data_flow_id)
            scoped_variable_m.meta = deepcopy(source_scoped_variable_m.meta)

        for transition_m in self.transitions:
            source_transition_m = source_state_m.get_transition_m(transition_m.transition.transition_id)
            transition_m.meta = deepcopy(source_transition_m.meta)

        for data_flow_m in self.data_flows:
            source_data_flow_m = source_state_m.get_data_flow_m(data_flow_m.data_flow.data_flow_id)
            data_flow_m.meta = deepcopy(source_data_flow_m.meta)

        for state_key, state in self.states.iteritems():
            state.copy_meta_data_from_state_m(source_state_m.states[state_key])

        super(ContainerStateModel, self).copy_meta_data_from_state_m(source_state_m)

    def _parse_for_element_meta_data(self, meta_data):
        """Load meta data for container state elements

        In addition to the meta data of states, this method also parses for meta data of container state elements.

        :param meta_data: Dictionary of loaded meta data
        """
        super(ContainerStateModel, self)._parse_for_element_meta_data(meta_data)
        for transition_m in self.transitions:
            self._copy_element_meta_data_from_meta_file_data(meta_data, transition_m, "transition",
                                                             transition_m.transition.transition_id)
        for data_flow_m in self.data_flows:
            self._copy_element_meta_data_from_meta_file_data(meta_data, data_flow_m, "data_flow",
                                                             data_flow_m.data_flow.data_flow_id)
        for scoped_variable_m in self.scoped_variables:
            self._copy_element_meta_data_from_meta_file_data(meta_data, scoped_variable_m, "scoped_variable",
                                         scoped_variable_m.scoped_variable.data_port_id)

    def _generate_element_meta_data(self, meta_data):
        """Generate meta data for state elements and add it to the given dictionary

        This method retrieves the meta data of the container state elements (data flows, transitions) and adds it
        to the given meta data dictionary.

        :param meta_data: Dictionary of meta data
        """
        super(ContainerStateModel, self)._generate_element_meta_data(meta_data)
        for transition_m in self.transitions:
            self._copy_element_meta_data_to_meta_file_data(meta_data, transition_m, "transition",
                                                           transition_m.transition.transition_id)
        for data_flow_m in self.data_flows:
            self._copy_element_meta_data_to_meta_file_data(meta_data, data_flow_m, "data_flow",
                                                           data_flow_m.data_flow.data_flow_id)
        for scoped_variable_m in self.scoped_variables:
            self._copy_element_meta_data_to_meta_file_data(meta_data, scoped_variable_m, "scoped_variable",
                                                           scoped_variable_m.scoped_variable.data_port_id)
