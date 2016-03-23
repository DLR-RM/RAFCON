from gtkmvc import ModelMT
from rafcon.mvc.models.abstract_state import AbstractStateModel

from rafcon.statemachine.outcome import Outcome

from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.outcome import OutcomeModel

from rafcon.utils import log
logger = log.get_logger(__name__)


class StateModel(AbstractStateModel):
    """This model class manages a State, for the moment only ExecutionStates

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param rafcon.statemachine.states.state.State state: The state to be managed
    :param AbstractStateModel parent: The state to be managed
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state
     """

    def __init__(self, state, parent=None, meta=None, load_meta_data=True):
        """Constructor"""
        super(StateModel, self).__init__(state, parent, meta)

        if load_meta_data and type(self) == StateModel:
            self.load_meta_data()

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, prop_name, info):
        """This method notifies the model lists and the parent state about changes

        The method is called each time, the model is changed. This happens, when the state itself changes or one of
        its children (outcomes, ports) changes. Changes of the children cannot be observed directly,
        therefore children notify their parent about their changes by calling this method.
        This method then checks, what has been changed by looking at the method that caused the change. In the
        following, it notifies the list in which the change happened about the change.
        E.g. one input data port changes its name. The model of the port observes itself and notifies the parent (
        i.e. the state model) about the change by calling this method with the information about the change. This
        method recognizes that the method "modify_input_data_port" caused the change and therefore triggers a notify
        on the list if input data port models.
        "_notify_method_before" is used as trigger method when the changing function is entered and
        "_notify_method_after" is used when the changing function returns. This changing function in the example
        would be "modify_input_data_port".

        :param model: The model that was changed
        :param prop_name: The property that was changed
        :param info: Information about the change (e.g. the name of the changing function)
        """
        # logger.info("STATEMODEL {0} {1}".format(0,1)) if 'after' in info else logger.info("STATEMODEL {0} {1}".format(1,0))
        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may relay on the updated models
        if 'after' in info and self.state == info['instance']:
            self.update_models(model, prop_name, info)

        # mark the state machine this state belongs to as dirty
        no_save_change = info["method_name"] in ["active", "child_execution", "state_execution_status"]
        if isinstance(model, AbstractStateModel) and prop_name == "state" and no_save_change:
            # do not track the active flag when marking the sm dirty
            pass
        else:
            # if the state_execution state is changed the sm must not be marked dirty
            if "after" in info and info["method_name"] != "state_execution_status":
                self._mark_state_machine_as_dirty()

        # TODO the modify observation to notify the list has to be changed in the manner, that the element-models
        # notify there parent with there own instance as argument

        changed_list = None
        cause = None

        if isinstance(model, DataPortModel) and model.parent is self:
            if model in self.input_data_ports:
                changed_list = self.input_data_ports
                cause = "input_data_port_change"
            elif model in self.output_data_ports:
                changed_list = self.output_data_ports
                cause = "output_data_port_change"

        # Outcomes are not modelled as class, therefore the model passed when adding/removing an outcome is the
        # parent StateModel. Thus we have to look at the method that caused a property change.
        elif isinstance(info.instance, Outcome) and self is model.parent:
            changed_list = self.outcomes
            cause = "outcome_change"

        if not (cause is None or changed_list is None):
            if 'before' in info:
                changed_list._notify_method_before(self.state, cause, (self.state,), info)
            elif 'after' in info:
                changed_list._notify_method_after(self.state, cause, None, (self.state,), info)

        # Notifies parent state
        super(StateModel, self).model_changed(model, prop_name, info)

    def get_model_info(self, model):
        model_key = None
        if model == "input_data_port":
            model_list = self.input_data_ports
            data_list = self.state.input_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        elif model == "output_data_port":
            model_list = self.output_data_ports
            data_list = self.state.output_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        elif model == "outcome":
            model_list = self.outcomes
            data_list = self.state.outcomes
            model_name = "outcome"
            model_class = OutcomeModel
        else:
            raise AttributeError("Wrong model specified!")
        return model_list, data_list, model_name, model_class, model_key

    def update_models(self, model, name, info):
        """ This method is always triggered when the core state changes

            It keeps the following models/model-lists consistent:
            input-data-port models
            output-data-port models
            outcome models
        """
        # logger.info("method_name: " + info.method_name)
        model_list = None
        if "input_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("input_data_port")
        elif "output_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("output_data_port")
        elif "outcome" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("outcome")

        if model_list is not None:
            if "add" in info.method_name:
                self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
            elif "remove" in info.method_name:
                self.remove_additional_model(model_list, data_list, model_name, model_key)

    def _load_input_data_port_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def _load_output_data_port_models(self):
        """Reloads the output data port models directly from the the state
        """
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def _load_outcome_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.outcomes = []
        for outcome in self.state.outcomes.itervalues():
            self.outcomes.append(OutcomeModel(outcome, self))

    def add_missing_model(self, model_list_or_dict, core_objects_dict, model_name, model_class, model_key):
        """
        :param model_list_or_dict: could be a list or dictionary of one model type
        :param core_objects_dict: dictionary of one type of core-elements (rafcon.statemachine)
        :param model_name: prop_name for the core-element hold by the model, this core-element is covered by the model
        :param model_class: model-class of the elements that should be insert
        :param model_key: if model_list_or_dict is a dictionary the key is the id of the respective element (e.g. 'state_id')
        :return:
        """
        for core_object in core_objects_dict.itervalues():
            found = False
            for model_or_key in model_list_or_dict:
                model = model_or_key if model_key is None else model_list_or_dict[model_or_key]
                if core_object is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list_or_dict.append(model_class(core_object, self))
                else:
                    model_list_or_dict[getattr(core_object, model_key)] = model_class(core_object, self)
                return

    def remove_additional_model(self, model_list_or_dict, core_objects_dict, model_name, model_key):
        for model_or_key in model_list_or_dict:
            model = model_or_key if model_key is None else model_list_or_dict[model_or_key]
            found = False
            for core_object in core_objects_dict.itervalues():
                if core_object is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list_or_dict.remove(model)
                else:
                    del model_list_or_dict[model_or_key]
                return
