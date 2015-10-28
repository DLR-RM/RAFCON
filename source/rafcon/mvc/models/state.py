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
        """Constructor
        """
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

        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may relay on the updated models
        if hasattr(info, 'after') and info['after'] and self.state == info['instance']:
            self.update_models(model, prop_name, info)

        # mark the state machine this state belongs to as dirty
        active_flag_responsible = info["method_name"] == "active" or info["method_name"] == "child_execution" or \
                                  info["method_name"] == "state_execution_status"
        if isinstance(model, StateModel) and prop_name == "state" and active_flag_responsible:
            # do not track the active flag when marking the sm dirty
            pass
        else:
            # if the state_execution state is changed the sm must not be marked dirty
            if info["method_name"] != "state_execution_status":
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
        elif "modify_outcome" in info.method_name and self is model or \
                isinstance(info.instance, Outcome) and self is model.parent:
            changed_list = self.outcomes
            cause = "outcome_change"

        if not (cause is None or changed_list is None):
            if hasattr(info, 'before') and info['before']:
                changed_list._notify_method_before(info.instance, cause, (model,), info)
            elif hasattr(info, 'after') and info['after']:
                changed_list._notify_method_after(info.instance, cause, None, (model,), info)

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

    def add_missing_model(self, model_list, data_list, model_name, model_class, model_key):
        for data in data_list.itervalues():
            found = False
            for model_item in model_list:
                model = model_item if model_key is None else model_list[model_item]
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.append(model_class(data, self))
                else:
                    model_list[getattr(data, model_key)] = model_class(data, self)
                return

    def remove_additional_model(self, model_list, data_list, model_name, model_key):
        for model_item in model_list:
            model = model_item if model_key is None else model_list[model_item]
            found = False
            for data in data_list.itervalues():
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.remove(model)
                else:
                    del model_list[model_item]
                return
