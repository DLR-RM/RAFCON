# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.model_mt import ModelMT
from builtins import range

from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.data_port import DataPortModel
from rafcon.gui.models.logical_port import IncomeModel, OutcomeModel
from rafcon.gui.utils.notification_overview import NotificationOverview
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.utils import log, type_helpers

logger = log.get_logger(__name__)


class StateModel(AbstractStateModel):
    """This model class manages a State, for the moment only ExecutionStates

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param rafcon.core.states.state.State state: The state to be managed
    :param AbstractStateModel parent: The state to be managed
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state
    :param __buildIn__.set expected_future_models: Existing models for new core elements
    """

    expected_future_models = None

    def __init__(self, state, parent=None, meta=None, load_meta_data=True, expected_future_models=None):
        """Constructor"""
        self.expected_future_models = set() if expected_future_models is None else set(expected_future_models)
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
        overview = NotificationOverview(info)

        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may rely on the updated models
        if overview.operation_finished() and not self.child_model_changed(overview):
            self.update_models(model, prop_name, info)

        cause, changed_list = self.get_cause_and_affected_model_list(model)

        if not (cause is None or cause is "income_change" or changed_list is None):
            if overview.operation_started():
                changed_list._notify_method_before(self.state, cause, (self.state,), info)
            else:
                changed_list._notify_method_after(self.state, cause, None, (self.state,), info)

        # Notifies parent state
        super(StateModel, self).model_changed(model, prop_name, info)

    def get_cause_and_affected_model_list(self, model):
        changed_list = None
        cause = None

        if isinstance(model, DataPortModel) and model.parent is self:
            if isinstance(model.core_element, InputDataPort):
                changed_list = self.input_data_ports
                cause = "input_data_port_change"
            elif isinstance(model.core_element, OutputDataPort):
                changed_list = self.output_data_ports
                cause = "output_data_port_change"

        elif isinstance(model, IncomeModel) and self is model.parent:
            changed_list = self.income
            cause = "income_change"

        elif isinstance(model, OutcomeModel) and self is model.parent:
            changed_list = self.outcomes
            cause = "outcome_change"

        return cause, changed_list

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
        elif model == "income":
            model_list = self.income
            data_list = self.state.income
            model_name = "income"
            model_class = IncomeModel
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

        if info.method_name in ["add_input_data_port", "remove_input_data_port", "input_data_ports"]:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("input_data_port")
        elif info.method_name in ["add_output_data_port", "remove_output_data_port", "output_data_ports"]:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("output_data_port")
        elif info.method_name in ["add_income", "remove_income", "income"]:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("income")
        elif info.method_name in ["add_outcome", "remove_outcome", "outcomes"]:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("outcome")
        else:
            return

        if "add" in info.method_name:
            self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
        elif "remove" in info.method_name:
            destroy = info.kwargs.get('destroy', True)
            self.remove_specific_model(model_list, info.result, model_key, destroy)
        elif info.method_name in ["input_data_ports", "output_data_ports", "income", "outcomes"]:
            self.re_initiate_model_list(model_list, data_list, model_name, model_class, model_key)

    def _load_input_data_port_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.values():
            self._add_model(self.input_data_ports, input_data_port, DataPortModel)

    def _load_output_data_port_models(self):
        """Reloads the output data port models directly from the the state
        """
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.values():
            self._add_model(self.output_data_ports, output_data_port, DataPortModel)

    def _load_income_model(self):
        """ Create income model from core income """
        self._add_model(self.income, self.state.income, IncomeModel)

    def _load_outcome_models(self):
        """ Create outcome models from core outcomes """
        self.outcomes = []
        for outcome in self.state.outcomes.values():
            self._add_model(self.outcomes, outcome, OutcomeModel)

    def re_initiate_model_list(self, model_list_or_dict, core_objects_dict, model_name, model_class, model_key):
        """Recreate model list

        The method re-initiate a handed list or dictionary of models with the new dictionary of core-objects.

        :param model_list_or_dict: could be a list or dictionary of one model type
        :param core_objects_dict: new dictionary of one type of core-elements (rafcon.core)
        :param model_name: prop_name for the core-element hold by the model, this core-element is covered by the model
        :param model_class: model-class of the elements that should be insert
        :param model_key: if model_list_or_dict is a dictionary the key is the id of the respective element
                          (e.g. 'state_id')
        :return:
        """
        if model_name == "income":
            if self.income.income != self.state.income:
                self._add_model(self.income, self.state.income, IncomeModel)
            return

        for _ in range(len(model_list_or_dict)):
            self.remove_additional_model(model_list_or_dict, core_objects_dict, model_name, model_key)

        if core_objects_dict:
            for _ in core_objects_dict:
                self.add_missing_model(model_list_or_dict, core_objects_dict, model_name, model_class, model_key)

    def _add_model(self, model_list_or_dict, core_element, model_class, model_key=None, load_meta_data=True):
        """Adds one model for a given core element.

        The method will add a model for a given core object and checks if there is a corresponding model object in the
        future expected model list. The method does not check if an object with corresponding model has already been
        inserted.

        :param model_list_or_dict:  could be a list or dictionary of one model type
        :param core_element: the core element to a model for, can be state or state element
        :param model_class: model-class of the elements that should be insert
        :param model_key: if model_list_or_dict is a dictionary the key is the id of the respective element
                          (e.g. 'state_id')
        :param load_meta_data: specific argument for loading meta data
        :return:
        """

        found_model = self._get_future_expected_model(core_element)
        if found_model:
            found_model.parent = self

        if model_class is IncomeModel:
            self.income = found_model if found_model else IncomeModel(core_element, self)
            return

        if model_key is None:
            model_list_or_dict.append(found_model if found_model else model_class(core_element, self))
        else:
            model_list_or_dict[model_key] = found_model if found_model else model_class(core_element, self,
                                                                                        load_meta_data=load_meta_data)

    def add_missing_model(self, model_list_or_dict, core_elements_dict, model_name, model_class, model_key):
        """Adds one missing model

        The method will search for the first core-object out of core_object_dict
        not represented in the list or dict of models handed by model_list_or_dict, adds it and returns without continue
        to search for more objects which maybe are missing in model_list_or_dict with respect to the
        core_object_dict.

        :param model_list_or_dict: could be a list or dictionary of one model type
        :param core_elements_dict: dictionary of one type of core-elements (rafcon.core)
        :param model_name: prop_name for the core-element hold by the model, this core-element is covered by the model
        :param model_class: model-class of the elements that should be insert
        :param model_key: if model_list_or_dict is a dictionary the key is the id of the respective element
                          (e.g. 'state_id')
        :return: True, is a new model was added, False else
        :rtype: bool
        """
        def core_element_has_model(core_object):
            for model_or_key in model_list_or_dict:
                model = model_or_key if model_key is None else model_list_or_dict[model_or_key]
                if core_object is getattr(model, model_name):
                    return True
            return False

        if model_name == "income":
            self._add_model(self.income, self.state.income, IncomeModel)
            return
            
        for core_element in core_elements_dict.values():
            if core_element_has_model(core_element):
                continue

            # get expected model and connect it to self or create a new model
            new_model = self._get_future_expected_model(core_element)
            if new_model:
                new_model.parent = self
            else:
                if type_helpers.type_inherits_of_type(model_class, StateModel):
                    new_model = model_class(core_element, self, expected_future_models=self.expected_future_models)
                    self.expected_future_models = new_model.expected_future_models  # update reused models
                    new_model.expected_future_models = set()  # clean the field because should not be used further
                else:
                    new_model = model_class(core_element, self)

            # insert new model into list or dict
            if model_key is None:
                model_list_or_dict.append(new_model)
            else:
                model_list_or_dict[getattr(core_element, model_key)] = new_model
            return True
        return False

    def remove_specific_model(self, model_list_or_dict, core_element, model_key=None, recursive=True, destroy=True):
        if isinstance(model_list_or_dict, IncomeModel):
            model = model_list_or_dict
            if destroy:
                model.prepare_destruction()
            return

        if model_key is None:
            model_list = model_list_or_dict
            for model in model_list[:]:
                if model.core_element is core_element:
                    if destroy:
                        model.prepare_destruction()
                    model_list.remove(model)
                    return
        else:
            model_dict = model_list_or_dict
            for model_id, model in list(model_dict.items()):
                if model.core_element is core_element:
                    if destroy:
                        model.prepare_destruction(recursive)
                    del model_dict[model_id]
                    return

    def remove_additional_model(self, model_list_or_dict, core_objects_dict, model_name, model_key, destroy=True):
        """Remove one unnecessary model

        The method will search for the first model-object out of
        model_list_or_dict that represents no core-object in the dictionary of core-objects handed by core_objects_dict,
        remove it and return without continue to search for more model-objects which maybe are unnecessary, too.

        :param model_list_or_dict: could be a list or dictionary of one model type
        :param core_objects_dict: dictionary of one type of core-elements (rafcon.core)
        :param model_name: prop_name for the core-element hold by the model, this core-element is covered by the model
        :param model_key: if model_list_or_dict is a dictionary the key is the id of the respective element
                          (e.g. 'state_id')
        :return:
        """
        if model_name == "income":
            self.income.prepare_destruction()
            self.income = None
            return

        if model_key is None:
            model_list = model_list_or_dict
            for model in model_list[:]:
                for core_object in core_objects_dict.values():
                    if core_object is getattr(model, model_name):
                        break
                else:  # core object not found
                    if destroy:
                        model.prepare_destruction()
                    model_list.remove(model)
                    return
        else:
            model_dict = model_list_or_dict
            for model_id, model in list(model_dict.items()):
                for core_object in core_objects_dict.values():
                    if core_object is getattr(model, model_name):
                        break
                else:  # core object not found
                    if destroy:
                        model.prepare_destruction()
                    del model_dict[model_id]
                    return

    def _get_future_expected_model(self, core_element):
        """Hand model for an core element from expected model list and remove the model from this list"""
        for model in self.expected_future_models:
            if model.core_element is core_element:
                # print("expected_future_model found -> remove model:", model, [model], id(model))
                self.expected_future_models.remove(model)
                return model
        return None
