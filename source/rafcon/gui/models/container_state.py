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
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from future.utils import string_types

from copy import deepcopy

from gtkmvc3.model_mt import ModelMT

from rafcon.core.states.container_state import ContainerState
from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.abstract_state import get_state_model_class_for_state
from rafcon.gui.models.data_flow import DataFlowModel, StateElementModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.models.state import StateModel
from rafcon.gui.models.transition import TransitionModel

from rafcon.gui.utils.notification_overview import NotificationOverview
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

    def __init__(self, container_state, parent=None, meta=None, load_meta_data=True, expected_future_models=None):
        """Constructor
        """
        assert isinstance(container_state, ContainerState)
        super(ContainerStateModel, self).__init__(container_state, parent, meta, load_meta_data, expected_future_models)

        self._load_child_state_models(load_meta_data)

        self._load_transition_models()
        self._load_data_flow_models()

        self.update_child_is_start()

        if load_meta_data:
            self.load_meta_data()

        # this class is an observer of its own properties:
        self.register_observer(self)

    def _load_port_models(self):
        super(ContainerStateModel, self)._load_port_models()
        self._load_scoped_variable_models()

    def _load_child_state_models(self, load_meta_data):
        """Adds models for each child state of the state

        :param bool load_meta_data: Whether to load the meta data of the child state
        """
        self.states = {}
        # Create model for each child class
        child_states = self.state.states
        for child_state in child_states.values():
            # Create hierarchy
            model_class = get_state_model_class_for_state(child_state)
            if model_class is not None:
                self._add_model(self.states, child_state, model_class, child_state.state_id, load_meta_data)
            else:
                logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(child_state)))

    def _load_scoped_variable_models(self):
        """ Adds models for each scoped variable of the state """
        self.scoped_variables = []
        for scoped_variable in self.state.scoped_variables.values():
            self._add_model(self.scoped_variables, scoped_variable, ScopedVariableModel)

    def _load_data_flow_models(self):
        """ Adds models for each data flow of the state """
        self.data_flows = []
        for data_flow in self.state.data_flows.values():
            self._add_model(self.data_flows, data_flow, DataFlowModel)

    def _load_transition_models(self):
        """ Adds models for each transition of the state """
        self.transitions = []
        for transition in self.state.transitions.values():
            self._add_model(self.transitions, transition, TransitionModel)

    def __contains__(self, item):
        """Checks whether `item` is an element of the container state model

        Following child items are checked: outcomes, input data ports, output data ports, scoped variables, states,
        transitions, data flows

        :param item: :class:`StateModel` or :class:`StateElementModel`
        :return: Whether item is a direct child of this state
        :rtype: bool
        """
        if not isinstance(item, (StateModel, StateElementModel)):
            return False
        return super(ContainerStateModel, self).__contains__(item) or item in self.states.values() \
               or item in self.transitions or item in self.data_flows \
               or item in self.scoped_variables

    def prepare_destruction(self, recursive=True):
        """Prepares the model for destruction

        Recursively un-registers all observers and removes references to child models. Extends the destroy method of
        the base class by child elements of a container state.
        """
        # logger.verbose("Prepare destruction container state ...")
        if recursive:
            for scoped_variable in self.scoped_variables:
                scoped_variable.prepare_destruction()
            for connection in self.transitions[:] + self.data_flows[:]:
                connection.prepare_destruction()
            for state in self.states.values():
                state.prepare_destruction(recursive)
        del self.scoped_variables[:]
        del self.transitions[:]
        del self.data_flows[:]
        self.states.clear()
        self.scoped_variables = None
        self.transitions = None
        self.data_flows = None
        self.states = None
        super(ContainerStateModel, self).prepare_destruction(recursive)

    def update_hash(self, obj_hash):
        super(ContainerStateModel, self).update_hash(obj_hash)
        for state_element in sorted(self.states.values()) + sorted(self.transitions[:] + self.data_flows[:] + \
                                                                   self.scoped_variables[:]):
            self.update_hash_from_dict(obj_hash, state_element)

    def update_meta_data_hash(self, obj_hash):
        super(ContainerStateModel, self).update_meta_data_hash(obj_hash)
        for state_element in sorted(self.states.values()) + sorted(self.transitions[:] + self.data_flows[:] + \
                                                                   self.scoped_variables[:]):
            state_element.update_meta_data_hash(obj_hash)

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
        overview = NotificationOverview(info)
        # if info.method_name == 'change_state_type':  # Handled in method 'change_state_type'
        #     return

        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may relay on the updated models
        if not self.child_model_changed(overview):
            if overview.operation_finished():
                self.update_child_models(model, prop_name, info)
                # if there is and exception set is_about_to_be_destroyed_recursively flag to False again
                if info.method_name in ["remove_state"] and isinstance(info.result, Exception):
                    state_id = info.kwargs['state_id'] if 'state_id' in info.kwargs else info.args[1]
                    self.states[state_id].is_about_to_be_destroyed_recursively = False
            elif overview.operation_started():
                # while before notification mark all states which get destroyed recursively
                if info.method_name in ["remove_state"] and \
                        info.kwargs.get('destroy', True) and info.kwargs.get('recursive', True):
                    state_id = info.kwargs['state_id'] if 'state_id' in info.kwargs else info.args[1]
                    self.states[state_id].is_about_to_be_destroyed_recursively = True

        # Finally call the method of the base class, to forward changes in ports and outcomes
        super(ContainerStateModel, self).model_changed(model, prop_name, info)

    def get_cause_and_affected_model_list(self, model):
        cause, changed_list = super(ContainerStateModel, self).get_cause_and_affected_model_list(model)

        if cause is None:
            # If the change happened in a child state, notify the list of all child states
            if (isinstance(model, AbstractStateModel) and model is not self) or (  # The state was changed directly
                    not isinstance(model, AbstractStateModel) and model.parent is not self):  # One of the member models was changed
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

        return cause, changed_list


    def update_child_is_start(self):
        """ Updates the `is_child` property of its child states """
        for state_id, state_m in self.states.items():
            state_m.update_is_start()

    def _get_model_info(self, model, info=None):
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
            model_class = None
            # TODO this if cause is not working if keys are used for arguments
            # if len(info.args) < 2:
            #     print("XXXX", info)
            if not isinstance(info.args[1], (string_types, dict)) and info.args[1] is not None:
                model_class = get_state_model_class_for_state(info.args[1])
            model_key = "state_id"
        return model_list, data_list, model_name, model_class, model_key

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
            self.update_child_is_start()

        if info.method_name in ["add_transition", "remove_transition", "transitions"]:
            (model_list, data_list, model_name, model_class, model_key) = self._get_model_info("transition")
        elif info.method_name in ["add_data_flow", "remove_data_flow", "data_flows"]:
            (model_list, data_list, model_name, model_class, model_key) = self._get_model_info("data_flow")
        elif info.method_name in ["add_state", "remove_state", "states"]:
            (model_list, data_list, model_name, model_class, model_key) = self._get_model_info("state", info)
        elif info.method_name in ["add_scoped_variable", "remove_scoped_variable", "scoped_variables"]:
            (model_list, data_list, model_name, model_class, model_key) = self._get_model_info("scoped_variable")
        else:
            return

        if isinstance(info.result, Exception):
            # Do nothing if the observed function raised an exception
            pass
        elif "add" in info.method_name:
            self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
        elif "remove" in info.method_name:
            destroy = info.kwargs.get('destroy', True)
            recursive = info.kwargs.get('recursive', True)
            self.remove_specific_model(model_list, info.result, model_key, recursive, destroy)
        elif info.method_name in ["transitions", "data_flows", "states", "scoped_variables"]:
            self.re_initiate_model_list(model_list, data_list, model_name, model_class, model_key)

    def insert_meta_data_from_models_dict(self, source_models_dict, notify_logger_method):
        # TODO D-Clean this up and integrate proper into group/ungroup functionality
        if 'states' in source_models_dict:
            for child_state_id, old_state_m in source_models_dict['states'].items():
                new_state_m = self.states[child_state_id]
                if new_state_m is None:
                    raise RuntimeError("State model to set meta data could not be found"
                                       " -> {0}".format(old_state_m.state))
                if new_state_m is old_state_m:
                    logger.verbose("Old {0} is new model {1}".format(old_state_m, new_state_m))
                else:
                    new_state_m.meta = old_state_m.meta
                    notify_logger_method("Should only happen in ungroup - new model {0}".format(new_state_m))
        if 'scoped_variables' in source_models_dict:
            for sv_dp_id, old_sv_m in source_models_dict['scoped_variables'].items():
                new_sv_m = self.get_scoped_variable_m(sv_dp_id)
                if new_sv_m is None:
                    raise RuntimeError("Scoped variable model to set meta data could not be found"
                                       " -> {0}".format(old_sv_m.scoped_variable))
                if new_sv_m is old_sv_m:
                    logger.verbose("Old {0} is new model {1}".format(old_sv_m, new_sv_m))
                else:
                    new_sv_m.meta = old_sv_m.meta
                    notify_logger_method("Should only happen in ungroup - new model {0}".format(new_sv_m))

        if 'transitions' in source_models_dict:
            for t_id, old_t_m in list(source_models_dict['transitions'].items()):
                new_t_m = self.get_transition_m(t_id)
                if new_t_m is None:
                    raise RuntimeError("transition model to set meta data could not be found"
                                       " -> {0}".format(old_t_m.transition))
                if new_t_m is old_t_m:
                    logger.verbose("Old {0} is new model {1}".format(old_t_m, new_t_m))
                else:
                    new_t_m.meta = old_t_m.meta
                    notify_logger_method("Should only happen in ungroup - new model {0}".format(new_t_m))
        if 'data_flows' in source_models_dict:
            for df_id, old_df_m in source_models_dict['data_flows'].items():
                new_df_m = self.get_data_flow_m(df_id)
                if new_df_m is None:
                    raise RuntimeError("data flow model to set meta data could not be found"
                                       " -> {0}".format(old_df_m.data_flow))
                if new_df_m is old_df_m:
                    logger.verbose("Old model {0} is new model {1}".format(old_df_m, new_df_m))
                else:
                    new_df_m.meta = old_df_m.meta
                    notify_logger_method("Should only happen in ungroup - new model {0}".format(new_df_m))

    @ModelMT.observe("state", after=True, before=True)
    def substitute_state(self, model, prop_name, info):
        if info.method_name != 'substitute_state':
            return

    @ModelMT.observe("state", after=True, before=True)
    def group_states(self, model, prop_name, info):
        if info.method_name != 'group_selected_states_and_scoped_variables':
            return

    @ModelMT.observe("state", after=True, before=True)
    def ungroup_state(self, model, prop_name, info):
        if info.method_name != 'ungroup_state':
            return

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

    def store_meta_data(self, copy_path=None):
        """Store meta data of container states to the filesystem

        Recursively stores meta data of child states. For further insides read the description of also called respective
        super class method.

        :param str copy_path: Optional copy path if meta data is not stored to the file system path of state machine
        """
        super(ContainerStateModel, self).store_meta_data(copy_path)
        for state_key, state in self.states.items():
            state.store_meta_data(copy_path)

    def copy_meta_data_from_state_m(self, source_state_m):
        """Dismiss current meta data and copy meta data from given state model

        In addition to the state model method, also the meta data of container states is copied. Then, the meta data
        of child states are recursively copied.

        :param source_state_m: State model to load the meta data from
        """
        for scoped_variable_m in self.scoped_variables:
            source_scoped_variable_m = source_state_m.get_scoped_variable_m(
                scoped_variable_m.scoped_variable.data_port_id)
            scoped_variable_m.meta = deepcopy(source_scoped_variable_m.meta)

        for transition_m in self.transitions:
            source_transition_m = source_state_m.get_transition_m(transition_m.transition.transition_id)
            transition_m.meta = deepcopy(source_transition_m.meta)

        for data_flow_m in self.data_flows:
            source_data_flow_m = source_state_m.get_data_flow_m(data_flow_m.data_flow.data_flow_id)
            data_flow_m.meta = deepcopy(source_data_flow_m.meta)

        for state_key, state in self.states.items():
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
