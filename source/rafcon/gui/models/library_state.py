# Copyright (C) 2015-2018 DLR
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

from copy import deepcopy

from gtkmvc3.model_mt import ModelMT

from rafcon.core.states.state import State
from rafcon.core.states.library_state import LibraryState

from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.abstract_state import get_state_model_class_for_state

from rafcon.gui.utils.notification_overview import NotificationOverview
from rafcon.gui.config import global_gui_config
from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryStateModel(AbstractStateModel):
    """This model class manages a LibraryState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param rafcon.core.states.library_state.LibraryState state: The state to be managed
     """

    state_copy = None

    def __init__(self, state, parent=None, meta=None, load_meta_data=True):
        assert isinstance(state, LibraryState)

        self.state_copy_initialized = False
        self.meta_data_was_scaled = False
        super(LibraryStateModel, self).__init__(state, parent, meta)

        self.recursive_generate_models(load_meta_data)

    def recursive_generate_models(self, load_meta_data):
        # TODO maybe find a different way to load the meta data of ports correctly
        # at the moment the models of state_copy get initialized and the meta data taken from there if not found in
        # state itself

        # regulate depth of library model generation to reduce resource consumption
        current_hierarchy_depth = self.state.library_hierarchy_depth
        min_temp_depth = 2
        max_hierarchy_depth = global_gui_config.get_config_value("MAX_VISIBLE_LIBRARY_HIERARCHY", min_temp_depth)
        if max_hierarchy_depth < min_temp_depth:
            logger.verbose("Increase MAX_VISIBLE_LIBRARY_HIERARCHY to {0} for state {1}.".format(min_temp_depth, self))
            max_hierarchy_depth = min_temp_depth
        no_fully_rec_lib_model = global_gui_config.get_config_value("NO_FULLY_RECURSIVE_LIBRARY_MODEL", False)
        recursive_model_generation = not (current_hierarchy_depth > max_hierarchy_depth) or not no_fully_rec_lib_model
        if recursive_model_generation:
            # logger.debug("initialize state copy {0}".format(self))
            self.initiate_library_root_state_model()
        else:
            # logger.verbose("Do not initialize state copy {0}".format(self))
            pass

        self._load_port_models()

        if load_meta_data:
            if not self.load_meta_data():
                # TODO decide to scale here or still in the editor -> at the moment meta data is missing here
                import rafcon.gui.helpers.meta_data as gui_helper_meta_data
                # gui_helper_meta_data.scale_library_ports_meta_data(self)
            else:
                self.meta_data_was_scaled = True

    def initiate_library_root_state_model(self):
        model_class = get_state_model_class_for_state(self.state.state_copy)
        if model_class is not None:
            self.state_copy = model_class(self.state.state_copy, parent=self)
            self.state_copy_initialized = True
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(self.state)))

    def enforce_generation_of_state_copy_model(self):
        """This enforce a load of state copy model without considering meta data"""
        self.initiate_library_root_state_model()
        self._load_port_models()

    def prepare_destruction(self, recursive=True):
        """Prepares the model for destruction

        Recursively un-registers all observers and removes references to child models
        """
        self.destruction_signal.emit()
        try:
            self.unregister_observer(self)
        except KeyError:  # Might happen if the observer was already unregistered
            pass
        if recursive:

            if self.state_copy:
                self.state_copy.prepare_destruction(recursive)
                self.state_copy = None
            else:
                if self.state_copy_initialized:
                    logger.verbose("Multiple calls of prepare destruction for {0}".format(self))

            # The next lines are commented because not needed and create problems if used why it is an open to-do
            # for port in self.input_data_ports[:] + self.output_data_ports[:] + self.outcomes[:]:
            #     if port.core_element is not None:
            #         # TODO setting data ports None in a Library state cause gtkmvc3 attribute getter problems why?
            #         port.prepare_destruction()

        del self.input_data_ports[:]
        del self.output_data_ports[:]
        del self.outcomes[:]
        self.state = None

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, LibraryStateModel):
            return self.state == other.state and self.meta == other.meta
        else:
            return False

    def __hash__(self):
        return id(self)

    def __copy__(self):
        state_m = AbstractStateModel.__copy__(self)
        state_m.state_copy.copy_meta_data_from_state_m(self.state_copy)
        return state_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def update_hash(self, obj_hash):
        super(LibraryStateModel, self).update_hash(obj_hash)
        self.update_hash_from_dict(obj_hash, self.state_copy)

    def _load_input_data_port_models(self):
        """Reloads the input data port models directly from the the state"""
        if not self.state_copy_initialized:
            return
        self.input_data_ports = []
        for input_data_port_m in self.state_copy.input_data_ports:
            new_ip_m = deepcopy(input_data_port_m)
            new_ip_m.parent = self
            new_ip_m.data_port = input_data_port_m.data_port
            self.input_data_ports.append(new_ip_m)

    def _load_output_data_port_models(self):
        """Reloads the output data port models directly from the the state"""
        if not self.state_copy_initialized:
            return
        self.output_data_ports = []
        for output_data_port_m in self.state_copy.output_data_ports:
            new_op_m = deepcopy(output_data_port_m)
            new_op_m.parent = self
            new_op_m.data_port = output_data_port_m.data_port
            self.output_data_ports.append(new_op_m)

    def _load_income_model(self):
        """Reloads the income model directly from the state"""
        if not self.state_copy_initialized:
            return
        self.income = None
        income_m = deepcopy(self.state_copy.income)
        income_m.parent = self
        income_m.income = income_m.income
        self.income = income_m

    def _load_outcome_models(self):
        """Reloads the outcome models directly from the state"""
        if not self.state_copy_initialized:
            return
        self.outcomes = []
        for outcome_m in self.state_copy.outcomes:
            new_oc_m = deepcopy(outcome_m)
            new_oc_m.parent = self
            new_oc_m.outcome = outcome_m.outcome
            self.outcomes.append(new_oc_m)

    def show_content(self):
        """Check if content of library is to be shown
        
        Content is shown, if the uppermost state's meta flag "show_content" is True and the library hierarchy depth
        (up to MAX_VISIBLE_LIBRARY_HIERARCHY level) is not to high.
        
        :return: Whether the content is to be shown
        :rtype: bool
        """
        current_hierarchy_depth = self.state.library_hierarchy_depth
        max_hierarchy_depth = global_gui_config.get_config_value("MAX_VISIBLE_LIBRARY_HIERARCHY", 2)
        if current_hierarchy_depth >= max_hierarchy_depth:
            return False
        if current_hierarchy_depth > 1:
            uppermost_lib_state = self.state.get_uppermost_library_root_state().parent
            uppermost_lib_state_m = self.get_state_machine_m().get_state_model_by_path(uppermost_lib_state.get_path())
        else:
            uppermost_lib_state_m = self
        uppermost_lib_meta = uppermost_lib_state_m.meta
        return False if 'show_content' not in uppermost_lib_meta['gui'] else uppermost_lib_meta['gui']['show_content']

    def copy_meta_data_from_state_m(self, source_state_m):
        assert isinstance(source_state_m, LibraryStateModel)
        super(LibraryStateModel, self).copy_meta_data_from_state_m(source_state_m)
        self.meta_data_was_scaled = source_state_m.meta_data_was_scaled

    @property
    def is_about_to_be_destroyed_recursively(self):
        return self._is_about_to_be_destroyed_recursively

    @is_about_to_be_destroyed_recursively.setter
    def is_about_to_be_destroyed_recursively(self, value):
        if not isinstance(value, bool):
            raise TypeError("The is_about_to_be_destroyed_recursively property has to be boolean.")
        self._is_about_to_be_destroyed_recursively = value
        if self.state_copy:
            self.state_copy.is_about_to_be_destroyed_recursively = value
            
    def _parse_for_element_meta_data(self, meta_data):
        if not self.state_copy_initialized:
            return 
        super(LibraryStateModel, self)._parse_for_element_meta_data(meta_data)

    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        overview = NotificationOverview(info)
        if overview.operation_finished() and self.child_model_changed(overview):
            # The only operations allowed are setting the parent of the state_copy and
            # changing the state_execution_status
            if not (info["instance"] == self.state_copy.state and info["method_name"] == "parent" or
                    info["method_name"] == "state_execution_status"):
                if not self.is_about_to_be_destroyed_recursively:
                    logger.warning("You have modified core property of an inner state of a library state.")
        super(LibraryStateModel, self).model_changed(model, prop_name, info)
