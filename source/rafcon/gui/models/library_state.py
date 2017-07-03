# Copyright (C) 2015-2017 DLR
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

from rafcon.core.states.state import State
from rafcon.core.states.library_state import LibraryState

from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.abstract_state import get_state_model_class_for_state

from rafcon.gui.config import global_gui_config
from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryStateModel(AbstractStateModel):
    """This model class manages a LibraryState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param State state: The state to be managed
     """

    state_copy = None

    def __init__(self, state, parent=None, meta=None, load_meta_data=True):
        """Constructor
        """
        # TODO maybe find a different way to load the meta data of ports correctly
        # at the moment the models of state_copy get initialized and the meta data taken from there if not found in
        # state itself
        self.state_copy_initialized = False
        self.meta_data_was_scaled = False
        super(LibraryStateModel, self).__init__(state, parent, meta)
        assert isinstance(state, LibraryState)
        model_class = get_state_model_class_for_state(state.state_copy)
        if model_class is not None:
            # print "load library inner state meta data\n", model_class, state.state_copy.file_system_path
            self.state_copy = model_class(state.state_copy, parent=self)  # TODO think about load_meta_data=False)
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))
        self.state_copy_initialized = True

        self._load_input_data_port_models()
        self._load_output_data_port_models()
        self._load_outcome_models()

        if load_meta_data:
            # print "load library hull state meta data\n", self.state.file_system_path
            # for m in self.input_data_ports[:] + self.output_data_ports[:] + self.outcomes[:]:
            #     print "lib init 1: ", m, m.meta
            # self.load_meta_data()

            if not self.load_meta_data():
                # TODO decide to scale here or still in the editor -> at the moment meta data is missing here
                # print "try to scale accordingly to state copy proportion"
                import rafcon.gui.helpers.meta_data as gui_helper_meta_data
                # gui_helper_meta_data.scale_library_ports_meta_data(self)
            else:
                self.meta_data_was_scaled = True

        # for m in self.input_data_ports[:] + self.output_data_ports[:] + self.outcomes[:]:
        #     print "lib init 2: ", m, m.meta

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, LibraryStateModel):
            return self.state == other.state and self.meta == other.meta
        else:
            return False

    def __copy__(self):
        state_m = AbstractStateModel.__copy__(self)
        state_m.state_copy.copy_meta_data_from_state_m(self.state_copy)
        return state_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def update_hash(self, obj_hash):
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

    def _load_outcome_models(self):
        """Reloads the outcome models directly from the the state"""
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
        
        Content is shown, if the state's meta flag "show_content" is True or if the same flag for a library up in
        the hierarchy (up to a certain configurable level) is True.
        
        :return: Whether the content is to be drawn
        :rtype: bool
        """
        current_hierarchy_depth = 1
        max_hierarchy_depth = global_gui_config.get_config_value("MAX_VISIBLE_LIBRARY_HIERARCHY", 2)
        state_m = self
        while True:
            if isinstance(state_m, LibraryStateModel) and state_m.meta['gui']['show_content']:
                return True
            if current_hierarchy_depth >= max_hierarchy_depth:
                return False
            if state_m.state.is_root_state_of_library:
                current_hierarchy_depth += 1
            if not state_m.parent:
                return False
            state_m = state_m.parent

    def copy_meta_data_from_state_m(self, source_state_m):
        super(LibraryStateModel, self).copy_meta_data_from_state_m(source_state_m)
        self.meta_data_was_scaled = source_state_m.meta_data_was_scaled
