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

from copy import copy, deepcopy
from os.path import join

from rafcon.core.states.state import State
from rafcon.core.states.library_state import LibraryState
from rafcon.core.singleton import library_manager
from rafcon.core.storage.storage import get_storage_id_for_state

from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.data_port import DataPortModel
from rafcon.gui.models.outcome import OutcomeModel
from rafcon.gui.models.abstract_state import get_state_model_class_for_state

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
        super(LibraryStateModel, self).__init__(state, parent, meta)
        assert isinstance(state, LibraryState)
        model_class = get_state_model_class_for_state(state.state_copy)
        if model_class is not None:
            # print "load library inner state meta data\n", model_class, state.state_copy.get_file_system_path()
            self.state_copy = model_class(state.state_copy, parent=self)  # TODO think about load_meta_data=False)
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))

        if load_meta_data:
            # print "load library hull state meta data\n", self.state.get_file_system_path()
            self.load_meta_data()

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

    def _load_input_data_port_models(self):
        """Reloads the input data port models directly from the the state"""
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def _load_output_data_port_models(self):
        """Reloads the output data port models directly from the the state"""
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def _load_outcome_models(self):
        """Reloads the input data port models directly from the the state"""
        self.outcomes = []
        for outcome in self.state.outcomes.itervalues():
            self.outcomes.append(OutcomeModel(outcome, self))
