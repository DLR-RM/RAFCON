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
            self.state_copy = model_class(state.state_copy, parent=self)
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))

        if load_meta_data:
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

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of container states from filesystem

        Recursively loads meta data of child states.
        """
        super(LibraryStateModel, self).load_meta_data(path)
        lib_os_path, _, _ = library_manager.get_os_path_to_library(self.state.library_path,
                                                                   self.state.library_name)
        if self.state.get_sm_for_state():
            # TODO: needs to be tested
            if self.state.get_sm_for_state().supports_saving_state_names:
                root_state_path = join(lib_os_path, get_storage_id_for_state(self.state_copy.state))
                # print "LibraryStateModel: ", root_state_path
                import os
                if not os.path.exists(root_state_path):
                    root_state_path = join(lib_os_path, get_storage_id_for_state(self.state_copy.state, old_delimiter=True))
                # was:
                # root_state_path = join(lib_os_path, get_storage_id_for_state(self.state_copy.state))
            else:
                root_state_path = join(lib_os_path, self.state_copy.state.state_id)
        else:
            root_state_path = join(lib_os_path, self.state_copy.state.state_id)
        self.state_copy.load_meta_data(root_state_path)
