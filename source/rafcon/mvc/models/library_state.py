from os.path import join

from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.models.abstract_state import AbstractStateModel
from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.outcome import OutcomeModel

from rafcon.mvc.models.abstract_state import state_to_state_model

from rafcon.statemachine.singleton import library_manager

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

        model_class = state_to_state_model(state.state_copy)
        if model_class is not None:
            self.state_copy = model_class(state.state_copy, parent=self)
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))

        if load_meta_data:
            self.load_meta_data()

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

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of container states from filesystem

        Recursively loads meta data of child states.
        """
        super(LibraryStateModel, self).load_meta_data(path)
        lib_os_path, _, _ = library_manager.get_os_path_to_library(self.state.library_path,
                                                                   self.state.library_name)
        root_state_path = join(lib_os_path, self.state_copy.state.state_id)
        self.state_copy.load_meta_data(root_state_path)
