from gtkmvc import ModelMT

from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


def state_to_state_model(state):
    """Determines the model required for the given state class

    :param state: Instance of a state (ExecutionState, BarrierConcurrencyState, ...)
    :return: The model class required for holding such a state instance
    """
    from rafcon.mvc.models.state import StateModel
    from rafcon.mvc.models.container_state import ContainerStateModel
    from rafcon.mvc.models.library_state import LibraryStateModel
    if isinstance(state, ContainerState):
        return ContainerStateModel
    elif isinstance(state, LibraryState):
        return LibraryStateModel
    elif isinstance(state, State):
        return StateModel
    else:
        return None


class AbstractStateModel(ModelMT):
    """This is an abstract class serving as base class for state models

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param rafcon.statemachine.states.state.State state: The state to be managed
    :param AbstractStateModel parent: The state to be managed
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state
     """

    is_start = None
    state = None
    outcomes = []
    input_data_ports = []
    output_data_ports = []

    __observables__ = ("state", "input_data_ports", "output_data_ports", "outcomes", "is_start")

    def __init__(self, state, parent=None, meta=None):
        """Constructor
        """
        if type(self) == AbstractStateModel:
            raise NotImplementedError

        ModelMT.__init__(self)
        assert isinstance(state, State)

        self.state = state

        # True if root_state or state is parent start_state_id else False
        self.is_start = state.is_root_state or parent is None or isinstance(parent.state, LibraryState) or \
                        state.state_id == state.parent.start_state_id

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        self.temp = Vividict()

        if isinstance(parent, AbstractStateModel):
            self.parent = parent
        else:
            self.parent = None

        self.register_observer(self)

        self.input_data_ports = []
        self.output_data_ports = []
        self.outcomes = []
        self._load_input_data_port_models()
        self._load_output_data_port_models()
        self._load_outcome_models()

    def get_data_port_model(self, data_port_id):
        """Searches and returns the model of a data port of a given state

        The method searches a port with the given id in the data ports of the given state model. If the state model
        is a container state, not only the input and output data ports are looked at, but also the scoped variables.

        :param data_port_id: The data port id to be searched
        :return: The model of the data port or None if it is not found
        """
        from itertools import chain
        data_ports_m = chain(self.input_data_ports, self.output_data_ports)
        for data_port_m in data_ports_m:
            if data_port_m.data_port.data_port_id == data_port_id:
                return data_port_m
        return None

    def _load_input_data_port_models(self):
        raise NotImplementedError

    def _load_output_data_port_models(self):
        raise NotImplementedError

    def _load_outcome_models(self):
        raise NotImplementedError
