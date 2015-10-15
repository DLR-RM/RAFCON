import os.path
from copy import deepcopy
from collections import namedtuple

from gtkmvc import ModelMT, Signal

from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.singleton import global_storage

from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.library_state import LibraryState

from rafcon.utils.constants import GLOBAL_STORAGE_BASE_PATH
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

MetaSignalMsg = namedtuple('MetaSignalMsg', ['origin', 'change', 'affects_children', 'notification'])

Notification = namedtuple('Notification', ['model', 'prop_name', 'info'])


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
    meta_signal = Signal()

    __observables__ = ("state", "input_data_ports", "output_data_ports", "outcomes", "is_start", "meta_signal")

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
        self.meta_signal = Signal()

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

    def get_input_data_port_m(self, data_port_id):
        """Returns the input data port model for the given data port id

        :param data_port_id: The data port id to search for
        :return: The model of the data port with the given id
        """
        for data_port_m in self.input_data_ports:
            if data_port_m.data_port.data_port_id == data_port_id:
                return data_port_m
        return None

    def get_output_data_port_m(self, data_port_id):
        """Returns the output data port model for the given data port id

        :param data_port_id: The data port id to search for
        :return: The model of the data port with the given id
        """
        for data_port_m in self.output_data_ports:
            if data_port_m.data_port.data_port_id == data_port_id:
                return data_port_m
        return None

    def get_data_port_m(self, data_port_id):
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

    def get_outcome_m(self, outcome_id):
        """Returns the outcome model for the given outcome id

        :param outcome_id: The outcome id to search for
        :return: The model of the outcome with the given id
        """
        for outcome_m in self.outcomes:
            if outcome_m.outcome.outcome_id == outcome_id:
                return outcome_m
        return False

    def _load_input_data_port_models(self):
        raise NotImplementedError

    def _load_output_data_port_models(self):
        raise NotImplementedError

    def _load_outcome_models(self):
        raise NotImplementedError

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, prop_name, info):
        """This method notifies parent state about changes made to the state
        """

        # Notify the parent state about the change (this causes a recursive call up to the root state)
        if self.parent is not None:
            self.parent.model_changed(model, prop_name, info)

    @ModelMT.observe("meta_signal", signal=True)
    def meta_changed(self, model, prop_name, info):
        """This method notifies the parent state about changes made to the meta data
        """
        # print "meta", self.state.name, self.state.state_id, model, prop_name, info
        if self.parent is not None:
            self.parent.meta_changed(model, prop_name, info)
        elif not isinstance(info.arg, dict):
            self.meta_signal.emit({'model': model, 'prop_name': prop_name, 'info': info})

    @staticmethod
    def overwrite_editor_meta(meta):
        """
        This function is for backward compatibility for state machines that still uses the "editor" key in their meta
        :param meta:
        :return:
        """
        if "editor" in meta['gui']:
            if "editor_opengl" not in meta['gui']:
                meta['gui']['editor_opengl'] = deepcopy(meta['gui']['editor'])
            del meta['gui']['editor']

    def _mark_state_machine_as_dirty(self):
        state_machine = self.state.get_sm_for_state()
        if state_machine:
            state_machine_id = self.state.get_sm_for_state().state_machine_id
            if state_machine_id is not None:
                from rafcon.statemachine.singleton import state_machine_manager
                state_machine_manager.state_machines[state_machine_id].marked_dirty = True

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of state model from the file system

        The meta data of the state model is loaded from the file system and stored in the meta property of the model.
        Existing meta data is removed. Also the meta data of all state elements (data ports, outcomes,
        etc) are loaded, as it is stored in the same file as the meta data of the state.
        """
        if not path:
            path = self.state.get_file_system_path()
        meta_path = os.path.join(path, StateMachineStorage.GRAPHICS_FILE)
        if os.path.exists(meta_path):
            tmp_meta = global_storage.storage_utils.load_dict_from_yaml(meta_path)
            # For backwards compatibility: move all meta data from editor to editor_opengl
            self.overwrite_editor_meta(tmp_meta)
            self._parse_for_element_meta_data(tmp_meta)
            # assign the meta data to the state
            self.meta = tmp_meta
            self.meta_signal.emit("load_meta_data")
        # Print info only if the state has a location different from the tmp directory
        elif meta_path[0:len(GLOBAL_STORAGE_BASE_PATH)] != GLOBAL_STORAGE_BASE_PATH:
            logger.info("State '{0}' has no meta data. It will now be generated automatically.".format(self.state.name))

    def store_meta_data(self):
        """Save meta data of state model to the file system

        This method generates a dictionary of the meta data of the state together with the meta data of all state
        elements (data ports, outcomes, etc.) and stores it on the filesystem.
        """
        meta_path = os.path.join(self.state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE)
        meta_data = deepcopy(self.meta)
        self._generate_element_meta_data(meta_data)
        global_storage.storage_utils.write_dict_to_yaml(meta_data, meta_path)

    def copy_meta_data_from_state_m(self, source_state_m):
        """Dismiss current meta data and copy meta data from given state model

        The meta data of the given state model is used as meta data for this state. Also the meta data of all state
        elements (data ports, outcomes, etc.) is overwritten with the meta data of the elements of the given state.

        :param source_state_m: State model to load the meta data from
        """

        self.meta = deepcopy(source_state_m.meta)

        for input_data_port_m in self.input_data_ports:
            source_data_port_m = source_state_m.get_input_data_port_m(input_data_port_m.data_port.data_port_id)
            input_data_port_m.meta = deepcopy(source_data_port_m.meta)
        for output_data_port_m in self.output_data_ports:
            source_data_port_m = source_state_m.get_output_data_port_m(output_data_port_m.data_port.data_port_id)
            output_data_port_m.meta = deepcopy(source_data_port_m.meta)
        for outcome_m in self.outcomes:
            source_outcome_m = source_state_m.get_outcome_m(outcome_m.outcome.outcome_id)
            outcome_m.meta = deepcopy(source_outcome_m.meta)

        self.meta_signal.emit("copy_meta_data_from_state_m")

    def _parse_for_element_meta_data(self, meta_data):
        """Load meta data for state elements

        The meta data of the state meta data file also contains the meta data for state elements (data ports,
        outcomes, etc). This method parses the loaded meta data for each state element model. The meta data of the
        elements is removed from the passed dictionary.

        :param meta_data: Dictionary of loaded meta data
        """
        for data_port_m in self.input_data_ports:
            self._copy_element_meta_data_from_meta_file_data(meta_data, data_port_m, "input_data_port",
                                                             data_port_m.data_port.data_port_id)
        for data_port_m in self.output_data_ports:
            self._copy_element_meta_data_from_meta_file_data(meta_data, data_port_m, "output_data_port",
                                                             data_port_m.data_port.data_port_id)
        for outcome_m in self.outcomes:
            self._copy_element_meta_data_from_meta_file_data(meta_data, outcome_m, "outcome",
                                                             outcome_m.outcome.outcome_id)

    @staticmethod
    def _copy_element_meta_data_from_meta_file_data(meta_data, element_m, element_name, element_id):
        """Helper method to assign the meta of the given element

        The method assigns the meta data of the elements from the given meta data dictionary. The copied meta data is
        then removed from the dictionary.

        :param meta_data: The loaded meta data
        :param element_m: The element model that is supposed to retrieve the meta data
        :param element_name: The name string of the element type in the dictionary
        :param element_id: The id of the element
        """
        meta_data_element_id = element_name + str(element_id)
        meta_data_element = meta_data[meta_data_element_id]
        AbstractStateModel.overwrite_editor_meta(meta_data_element)
        element_m.meta = meta_data_element
        del meta_data[meta_data_element_id]

    def _generate_element_meta_data(self, meta_data):
        """Generate meta data for state elements and add it to the given dictionary

        This method retrieves the meta data of the state elements (data ports, outcomes, etc) and adds it to the
        given meta data dictionary.

        :param meta_data: Dictionary of meta data
        """
        for data_port_m in self.input_data_ports:
            self._copy_element_meta_data_to_meta_file_data(meta_data, data_port_m, "input_data_port",
                                                           data_port_m.data_port.data_port_id)
        for data_port_m in self.output_data_ports:
            self._copy_element_meta_data_to_meta_file_data(meta_data, data_port_m, "output_data_port",
                                                           data_port_m.data_port.data_port_id)
        for outcome_m in self.outcomes:
            self._copy_element_meta_data_to_meta_file_data(meta_data, outcome_m, "outcome",
                                                           outcome_m.outcome.outcome_id)

    @staticmethod
    def _copy_element_meta_data_to_meta_file_data(meta_data, element_m, element_name, element_id):
        """Helper method to generate meta data for meta data file for the given element

        The methods loads teh meta data of the given element and copies it into the given meta data dictionary
        intended for being stored on the filesystem.

        :param meta_data: The meta data to be stored
        :param element_m: The element model to get the meta data from
        :param element_name: The name string of the element type in the dictionary
        :param element_id: The id of the element
        """
        meta_data_element_id = element_name + str(element_id)
        meta_data_element = element_m.meta
        meta_data[meta_data_element_id] = meta_data_element
