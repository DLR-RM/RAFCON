import os.path
from copy import copy, deepcopy
from weakref import ref

from rafcon.mvc.models.signals import MetaSignalMsg, Notification
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.state import State
from rafcon.statemachine.storage import storage
from rafcon.utils import log
from rafcon.utils import storage_utils
from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict

from gtkmvc import ModelMT, Signal

logger = log.get_logger(__name__)


def diff_for_state_element_lists(self_list_of_elements, other_list_of_elements, name):
    id_name = name + '_id' if 'scope' not in name else 'data_port_id'
    self_dict = {getattr(getattr(elem, name), id_name): elem for elem in self_list_of_elements}
    try:
        # print "diff ", name
        diff_list = [self_dict[getattr(getattr(elem, name), id_name)] == elem for elem in other_list_of_elements]
        diff_list.append(len(self_list_of_elements) == len(other_list_of_elements))
    except KeyError:
        return [False]
    return diff_list


def get_state_model_class_for_state(state):
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
        logger.warning("There is not model for state of type {0} {1}".format(type(state), state))
        return None


class AbstractStateModel(ModelMT, Hashable):
    """This is an abstract class serving as base class for state models

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param rafcon.statemachine.states.state.State state: The state to be managed
    :param AbstractStateModel parent: The state to be managed
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state
     """

    _parent = None
    is_start = None
    state = None
    outcomes = []
    input_data_ports = []
    output_data_ports = []
    meta_signal = Signal()
    state_type_changed_signal = Signal()

    __observables__ = ("state", "input_data_ports", "output_data_ports", "outcomes", "is_start", "meta_signal",
                       "state_type_changed_signal")

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

        self.parent = parent

        self.state_type_changed_signal = Signal()

        self.register_observer(self)

        self.input_data_ports = []
        self.output_data_ports = []
        self.outcomes = []
        self._load_input_data_port_models()
        self._load_output_data_port_models()
        self._load_outcome_models()

    def __str__(self):
        return "Model of state: {0}".format(self.state)

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, AbstractStateModel):
            if not all(diff_for_state_element_lists(self.input_data_ports, other.input_data_ports, 'data_port')) or \
                    not all(diff_for_state_element_lists(self.output_data_ports, other.output_data_ports, 'data_port')) or \
                    not all(diff_for_state_element_lists(self.outcomes, other.outcomes, 'outcome')):
                return False
            return self.state == other.state and self.meta == other.meta
        else:
            return False

    def __copy__(self):
        state = copy(self.state)
        state_m = self.__class__(state, parent=None, meta=None, load_meta_data=False)
        state_m.copy_meta_data_from_state_m(self)
        return state_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def prepare_destruction(self):
        """Prepares the model for destruction

        Recursively unregisters all observers and removes references to child models
        """
        try:
            self.unregister_observer(self)
        except KeyError:  # Might happen if the observer was already unregistered
            pass
        for port in self.input_data_ports[:] + self.output_data_ports[:] + self.outcomes[:]:
            port.prepare_destruction()
        del self.input_data_ports[:]
        del self.output_data_ports[:]
        del self.outcomes[:]

    def update_hash(self, obj_hash):
        for state_element in self.outcomes[:] + self.input_data_ports[:] + self.output_data_ports[:]:
            state_element.update_hash(obj_hash)
        Hashable.update_hash_from_dict(obj_hash, self.meta)

    @property
    def parent(self):
        if not self._parent:
            return None
        return self._parent()

    @parent.setter
    def parent(self, parent_m):
        if isinstance(parent_m, AbstractStateModel):
            self._parent = ref(parent_m)
        else:
            self._parent = None

    def get_sm_m_for_state_m(self, two_factor_check=True):
        """ Get respective state machine model

        Get a reference of the state machine model the state model belongs to. As long as the root state model
        has no direct reference to its state machine model the state machine manager model is checked respective model.

        :rtype: rafcon.mvc.models.state_machine.StateMachineModel
        :return: respective state machine model
        """
        from rafcon.mvc.singleton import state_machine_manager_model
        state_machine = self.state.get_sm_for_state()
        if state_machine:
            if state_machine.state_machine_id in state_machine_manager_model.state_machines:
                sm_m = state_machine_manager_model.state_machines[state_machine.state_machine_id]
                if not two_factor_check or sm_m.get_state_model_by_path(self.state.get_path()) is self:
                    return sm_m
                else:
                    logger.debug("State model requesting its state machine model parent seems to be obsolete. "
                                   "This is a hint to duplicated models and dirty coding")

        return None

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
        msg = info.arg
        if msg.notification is None:
            # Meta data of this state was changed, add information about notification to the signal message
            notification = Notification(model, prop_name, info)
            msg = msg._replace(notification=notification)

        if self.parent is not None:
            # Notify parent about change of meta data
            info.arg = msg
            self.parent.meta_changed(model, prop_name, info)
        elif not msg.change.startswith('sm_notification_'):  # Prevent recursive call
            # If we are the root state, inform the state machine model by emitting our own meta signal.
            # To make the signal distinguishable for a change of meta data to our state, the change property of
            # the message is prepended with 'sm_notification_'
            msg = msg._replace(change='sm_notification_' + msg.change)
            self.meta_signal.emit(msg)

    def _mark_state_machine_as_dirty(self):
        state_machine = self.state.get_sm_for_state()
        if state_machine:
            state_machine_id = state_machine.state_machine_id
            from rafcon.statemachine.singleton import state_machine_manager
            if state_machine_id is not None and state_machine_id in state_machine_manager.state_machines:
                state_machine_manager.state_machines[state_machine_id].marked_dirty = True

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of state model from the file system

        The meta data of the state model is loaded from the file system and stored in the meta property of the model.
        Existing meta data is removed. Also the meta data of all state elements (data ports, outcomes,
        etc) are loaded, as it is stored in the same file as the meta data of the state.

        :param str path: Optional path to the meta data file. If not given, the path will be derived from the state's
            path on the filesystem
        """
        if not path:
            path = self.state.get_file_system_path()

        path_meta_data = os.path.join(path, storage.FILE_NAME_META_DATA)

        # TODO: Should be removed with next minor release
        if not os.path.exists(path_meta_data):
            path_meta_data = os.path.join(path, storage.FILE_NAME_META_DATA_OLD)

        try:
            tmp_meta = storage.load_data_file(path_meta_data)
        except ValueError:
            # raise
            tmp_meta = {}

        # JSON returns a dict, which must be converted to a Vividict
        tmp_meta = Vividict(tmp_meta)

        if tmp_meta:
            self._parse_for_element_meta_data(tmp_meta)
            # assign the meta data to the state
            self.meta = tmp_meta
            self.meta_signal.emit(MetaSignalMsg("load_meta_data", "all", True))

    def store_meta_data(self, temp_path=None):
        """Save meta data of state model to the file system

        This method generates a dictionary of the meta data of the state together with the meta data of all state
        elements (data ports, outcomes, etc.) and stores it on the filesystem.
        """
        if temp_path:
            meta_file_path_json = os.path.join(temp_path, self.state.get_storage_path(), storage.FILE_NAME_META_DATA)
        else:
            meta_file_path_json = os.path.join(self.state.get_file_system_path(), storage.FILE_NAME_META_DATA)
        meta_data = deepcopy(self.meta)
        self._generate_element_meta_data(meta_data)
        storage_utils.write_dict_to_json(meta_data, meta_file_path_json)

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

        self.meta_signal.emit(MetaSignalMsg("copy_state_m", "all", True))

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
