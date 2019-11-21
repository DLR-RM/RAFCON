# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import str
import os.path
from copy import copy, deepcopy
from weakref import ref
from gtkmvc3.model_mt import ModelMT
from gtkmvc3.observable import Signal
import gtkmvc3.support.wrappers as wrappers

from rafcon.gui.models.signals import MetaSignalMsg, Notification
from rafcon.gui.models.meta import MetaModel

from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import State
from rafcon.core.storage import storage

from rafcon.utils import storage_utils, constants
from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict
from rafcon.utils import log


logger = log.get_logger(__name__)


def mirror_y_axis_in_vividict_element(vividict, key):
    from rafcon.gui.helpers.meta_data import contains_geometric_info
    rel_pos = vividict[key]
    if contains_geometric_info(rel_pos):
        vividict[key] = (rel_pos[0], -rel_pos[1])
    else:
        del vividict[key]
    return vividict


def get_state_model_class_for_state(state):
    """Determines the model required for the given state class

    :param state: Instance of a state (ExecutionState, BarrierConcurrencyState, ...)
    :return: The model class required for holding such a state instance
    """
    from rafcon.gui.models.state import StateModel
    from rafcon.gui.models.container_state import ContainerStateModel
    from rafcon.gui.models.library_state import LibraryStateModel
    if isinstance(state, ContainerState):
        return ContainerStateModel
    elif isinstance(state, LibraryState):
        return LibraryStateModel
    elif isinstance(state, State):
        return StateModel
    else:
        logger.warning("There is not model for state of type {0} {1}".format(type(state), state))
        return None


class AbstractStateModel(MetaModel, Hashable):
    """This is an abstract class serving as base class for state models

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param state: The state to be managed which can be any derivative of rafcon.core.states.state.State.
    :param AbstractStateModel parent: The state to be managed
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state
     """

    _parent = None
    _is_about_to_be_destroyed_recursively = False
    is_start = None
    state = None
    income = None
    outcomes = []
    input_data_ports = []
    output_data_ports = []
    meta_signal = Signal()
    action_signal = Signal()
    destruction_signal = Signal()

    state_counter = 0

    __observables__ = ("state", "input_data_ports", "output_data_ports", "income", "outcomes", "is_start",
                       "meta_signal", "action_signal", "destruction_signal")

    def __init__(self, state, parent=None, meta=None):
        AbstractStateModel.state_counter += 1
        if type(self) == AbstractStateModel:
            raise NotImplementedError

        MetaModel.__init__(self, meta)
        assert isinstance(state, State)
        self.is_start = False

        self.state = state

        self.parent = parent

        self.meta_signal = Signal()
        self.action_signal = Signal()
        self.destruction_signal = Signal()

        self.register_observer(self)

        self.input_data_ports = []
        self.output_data_ports = []
        self.outcomes = []
        self._load_port_models()

    def __str__(self):
        return "Model of state: {0}".format(self.state)

    def __eq__(self, other):
        if type(self) != type(other):
            return False
        if self.state != other.state:
            return False
        if self.meta != other.meta:
            return False
        for attr in self.state.state_element_attrs:
            # some state element attributes are wrapped in a wrappers.ObsWrapperBase class, some of which are held in
            # lists or dicts. Those need to be compared element-wise.
            my_attr = getattr(self, attr)
            other_attr = getattr(other, attr)
            if isinstance(my_attr, wrappers.ObsSeqWrapper):
                if isinstance(my_attr, wrappers.ObsListWrapper):  # elements are stored in a list
                    elements = my_attr
                    other_elements = other_attr
                elif isinstance(my_attr, wrappers.ObsMapWrapper):  # elements are stored in a dict
                    elements = my_attr.items()
                    other_elements = other_attr.items()
                else:
                    raise ValueError("Unsupported state element type: " + str(type(my_attr)))
                if len(elements) != len(other_elements):
                    return False
                if not all([element in other_elements for element in other_elements]):
                    return False
            else:
                return my_attr == other_attr
        return True

    def __hash__(self):
        return id(self)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __cmp__(self, other):
        if isinstance(other, AbstractStateModel):
            return self.core_element.__cmp__(other.core_element)

    def __lt__(self, other):
        return self.__cmp__(other) < 0

    def __contains__(self, item):
        """Checks whether `item` is an element of the state model

        Following child items are checked: outcomes, input data ports, output data ports

        :param item: :class:`StateModel` or :class:`StateElementModel`
        :return: Whether item is a direct child of this state
        :rtype: bool
        """
        from rafcon.gui.models.state_element import StateElementModel
        if not isinstance(item, StateElementModel):
            return False
        return item is self.income or item in self.outcomes or \
               item in self.input_data_ports or item in self.output_data_ports

    def __copy__(self):
        state = copy(self.state)
        state_m = self.__class__(state, parent=None, meta=None, load_meta_data=False)
        state_m.copy_meta_data_from_state_m(self)
        return state_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def update_is_start(self):
        """Updates the `is_start` property of the state
        
        A state is a start state, if it is the root state, it has no parent, the parent is a LibraryState or the state's
        state_id is identical with the ContainerState.start_state_id of the ContainerState it is within.
        """
        self.is_start = self.state.is_root_state or \
                        self.parent is None or \
                        isinstance(self.parent.state, LibraryState) or \
                        self.state.state_id == self.state.parent.start_state_id

    @property
    def core_element(self):
        return self.state

    @property
    def hierarchy_level(self):
        # TODO rewrite it to be more efficient -> try a recursive pattern on parent
        return len(self.state.get_path().split('/'))

    def prepare_destruction(self, recursive=True):
        """Prepares the model for destruction

        Recursively un-registers all observers and removes references to child models
        """
        if self.state is None:
            logger.verbose("Multiple calls of prepare destruction for {0}".format(self))
        self.destruction_signal.emit()
        try:
            self.unregister_observer(self)
        except KeyError:  # Might happen if the observer was already unregistered
            logger.verbose("Observer already unregistered!")
            pass
        if recursive:
            if self.income:
                self.income.prepare_destruction()
            for port in self.input_data_ports[:] + self.output_data_ports[:] + self.outcomes[:]:
                port.prepare_destruction()
        del self.input_data_ports[:]
        del self.output_data_ports[:]
        del self.outcomes[:]
        self.state = None
        self.input_data_ports = None
        self.output_data_ports = None
        self.income = None
        self.outcomes = None
        # History TODO: these are needed by the modification history
        # self.action_signal = None
        # self.meta_signal = None
        # self.destruction_signal = None
        self.observe = None
        super(AbstractStateModel, self).prepare_destruction()

    def update_hash(self, obj_hash):
        self.update_hash_from_dict(obj_hash, self.core_element)
        for state_element in sorted([self.income] + self.outcomes[:] + self.input_data_ports[:] + self.output_data_ports[:]):
            self.update_hash_from_dict(obj_hash, state_element)
        if not self.state.get_next_upper_library_root_state():
            self.update_hash_from_dict(obj_hash, self.meta)

    def update_meta_data_hash(self, obj_hash):
        super(AbstractStateModel, self).update_meta_data_hash(obj_hash)
        for state_element in sorted([self.income] + self.outcomes[:] + self.input_data_ports[:] + self.output_data_ports[:]):
            state_element.update_meta_data_hash(obj_hash)

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
        self.update_is_start()

    @property
    def is_about_to_be_destroyed_recursively(self):
        return self._is_about_to_be_destroyed_recursively

    @is_about_to_be_destroyed_recursively.setter
    def is_about_to_be_destroyed_recursively(self, value):
        if not isinstance(value, bool):
            raise TypeError("The is_about_to_be_destroyed_recursively property has to be boolean.")
        self._is_about_to_be_destroyed_recursively = value

    def get_state_machine_m(self, two_factor_check=True):
        """ Get respective state machine model

        Get a reference of the state machine model the state model belongs to. As long as the root state model
        has no direct reference to its state machine model the state machine manager model is checked respective model.

        :rtype: rafcon.gui.models.state_machine.StateMachineModel
        :return: respective state machine model
        """
        from rafcon.gui.singleton import state_machine_manager_model
        state_machine = self.state.get_state_machine()
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

    def _load_port_models(self):
        self._load_income_model()
        self._load_outcome_models()
        self._load_input_data_port_models()
        self._load_output_data_port_models()

    def _load_input_data_port_models(self):
        raise NotImplementedError

    def _load_output_data_port_models(self):
        raise NotImplementedError

    def _load_income_model(self):
        raise NotImplementedError

    def _load_outcome_models(self):
        raise NotImplementedError

    def child_model_changed(self, notification_overview):
        return self.state != notification_overview.get_affected_core_element()

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, prop_name, info):
        """This method notifies parent state about changes made to the state
        """
        # Notify the parent state about the change (this causes a recursive call up to the root state)
        if self.parent is not None:
            self.parent.model_changed(model, prop_name, info)

        if prop_name == 'parent':
            self.update_is_start()

    @ModelMT.observe("action_signal", signal=True)
    def action_signal_triggered(self, model, prop_name, info):
        """This method notifies the parent state and child state models about complex actions
        """
        msg = info.arg
        # print("action_signal_triggered state: ", self.state.state_id, model, prop_name, info)
        if msg.action.startswith('sm_notification_'):
            return
        # # affected child propagation from state
        # if hasattr(self, 'states'):
        #     for m in info['arg'].affected_models:
        #         print(m, self.states)
        #         print([m is mm for mm in self.states.values()], [m in self for m in info['arg'].affected_models], \)
        #             [m in self.states.values() for m in info['arg'].affected_models]
        if any([m in self for m in info['arg'].affected_models]):
            if not msg.action.startswith('parent_notification_'):
                new_msg = msg._replace(action='parent_notification_' + msg.action)
            else:
                new_msg = msg
            for m in info['arg'].affected_models:
                # print('???propagate it to', m, m.parent)
                if isinstance(m, AbstractStateModel) and m in self:
                    # print('!!!propagate it from {0} to {1} {2}'.format(self.state.state_id, m.state.state_id, m))
                    m.action_signal.emit(new_msg)

        if msg.action.startswith('parent_notification_'):
            return

        # recursive propagation of action signal TODO remove finally
        if self.parent is not None:
            # Notify parent about change of meta data
            info.arg = msg
            # print("DONE1", self.state.state_id, msg)
            self.parent.action_signal_triggered(model, prop_name, info)
            # print("FINISH DONE1", self.state.state_id, msg)
        # state machine propagation of action signal (indirect) TODO remove finally
        elif not msg.action.startswith('sm_notification_'):  # Prevent recursive call
            # If we are the root state, inform the state machine model by emitting our own meta signal.
            # To make the signal distinguishable for a change of meta data to our state, the change property of
            # the message is prepended with 'sm_notification_'
            # print("DONE2", self.state.state_id, msg)
            new_msg = msg._replace(action='sm_notification_' + msg.action)
            self.action_signal.emit(new_msg)
            # print("FINISH DONE2", self.state.state_id, msg)
        else:
            # print("DONE3 NOTHING")
            pass

    @ModelMT.observe("meta_signal", signal=True)
    def meta_changed(self, model, prop_name, info):
        """This method notifies the parent state about changes made to the meta data
        """
        msg = info.arg
        # print("meta_changed state: ", model, prop_name, info)
        if msg.notification is None:
            # Meta data of this state was changed, add information about notification to the signal message
            notification = Notification(model, prop_name, info)
            msg = msg._replace(notification=notification)
            # print("DONE0 ", msg)

        if self.parent is not None:
            # Notify parent about change of meta data
            info.arg = msg
            self.parent.meta_changed(model, prop_name, info)
            # print("DONE1 ", msg)
        elif not msg.change.startswith('sm_notification_'):  # Prevent recursive call
            # If we are the root state, inform the state machine model by emitting our own meta signal.
            # To make the signal distinguishable for a change of meta data to our state, the change property of
            # the message is prepended with 'sm_notification_'
            msg = msg._replace(change='sm_notification_' + msg.change)
            self.meta_signal.emit(msg)
            # print("DONE2 ", msg)
        else:
            # print("DONE3 NOTHING")
            pass

    # ---------------------------------------- meta data methods ---------------------------------------------

    def load_meta_data(self, path=None):
        """Load meta data of state model from the file system

        The meta data of the state model is loaded from the file system and stored in the meta property of the model.
        Existing meta data is removed. Also the meta data of all state elements (data ports, outcomes,
        etc) are loaded, as those stored in the same file as the meta data of the state.

        This is either called on the __init__ of a new state model or if a state model for a container state is created,
        which then calls load_meta_data for all its children.

        :param str path: Optional file system path to the meta data file. If not given, the path will be derived from
            the state's path on the filesystem
        :return: if meta data file was loaded True otherwise False
        :rtype: bool
        """
        # TODO: for an Execution state this method is called for each hierarchy level again and again, still?? check it!
        # print("1AbstractState_load_meta_data: ", path, not path)
        if not path:
            path = self.state.file_system_path
        # print("2AbstractState_load_meta_data: ", path)
        if path is None:
            self.meta = Vividict({})
            return False
        path_meta_data = os.path.join(path, storage.FILE_NAME_META_DATA)

        # TODO: Should be removed with next minor release
        if not os.path.exists(path_meta_data):
            logger.debug("Because meta data was not found in {0} use backup option {1}"
                         "".format(path_meta_data, os.path.join(path, storage.FILE_NAME_META_DATA_OLD)))
            path_meta_data = os.path.join(path, storage.FILE_NAME_META_DATA_OLD)
            # TODO use the following logger message to debug meta data load process and to avoid maybe repetitive loads
            # if not os.path.exists(path_meta_data):
            #     logger.info("path not found {0}".format(path_meta_data))

        try:
            # print("try to load meta data from {0} for state {1}".format(path_meta_data, self.state))
            tmp_meta = storage.load_data_file(path_meta_data)
        except ValueError as e:
            # if no element which is newly generated log a warning
            # if os.path.exists(os.path.dirname(path)):
            #     logger.debug("Because '{1}' meta data of {0} was not loaded properly.".format(self, e))
            if not path.startswith(constants.RAFCON_TEMP_PATH_STORAGE) and not os.path.exists(os.path.dirname(path)):
                logger.debug("Because '{1}' meta data of {0} was not loaded properly.".format(self, e))
            tmp_meta = {}

        # JSON returns a dict, which must be converted to a Vividict
        tmp_meta = Vividict(tmp_meta)

        if tmp_meta:
            self._parse_for_element_meta_data(tmp_meta)
            # assign the meta data to the state
            self.meta = tmp_meta
            self.meta_signal.emit(MetaSignalMsg("load_meta_data", "all", True))
            return True
        else:
            # print("nothing to parse", tmp_meta)
            return False

    def store_meta_data(self, copy_path=None):
        """Save meta data of state model to the file system

        This method generates a dictionary of the meta data of the state together with the meta data of all state
        elements (data ports, outcomes, etc.) and stores it on the filesystem.
        Secure that the store meta data method is called after storing the core data otherwise the last_stored_path is
        maybe wrong or None.
        The copy path is considered to be a state machine file system path but not the current one but e.g.
        of a as copy saved state machine. The meta data will be stored in respective relative state folder in the state
        machine  hierarchy. This folder has to exist.
        Dues the core elements of the state machine has to be stored first.

        :param str copy_path: Optional copy path if meta data is not stored to the file system path of state machine
        """
        if copy_path:
            meta_file_path_json = os.path.join(copy_path, self.state.get_storage_path(), storage.FILE_NAME_META_DATA)
        else:
            if self.state.file_system_path is None:
                logger.error("Meta data of {0} can be stored temporary arbitrary but by default first after the "
                             "respective state was stored and a file system path is set.".format(self))
                return
            meta_file_path_json = os.path.join(self.state.file_system_path, storage.FILE_NAME_META_DATA)
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
        self.income.meta = deepcopy(source_state_m.income.meta)

        self.meta_signal.emit(MetaSignalMsg("copy_state_m", "all", True))

    def _parse_for_element_meta_data(self, meta_data):
        """Load meta data for state elements

        The meta data of the state meta data file also contains the meta data for state elements (data ports,
        outcomes, etc). This method parses the loaded meta data for each state element model. The meta data of the
        elements is removed from the passed dictionary.

        :param meta_data: Dictionary of loaded meta data
        """
        # print("_parse meta data", meta_data)
        for data_port_m in self.input_data_ports:
            self._copy_element_meta_data_from_meta_file_data(meta_data, data_port_m, "input_data_port",
                                                             data_port_m.data_port.data_port_id)
        for data_port_m in self.output_data_ports:
            self._copy_element_meta_data_from_meta_file_data(meta_data, data_port_m, "output_data_port",
                                                             data_port_m.data_port.data_port_id)
        for outcome_m in self.outcomes:
            self._copy_element_meta_data_from_meta_file_data(meta_data, outcome_m, "outcome",
                                                             outcome_m.outcome.outcome_id)
        if "income" in meta_data:
            if "gui" in meta_data and "editor_gaphas" in meta_data["gui"] and \
                    "income" in meta_data["gui"]["editor_gaphas"]:  # chain necessary to prevent key generation
                del meta_data["gui"]["editor_gaphas"]["income"]
        elif "gui" in meta_data and "editor_gaphas" in meta_data["gui"] and \
                "income" in meta_data["gui"]["editor_gaphas"]:  # chain necessary to prevent key generation in meta data
            meta_data["income"]["gui"]["editor_gaphas"] = meta_data["gui"]["editor_gaphas"]["income"]
            del meta_data["gui"]["editor_gaphas"]["income"]
        self._copy_element_meta_data_from_meta_file_data(meta_data, self.income, "income", "")

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
        # print(meta_data_element_id, element_m, meta_data_element)
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

        self._copy_element_meta_data_to_meta_file_data(meta_data, self.income, "income", "")

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

    def _meta_data_editor_gaphas2opengl(self, vividict):
        vividict = mirror_y_axis_in_vividict_element(vividict, 'rel_pos')
        if 'income' in vividict:
            del vividict['income']
        if 'name' in vividict:
            del vividict['name']
        return vividict

    def _meta_data_editor_opengl2gaphas(self, vividict):
        from rafcon.gui.helpers.meta_data import contains_geometric_info
        vividict = mirror_y_axis_in_vividict_element(vividict, 'rel_pos')
        if contains_geometric_info(vividict['size']):
            self.temp['conversion_from_opengl'] = True
            size = vividict['size']

            # Determine size and position of NameView
            margin = min(size) / 12.
            name_height = min(size) / 8.
            name_width = size[0] - 2 * margin
            vividict['name']['size'] = (name_width, name_height)
            vividict['name']['rel_pos'] = (margin, margin)
        return vividict
