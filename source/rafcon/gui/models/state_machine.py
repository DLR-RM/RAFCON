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
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
import threading
from copy import copy, deepcopy

from gtkmvc3.model_mt import ModelMT
from gtkmvc3.observable import Signal
from gtkmvc3.observer import Observer

from rafcon.core.state_machine import StateMachine
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.storage import storage
from rafcon.gui.config import global_gui_config
from rafcon.gui.models.meta import MetaModel
from rafcon.gui.models import ContainerStateModel, AbstractStateModel, StateModel, LibraryStateModel
from rafcon.gui.models.selection import Selection
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.utils.notification_overview import NotificationOverview
import rafcon.gui.utils.constants
from rafcon.utils import log, constants
from rafcon.utils import storage_utils
from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict
from rafcon.utils.timer import measure_time

logger = log.get_logger(__name__)


class StateMachineModel(MetaModel, Hashable):
    """This model class manages a :class:`rafcon.core.state_machine.StateMachine`

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine).

    :param StateMachine state_machine: The state machine to be controlled and modified
    """

    state_machine = None
    selection = None
    root_state = None
    meta = None
    ongoing_complex_actions = None
    meta_signal = Signal()
    state_meta_signal = Signal()
    action_signal = Signal()
    state_action_signal = Signal()
    sm_selection_changed_signal = Signal()
    destruction_signal = Signal()

    suppress_new_root_state_model_one_time = False

    __observables__ = ("state_machine", "root_state", "meta_signal", "state_meta_signal", "sm_selection_changed_signal",
                       "action_signal", "state_action_signal", "destruction_signal", "ongoing_complex_actions")

    @measure_time
    def __init__(self, state_machine, meta=None, load_meta_data=True):
        """Constructor
        """
        MetaModel.__init__(self)  # pass columns as separate parameters

        assert isinstance(state_machine, StateMachine)

        self.state_machine = state_machine
        self.state_machine_id = state_machine.state_machine_id

        root_state = self.state_machine.root_state
        if isinstance(root_state, ContainerState):
            self.root_state = ContainerStateModel(root_state, parent=self, load_meta_data=load_meta_data)
        else:
            self.root_state = StateModel(root_state, parent=self, load_meta_data=load_meta_data)

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # ongoing_complex_actions is updated by ComplexActionObserver -> secure encapsulated observation
        # and made observable by state machine model here
        self.ongoing_complex_actions = {}
        self.complex_action_observer = ComplexActionObserver(self)

        self.meta_signal = Signal()
        self.state_meta_signal = Signal()
        self.action_signal = Signal()
        self.state_action_signal = Signal()
        self.sm_selection_changed_signal = Signal()
        self.destruction_signal = Signal()

        self.temp = Vividict()

        if load_meta_data:
            self.load_meta_data(recursively=False)

        self.selection = Selection(self.sm_selection_changed_signal)

        self.storage_lock = threading.Lock()  # lock can not be substituted by the state machine lock -> maybe because it is a RLock

        self.history = None
        if global_gui_config.get_config_value('HISTORY_ENABLED'):
            from rafcon.gui.models.modification_history import ModificationsHistoryModel
            self.history = ModificationsHistoryModel(self)
        else:
            logger.info("The modification history is disabled")

        self.auto_backup = None
        if global_gui_config.get_config_value('AUTO_BACKUP_ENABLED'):
            from rafcon.gui.models.auto_backup import AutoBackupModel
            self.auto_backup = AutoBackupModel(self)

        self.root_state.register_observer(self)
        self.register_observer(self)

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, StateMachineModel):
            return self.root_state == other.root_state and self.meta == other.meta
        else:
            return False

    def __hash__(self):
        return id(self)

    def __copy__(self):
        sm_m = self.__class__(copy(self.state_machine))
        sm_m.root_state.copy_meta_data_from_state_m(self.root_state)
        sm_m.meta = deepcopy(self.meta)
        return sm_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def core_element(self):
        return self.state_machine

    def __destroy__(self):
        self.destroy()

    def destroy(self):
        if self.auto_backup is not None:
            self.auto_backup.destroy()
            self.auto_backup = None

    def prepare_destruction(self):
        """Prepares the model for destruction

        Unregister itself as observer from the state machine and the root state
        """
        if self.state_machine is None:
            logger.verbose("Multiple calls of prepare destruction for {0}".format(self))
        self.destruction_signal.emit()
        if self.history is not None:
            self.history.prepare_destruction()
        if self.auto_backup is not None:
            self.auto_backup.prepare_destruction()
        try:
            self.unregister_observer(self)
            self.root_state.unregister_observer(self)
        except KeyError:  # Might happen if the observer was already unregistered
            pass
        with self.state_machine.modification_lock():
            self.root_state.prepare_destruction()
        self.root_state = None
        self.state_machine = None
        super(StateMachineModel, self).prepare_destruction()

    def update_hash(self, obj_hash):
        self.update_hash_from_dict(obj_hash, self.root_state)
        self.update_hash_from_dict(obj_hash, self.meta)

    def update_meta_data_hash(self, obj_hash):
        super(StateMachineModel, self).update_meta_data_hash(obj_hash)
        self.root_state.update_meta_data_hash(obj_hash)

    @ModelMT.observe("state", before=True)
    @ModelMT.observe("income", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("states", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def root_state_model_before_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self._send_root_state_notification(model, prop_name, info)

    @ModelMT.observe("state", after=True)
    @ModelMT.observe("income", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("states", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def root_state_model_after_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self._send_root_state_notification(model, prop_name, info)

    @ModelMT.observe("state_machine", after=True)
    def state_machine_model_after_change(self, model, prop_name, info):
        overview = NotificationOverview(info)
        if overview.caused_modification():
            if not self.state_machine.marked_dirty:
                self.state_machine.marked_dirty = True

    @ModelMT.observe("meta_signal", signal=True)
    def meta_changed(self, model, prop_name, info):
        """When the meta was changed, we have to set the dirty flag, as the changes are unsaved"""
        self.state_machine.marked_dirty = True
        msg = info.arg
        if model is not self and msg.change.startswith('sm_notification_'):  # Signal was caused by the root state
            # Emit state_meta_signal to inform observing controllers about changes made to the meta data within the
            # state machine
            # -> removes mark of "sm_notification_"-prepend to mark root-state msg forwarded to state machine label
            msg = msg._replace(change=msg.change.replace('sm_notification_', '', 1))
            self.state_meta_signal.emit(msg)

    @ModelMT.observe("action_signal", signal=True)
    def action_signal_triggered(self, model, prop_name, info):
        """When the action was performed, we have to set the dirty flag, as the changes are unsaved"""
        # print("ACTION_signal_triggered state machine: ", model, prop_name, info)
        self.state_machine.marked_dirty = True
        msg = info.arg
        if model is not self and msg.action.startswith('sm_notification_'):  # Signal was caused by the root state
            # Emit state_action_signal to inform observing controllers about changes made to the state within the
            # state machine
            # print("DONE1 S", self.state_machine.state_machine_id, msg, model)
            # -> removes mark of "sm_notification_"-prepend to mark root-state msg forwarded to state machine label
            msg = msg._replace(action=msg.action.replace('sm_notification_', '', 1))
            self.state_action_signal.emit(msg)
            # print("FINISH DONE1 S", self.state_machine.state_machine_id, msg)
        else:
            # print("DONE2 S", self.state_machine.state_machine_id, msg)
            pass

    @staticmethod
    def _list_modified(prop_name, info):
        """Check whether the given operation is a list operation

        The function checks whether the property that has been changed is a list. If so, the operation is investigated
        further. If the operation is a basic list operation, the function return True.
        :param prop_name: The property that was changed
        :param info: Dictionary with information about the operation
        :return: True if the operation was a list operation, False else
        """
        if prop_name in ["states", "transitions", "data_flows", "input_data_ports", "output_data_ports",
                         "scoped_variables", "outcomes"]:
            if info['method_name'] in ["append", "extend", "insert", "pop", "remove", "reverse", "sort",
                                       "__delitem__", "__setitem__"]:
                return True
        return False

    def get_state_model_by_path(self, path):
        """Returns the `StateModel` for the given `path` 
        
        Searches a `StateModel` in the state machine, who's path is given by `path`.
        
        :param str path: Path of the searched state 
        :return: The state with that path
        :rtype: StateModel
        :raises: ValueError, if path is invalid/not existing with this state machine  
        """
        path_elements = path.split('/')
        path_elements.pop(0)
        current_state_model = self.root_state
        for state_id in path_elements:
            if isinstance(current_state_model, ContainerStateModel):
                if state_id in current_state_model.states:
                    current_state_model = current_state_model.states[state_id]
                else:
                    raise ValueError("Invalid path: State with id '{}' not found in state with id {}".format(
                        state_id, current_state_model.state.state_id))
            elif isinstance(current_state_model, LibraryStateModel):
                if state_id == current_state_model.state_copy.state.state_id:
                    current_state_model = current_state_model.state_copy
                else:
                    raise ValueError("Invalid path: state id '{}' does not coincide with state id '{}' of state_copy "
                                     "of library state with id '{}'".format(
                                        state_id, current_state_model.state_copy.state.state_id,
                                        current_state_model.state.state_id))
            else:
                raise ValueError("Invalid path: State with id '{}' has no children".format(
                    current_state_model.state.state_id))
        return current_state_model

    @ModelMT.observe("state_machine", after=True)
    def root_state_assign(self, model, prop_name, info):
        if info.method_name != 'root_state':
            return
        if self.suppress_new_root_state_model_one_time:
            self.suppress_new_root_state_model_one_time = False
            return
        # print("ASSIGN ROOT_STATE", model, prop_name, info)
        try:
            self.root_state.unregister_observer(self)
        except KeyError:
            pass
        if isinstance(self.state_machine.root_state, ContainerState):  # could not be a LibraryState
            self.root_state = ContainerStateModel(self.state_machine.root_state)
        else:
            assert not isinstance(self.state_machine.root_state, LibraryState)
            self.root_state = StateModel(self.state_machine.root_state)
        self.root_state.register_observer(self)

    @ModelMT.observe("state_machine", after=True, before=True)
    def change_root_state_type(self, model, prop_name, info):
        if info.method_name != 'change_root_state_type':
            return

        if 'before' in info:
            self._send_root_state_notification(model, prop_name, info)
        else:
            # Do not forward the notification yet, but store its parameters locally at the function
            # The function helpers.state.change_state_type will forward the notification after some preparation
            self.change_root_state_type.__func__.suppressed_notification_parameters = [model, prop_name, info]

    def _send_root_state_notification(self, model, prop_name, info):
        cause = 'root_state_change'
        try:
            if 'before' in info:
                self.state_machine._notify_method_before(self.state_machine, cause, (self.state_machine, ), info)
            elif 'after' in info:
                self.state_machine._notify_method_after(self.state_machine, cause, None, (self.state_machine, ), info)
        except AssertionError as e:
            # This fixes an AssertionError raised by GTKMVC. It can probably occur, when a controller unregisters
            # itself from a model, while the notification chain still propagates upwards.
            logger.exception("A exception occurs in the _send_root_state_notification method {0}.".format(e))

    #######################################################
    # --------------------- meta data methods ---------------------
    #######################################################

    def load_meta_data(self, path=None, recursively=True):
        """Load meta data of state machine model from the file system

        The meta data of the state machine model is loaded from the file system and stored in the meta property of the
        model. Existing meta data is removed. Also the meta data of root state and children is loaded.

        :param str path: Optional path to the meta data file. If not given, the path will be derived from the state
            machine's path on the filesystem
        """
        meta_data_path = path if path is not None else self.state_machine.file_system_path

        if meta_data_path:
            path_meta_data = os.path.join(meta_data_path, storage.FILE_NAME_META_DATA)

            try:
                tmp_meta = storage.load_data_file(path_meta_data)
            except ValueError:
                tmp_meta = {}
        else:
            tmp_meta = {}

        # JSON returns a dict, which must be converted to a Vividict
        tmp_meta = Vividict(tmp_meta)

        if recursively:
            root_state_path = None if not path else os.path.join(path, self.root_state.state.state_id)
            self.root_state.load_meta_data(root_state_path)

        if tmp_meta:
            # assign the meta data to the state
            self.meta = tmp_meta
            self.meta_signal.emit(MetaSignalMsg("load_meta_data", "all", True))

    def store_meta_data(self, copy_path=None):
        """Save meta data of the state machine model to the file system

        This method generates a dictionary of the meta data of the state machine and stores it on the filesystem.

        :param str copy_path: Optional, if the path is specified, it will be used instead of the file system path
        """
        if copy_path:
            meta_file_json = os.path.join(copy_path, storage.FILE_NAME_META_DATA)
        else:
            meta_file_json = os.path.join(self.state_machine.file_system_path, storage.FILE_NAME_META_DATA)

        storage_utils.write_dict_to_json(self.meta, meta_file_json)

        self.root_state.store_meta_data(copy_path)


class ComplexActionObserver(Observer):
    """ This Observer observes the and structures the information of complex actions and separates those observations
     from the StateMachineModel to avoid to mix these with the root state observation of the state machine model.
     
     - In ongoing_complex_actions dictionary all active actions and there properties are hold
     - Only once at the end of an complex action the ongoing_complex_actions dictionary is empty
       and the nested_action_already_in list has elements in to secure accessibility of action properties in the pattern
     - The nested_action_already_in list is cleaned after the ongoing_complex_actions dictionary was cleared observable
    """

    def __init__(self, model):
        Observer.__init__(self)
        self.model = model
        self.observe_model(model)

        self.nested_action_already_in = []

    @property
    def ongoing_complex_actions(self):
        return self.model.ongoing_complex_actions

    @ongoing_complex_actions.setter
    def ongoing_complex_actions(self, value):
        self.model.ongoing_complex_actions = value

    @ModelMT.observe("state_action_signal", signal=True)
    def state_action_signal(self, model, prop_name, info):
        if not ('arg' in info and info['arg'].after is False):
            return

        msg = info['arg']
        action = msg.action

        if action in rafcon.gui.utils.constants.complex_actions:

            self.ongoing_complex_actions[action] = {}
            if action in ['group_states', 'paste', 'cut']:
                # msg.action_parent_m.register_observer(self)
                self.observe_model(msg.action_parent_m)
            else:
                self.observe_model(msg.affected_models[0])

            self.ongoing_complex_actions[action]['affected_models'] = msg.affected_models
            if action in ['change_state_type', 'change_root_state_type', 'undo/redo']:
                old_state_m = msg.affected_models[0]
                self.ongoing_complex_actions[action]['target'] = old_state_m
            else:
                self.ongoing_complex_actions[action]['target'] = msg.action_parent_m

    @ModelMT.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        if not (isinstance(model, AbstractStateModel) and 'arg' in info and info['arg'].after):
            return

        msg = info['arg']
        action = msg.action

        if isinstance(msg.result, Exception) and action in self.ongoing_complex_actions:
            pass

        if action in ['substitute_state', 'group_states', 'ungroup_state', 'paste', 'cut']:
            new_state_m = msg.action_parent_m
        elif action in ['change_state_type', 'change_root_state_type', 'undo/redo']:
            new_state_m = msg.affected_models[-1]
        else:
            return

        self.ongoing_complex_actions[action]['new'] = new_state_m
        self.nested_action_already_in.append((action, self.ongoing_complex_actions[action]))

        self.ongoing_complex_actions.pop(action)

        if not self.ongoing_complex_actions:
            # common case remove the view here in the after action signal
            self.relieve_model(model)
            self.nested_action_already_in = []
