# Copyright (C) 2016-2018 DLR
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

"""
.. module:: modification_history
   :synopsis: The module provides classes to document, undo or redo state machine edit steps.

The History-Class provides the observation functionalities to register and identify all core or gui (graphical) edit
actions that are a actual change to the state machine. Those changes are stored as Action-Objects in the
ModificationsHistory-Class.

The HistoryChanges-Class provides the functionalities to organize and access all actions of the edit process.
Hereby the branching of the edit process is stored and should be accessible, too.

"""
from builtins import object
from builtins import str
import copy

from gtkmvc3.model_mt import ModelMT
from gtkmvc3.observable import Observable

from rafcon.gui.action import ActionDummy, Action, StateMachineAction, StateAction, DataPortAction, \
    ScopedVariableAction, OutcomeAction, TransitionAction, DataFlowAction, AddObjectAction, RemoveObjectAction, \
    MetaAction, get_state_element_meta
from rafcon.gui.models.signals import MetaSignalMsg, StateTypeChangeSignalMsg, ActionSignalMsg

from rafcon.core.states.state import State
from rafcon.core.state_machine import StateMachine
from rafcon.core.state_elements.data_flow import DataFlow
from rafcon.core.state_elements.data_port import DataPort, InputDataPort
from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.state_elements.scope import ScopedVariable
from rafcon.core.state_elements.transition import Transition

from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.utils.notification_overview import NotificationOverview
from rafcon.gui.helpers.meta_data import check_gaphas_state_machine_meta_data_consistency

from rafcon.utils import log
from rafcon.utils.constants import TEMP_PATH, RAFCON_TEMP_PATH_BASE, BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS

logger = log.get_logger(__name__)

HISTORY_DEBUG_LOG_FILE = RAFCON_TEMP_PATH_BASE + '../test_file.txt'


class ModificationsHistoryModel(ModelMT):
    state_machine_model = None
    modifications = None
    change_count = None

    __observables__ = ("modifications", "change_count",)

    def __init__(self, state_machine_model):
        ModelMT.__init__(self)

        assert isinstance(state_machine_model, StateMachineModel)
        self.state_machine_model = state_machine_model
        self.__state_machine_id = state_machine_model.state_machine.state_machine_id
        self._tmp_meta_storage = None
        self.tmp_meta_storage = self.get_root_state_element_meta()

        self.observe_model(self.state_machine_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

        self._active_action = None
        self.locked = False
        self.busy = False
        self.count_before = 0

        self.modifications = ModificationsHistory()
        self.change_count = 0

        self.fake = False

        self.refactored_history = True
        self.with_verbose = False
        self.with_debug_logs = False
        self.with_meta_data_actions = True
        self.check_gaphas_consistency = False

        self.re_initiate_meta_data()

    @property
    def active_action(self):
        return self._active_action

    @active_action.setter
    def active_action(self, value):
        self._active_action = value

    @property
    def tmp_meta_storage(self):
        return self._tmp_meta_storage

    @tmp_meta_storage.setter
    def tmp_meta_storage(self, value):
        self._tmp_meta_storage = value

    def get_root_state_element_meta(self):
        return get_state_element_meta(self.state_machine_model.root_state)

    def update_internal_tmp_storage(self):
        if self.check_gaphas_consistency:
            logger.info("Check gaphas view is meta data consistent before doing tmp-storage update")
            check_gaphas_state_machine_meta_data_consistency(self.state_machine_model, with_logger_messages=True)
        self.tmp_meta_storage = self.get_root_state_element_meta()

    def prepare_destruction(self):
        """Prepares the model for destruction

        Un-registers itself as observer from the state machine and the root state
        """
        try:
            self.relieve_model(self.state_machine_model)
            assert self.__buffered_root_state_model is self.state_machine_model.root_state
            self.relieve_model(self.__buffered_root_state_model)
            self.state_machine_model = None
            self.__buffered_root_state_model = None
            self.modifications.prepare_destruction()
        except KeyError:  # Might happen if the observer was already unregistered
            pass
        if self.active_action:
            try:
                self.active_action.prepare_destruction()
            except Exception as e:
                logger.exception("The modification history has had left over an active-action and "
                                 "could not destroy it {0}.".format(e))
            self.active_action = None

    def get_state_element_meta_from_internal_tmp_storage(self, state_path):
        path_elements = state_path.split('/')
        path_elements.pop(0)
        # print(path_elements)
        act_state_elements_meta = self.tmp_meta_storage
        for path_elem in path_elements:
            act_state_elements_meta = act_state_elements_meta['states'][path_elem]
        # print(act_state_elements_meta)
        return act_state_elements_meta

    def recover_specific_version(self, pointer_on_version_to_recover):
        """ Recovers a specific version of the all_time_history element by doing several undos and redos.

        :param pointer_on_version_to_recover: the id of the list element which is to recover
        :return:
        """
        # search for traceable path -> list of action to undo and list of action to redo
        logger.info("Going to history status #{0}".format(pointer_on_version_to_recover))
        undo_redo_list = self.modifications.get_undo_redo_list_from_active_trail_history_item_to_version_id(pointer_on_version_to_recover)
        logger.debug("Multiple undo and redo to reach modification history element of version {0} "
                     "-> undo-redo-list is: {1}".format(pointer_on_version_to_recover, undo_redo_list))
        # logger.debug("acquire lock 1 - for multiple action {0}".format(self.modifications.trail_pointer))
        self.state_machine_model.storage_lock.acquire()
        # logger.debug("acquired lock 1 - for multiple action {0}".format(self.modifications.trail_pointer))
        for elem in undo_redo_list:
            if elem[1] == 'undo':
                # do undo
                self._undo(elem[0])
            else:
                # do redo
                self._redo(elem[0])

        self.modifications.reorganize_trail_history_for_version_id(pointer_on_version_to_recover)
        self.change_count += 1
        # logger.debug("release lock 1 - for multiple action {0}".format(self.modifications.trail_pointer))
        self.state_machine_model.storage_lock.release()
        # logger.debug("released lock 1 - for multiple action {0}".format(self.modifications.trail_pointer))

    def _undo(self, version_id):
        self.busy = True
        # print("undo 1", self.modifications, self.modifications.all_time_history[version_id].action)
        self.modifications.all_time_history[version_id].action.undo()
        self.modifications.trail_pointer -= 1
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()
        self.update_internal_tmp_storage()

    def undo(self):
        if not self.modifications.trail_history or self.modifications.trail_pointer == 0 \
                or not self.modifications.trail_pointer < len(self.modifications.trail_history):
            logger.debug("There is no more action that can be undone")
            return
        # logger.debug("acquire lock 2 - for undo {0}".format(self.modifications.trail_pointer))
        if self.state_machine_model.storage_lock.locked():
            # logger.debug("is locked already 2 - for undo {0}".format(self.modifications.trail_pointer))
            return
        self.state_machine_model.storage_lock.acquire()
        # logger.debug("acquired lock 2 - for undo {0}".format(self.modifications.trail_pointer))
        self.busy = True
        # print("undo 2", self.modifications)
        self.modifications.undo()
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()
        self.update_internal_tmp_storage()
        self.change_count += 1
        # logger.debug("release lock 2 - for undo {0}".format(self.modifications.trail_pointer))
        self.state_machine_model.storage_lock.release()
        # logger.debug("release lock 2 - for undo {0}".format(self.modifications.trail_pointer))

    def _redo(self, version_id):
        self.busy = True
        self.modifications.all_time_history[version_id].action.redo()
        self.modifications.trail_pointer += 1
        self.busy = False
        if self.modifications.trail_history is not None \
                and self.modifications.trail_pointer < len(self.modifications.trail_history) \
                and isinstance(self.modifications.trail_history[self.modifications.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

    def redo(self):
        if not self.modifications.trail_history or self.modifications.trail_history and not self.modifications.trail_pointer + 1 < len(
                self.modifications.trail_history):
            logger.debug("There is no more action that can be redone")
            return
        # logger.debug("acquire lock 3 - for redo")
        if self.state_machine_model.storage_lock.locked():
            # logger.debug("is locked already 3 - for redo {0}".format(self.modifications.trail_pointer))
            return
        self.state_machine_model.storage_lock.acquire()
        # logger.debug("acquired lock 3 - for redo")
        self.busy = True
        self.modifications.redo()
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()
        self.update_internal_tmp_storage()
        self.change_count += 1
        # logger.debug("release lock 3 - for redo")
        self.state_machine_model.storage_lock.release()
        # logger.debug("release lock 3 - for redo")

    def _interrupt_active_action(self, info=None):
        if self.with_verbose:
            logger.warning("function crash detected {}_after".format(info['prop_name']))
        else:
            logger.info("Active Action {} is interrupted and removed.".format(info['prop_name']))
        # self.busy = True
        self.active_action.prepare_destruction()
        # self.active_action = None
        # self.busy = False
        self.locked = False
        self.count_before = 0
        if self.with_verbose and info is not None:
            logger.verbose(NotificationOverview(info, False, self.__class__.__name__))
        if self.state_machine_model.storage_lock.locked():
            # logger.debug("release lock 0 - for interrupt active action")
            self.state_machine_model.storage_lock.release()
            # logger.debug("release lock 0 - for interrupt active action")

    def _re_initiate_observation(self):
        # logger.info("re initiate root_state observation")
        self.relieve_model(self.__buffered_root_state_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

    @staticmethod
    def store_test_log_file(string):
        with open(HISTORY_DEBUG_LOG_FILE, 'a+') as f:
            f.write(string)

    def start_new_action(self, overview):
        """

        :param overview:
        :return:
        """
        if self.fake:
            self.active_action = ActionDummy()
            return True

        result = True
        cause = overview['method_name'][-1]
        if self.with_verbose:
            logger.verbose("Create Action for: {0} for prop_name: {1}"
                           "".format(overview['method_name'][-1], overview['prop_name'][-1]))

        if self.with_debug_logs:
            self.store_test_log_file(str(overview) + "\n")
            if isinstance(overview['instance'][-1], State):
                self.store_test_log_file(overview['method_name'][-1] + "\t" + str(overview['instance'][-1]) + "\t" + overview['instance'][-1].get_path() + "\n")
            else:
                self.store_test_log_file(overview['method_name'][-1] + "\t" + str(overview['instance'][-1]) + "\t" + overview['instance'][-1].parent.get_path() + "\n")

        if self.refactored_history:
            if isinstance(overview['instance'][-1], DataFlow) or \
                    isinstance(overview['instance'][-1], Transition) or \
                    isinstance(overview['instance'][-1], ScopedVariable):
                if isinstance(overview['instance'][-1], DataFlow):
                    assert overview['instance'][-1] is overview['model'][-1].data_flow
                    action_class = DataFlowAction
                elif isinstance(overview['instance'][-1], Transition):
                    assert overview['instance'][-1] is overview['model'][-1].transition
                    action_class = TransitionAction
                else:
                    assert overview['instance'][-1] is overview['model'][-1].scoped_variable
                    action_class = ScopedVariableAction  # is a DataPort too
                if self.with_debug_logs:
                    self.store_test_log_file("#1 DataFlow, Transition, ScopedVariable \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.active_action = action_class(parent_path=overview['instance'][-1].parent.get_path(),
                                                  state_machine_model=self.state_machine_model,
                                                  overview=overview)
            elif isinstance(overview['instance'][-1], Outcome):
                assert overview['instance'][-1] is overview['model'][-1].outcome
                if self.with_debug_logs:
                    self.store_test_log_file("#2 Outcome \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.active_action = OutcomeAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                   state_machine_model=self.state_machine_model,
                                                   overview=overview)
            elif isinstance(overview['instance'][-1], DataPort):
                if isinstance(overview['instance'][-1], InputDataPort):
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                else:
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                if self.with_debug_logs:
                    self.store_test_log_file("#3 DataPort \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.active_action = DataPortAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            elif isinstance(overview['instance'][-1], State):
                assert overview['instance'][-1] is overview['model'][-1].state
                if "semantic_data" in overview['method_name'][-1]:
                    self.active_action = StateAction(parent_path=overview['instance'][-1].get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
                elif "add_" in cause:
                    if self.with_debug_logs:
                        self.store_test_log_file("#3 ADD \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.active_action = AddObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                         state_machine_model=self.state_machine_model,
                                                         overview=overview)
                elif "remove_" in cause:
                    assert cause in ["remove_transition", "remove_data_flow", "remove_income", "remove_outcome",
                                     "remove_input_data_port", "remove_output_data_port", "remove_scoped_variable",
                                     "remove_state"]
                    if ("transition" in cause or "data_flow" in cause or "scoped_variable" in cause or "state" in cause) or\
                            (("data_port" in cause or "outcome" in cause or "income" in cause) and not isinstance(overview['model'][-1].state.parent, State)):
                        if self.with_debug_logs:
                            self.store_test_log_file("#4 REMOVE1 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                        # if "transition" in cause:
                        #     return self.start_new_action_old(overview)
                        self.active_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                state_machine_model=self.state_machine_model,
                                                                overview=overview)
                    elif "data_port" in cause or "outcome" in cause or "income" in cause:

                        if isinstance(overview['instance'][-1].parent, State):
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE2 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.active_action = RemoveObjectAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                        else:
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE3 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.active_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                    else:
                        logger.warning("un foreseen cause: {0} in remove state element".format(cause))
                        assert False
                else:
                    if self.with_debug_logs:
                        self.store_test_log_file("#6 STATE \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.active_action = StateAction(parent_path=overview['instance'][-1].get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
            elif isinstance(overview['instance'][-1], StateMachine):
                assert overview['instance'][-1] is overview['model'][-1].state_machine
                assert False  # should never happen
            else:  # FAILURE
                logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                               "\ntype(instance): %s \nmodel[0]: %s \nprop_name[0]: %s \ninfo[-1]: %s \ninfo[0]: %s ",
                               type(overview['info'][-1]['instance']), overview['model'][0], overview['prop_name'][0],
                               overview['info'][-1], overview['info'][0])
                assert False  # should never happen

        else:
            result = self.start_new_action_old(overview)

        return result

    def start_new_action_old(self, overview):

        result = True

        if isinstance(overview['instance'][-1], DataFlow) or \
                isinstance(overview['instance'][-1], Transition) or \
                isinstance(overview['instance'][-1], ScopedVariable):  # internal modifications No Add or Remove Actions
            if self.with_debug_logs:
                self.store_test_log_file("$1 DataFlow, Transition, ScopedVariable Change\n model_path: {0}{1}\nparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
            if self.with_verbose:
                logger.verbose("CHANGE OF OBJECT", overview['info'][-1])
            # the model should be StateModel or ContainerStateModel and "info" from those model notification
            self.active_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                        state_machine_model=self.state_machine_model,
                                        overview=overview)

        elif overview['model'][-1].parent and (isinstance(overview['instance'][-1], DataPort) or
                                               isinstance(overview['instance'][-1], Outcome) or
                                               overview['method_name'][-1] in ['add_outcome', 'remove_outcome',
                                                                               'add_output_data_port',
                                                                               'remove_output_data_port',
                                                                               'add_input_data_port',
                                                                               'remove_input_data_port']):

            if self.with_verbose:
                if isinstance(overview['instance'][-1], State):
                    logger.verbose("Path_root1: ", overview['instance'][-1].get_path())
                else:
                    logger.verbose("Path_root1: ", overview['instance'][-1].parent.get_path())

            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):
                    level_status = 'State'
                    self.active_action = Action(parent_path=overview['instance'][-1].get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                elif not isinstance(overview['model'][-1].parent.state.parent, State):  # is root_state
                    level_status = 'ParentState'
                    self.active_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                else:
                    level_status = 'ParentParentState'
                    self.active_action = Action(parent_path=overview['instance'][-1].parent.parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                if self.with_debug_logs:
                    if isinstance(overview['instance'][-1], State):
                        self.store_test_log_file("$2 '{3}' add, remove modify of outcome, input or output\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path(),level_status))
                    else:
                        self.store_test_log_file("$2 '{3}' modify of outcome, input or output\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path(),level_status))
            else:
                assert False

        elif overview['prop_name'][-1] == 'state':
            if self.with_verbose:
                logger.verbose("Instance path: {0}", overview['instance'][-1].get_path())
                logger.verbose("Model path   : {0}", overview['model'][-1].state.get_path())
            if "add_" in overview['method_name'][-1]:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 add Outcome,In-OutPut in root and State, ScopedVariable, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.active_action = Action(parent_path=overview['instance'][-1].get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)
            else:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 remove Outcome,In-OutPut in root and State, ScopedVariables, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.active_action = Action(parent_path=overview['instance'][-1].get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)

        else:  # FAILURE
            logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                           "\n%s \n%s \n%s \n%s",
                           overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
            return False

        return result

    def finish_new_action(self, overview):
        if isinstance(self.active_action, MetaAction) and self.check_gaphas_consistency:
            check_gaphas_state_machine_meta_data_consistency(self.state_machine_model, with_logger_messages=True)

        # logger.debug("History stores AFTER")
        if self.with_debug_logs:
            self.store_test_log_file(str(overview) + "\n")

        try:
            self.active_action.set_after(overview)
            self.state_machine_model.history.modifications.insert_action(self.active_action)
            # logger.debug("history is now: %s" % self.state_machine_model.history.modifications.single_trail_history())
            self.update_internal_tmp_storage()
        except:
            logger.exception("Failure occurred while finishing action")
            # traceback.print_exc(file=sys.stdout)
            raise

        self.change_count += 1

    def re_initiate_meta_data(self):
        self.active_action = []
        self.update_internal_tmp_storage()

    @ModelMT.observe("meta_signal", signal=True)  # meta data of root_state_model changed
    # @ModelMT.observe("state_meta_signal", signal=True)  # meta data of state_machine_model changed
    def meta_changed_notify_after(self, changed_model, prop_name, info):
        if not self.with_meta_data_actions:
            return
        overview = NotificationOverview(info, False, self.__class__.__name__)
        # logger.info("meta_changed: \n{0}".format(overview))
        # filter self emit and avoid multiple signals of the root_state, by comparing first and last model in overview
        if len(overview['model']) > 1 and overview['model'][0] is overview['model'][-1]:
            # print("ALL", overview['signal'][0].change.startswith('sm_notification'))
            return
        if self.busy:
            return
        if overview['signal'][-1]['origin'] == 'load_meta_data':
            return

        if self.active_action is None or overview['signal'][-1]['change'] in ['append_initial_change']:
            # update last actions after_storage -> meta-data
            self.re_initiate_meta_data()
        elif self.active_action is None or \
                overview['signal'][-1]['change'] in ['append_to_last_change'] or \
                overview['signal'][-1]['origin'] in ['group_states', 'ungroup_state', 'substitute_state']:
            # update last actions after_storage -> meta-data
            self.active_action.after_storage = self.active_action.get_storage()
            self.update_internal_tmp_storage()
        else:
            if isinstance(overview['model'][-1], AbstractStateModel):
                changed_parent_model = overview['model'][-1]
            else:
                changed_parent_model = overview['model'][-1].parent
            changed_parent_state_path = changed_parent_model.state.get_path()

            # TODO think about to remove this work around again
            # ignore meta data changes inside of library states
            changed_parent_state = self.state_machine_model.get_state_model_by_path(changed_parent_state_path).state
            if changed_parent_state.get_next_upper_library_root_state():
                return

            if self.count_before == 0:
                self.active_action = MetaAction(changed_parent_model.state.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                # b_tuple = self.actual_action.before_storage
                meta_dict = self.get_state_element_meta_from_internal_tmp_storage(changed_parent_state_path)
                self.active_action.before_storage = meta_dict

                self.finish_new_action(overview)
            else:
                logger.info('Meta data change signal was emitted while other action was is performed. \n{0}'.format(overview))

    def before_count(self):
        if self.count_before == 0:
            # logger.debug('acquire lock 0 - for before count')
            self.state_machine_model.storage_lock.acquire()
            # logger.debug('acquired lock 0 - for before count')
            self.locked = True
        self.count_before += 1
        if self.with_verbose:
            logger.verbose("LOCKED count up {0}".format(self.count_before))

    def after_count(self):
        self.count_before -= 1
        if self.with_verbose:
            logger.verbose("LOCKED count down {0}".format(self.count_before))
        if self.count_before == 0:
            self.locked = False
            # logger.debug("release lock 1 - for after_count")
            self.state_machine_model.storage_lock.release()
            # logger.debug("release lock 1 - for after_count")

    @ModelMT.observe("state_action_signal", signal=True)
    def state_action_signal(self, model, prop_name, info):
        # logger.verbose("STATE ACTION SIGNAL: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        if self.busy:  # if proceeding undo or redo
            return
        # print("state: ", NotificationOverview(info, self.with_verbose, self.__class__.__name__))
        if isinstance(model, StateMachineModel) and isinstance(info['arg'], ActionSignalMsg) and \
                not info['arg'].after and info['arg'].action in ['change_root_state_type', 'change_state_type',
                                                                 'paste', 'cut',
                                                                 'substitute_state', 'group_states', 'ungroup_state']:

            overview = NotificationOverview(info, self.with_verbose, self.__class__.__name__)
            if self.with_debug_logs:
                self.store_test_log_file(str(overview) + "\n")

            overview['instance'].insert(0, self.state_machine_model.state_machine)
            overview['model'].insert(0, self.state_machine_model)
            if info['arg'].action == 'change_root_state_type':
                assert info['arg'].action_parent_m is self.state_machine_model
                # print("CREATE STATE MACHINE ACTION:", self.active_action)
                self.active_action = StateMachineAction(parent_path=info['arg'].action_parent_m.root_state.state.get_path(),
                                                        state_machine_model=info['arg'].action_parent_m,
                                                        overview=overview)
            else:
                # print("CREATE STATE ACTION:", self.active_action)
                self.active_action = StateAction(parent_path=info['arg'].action_parent_m.state.get_path(),
                                                 state_machine_model=self.state_machine_model,
                                                 overview=overview)

            self.before_count()
            if info['arg'].action in ['group_states', 'paste', 'cut']:
                self.observe_model(info['arg'].action_parent_m)
                # print("OBSERVE MODEL", info['arg'].action_parent_m)
            else:
                self.observe_model(info['arg'].affected_models[0])
                # print("OBSERVE MODEL", info['arg'].affected_models[0])

    @ModelMT.observe("action_signal", signal=True)
    def action_signal_after_complex_action(self, model, prop_name, info):
        # logger.verbose("ACTION SIGNAL: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        if self.busy:  # if proceeding undo or redo
            return
        if isinstance(model, AbstractStateModel) and isinstance(info['arg'], ActionSignalMsg) and \
                info['arg'].after and info['arg'].action in ['change_root_state_type', 'change_state_type',
                                                             'paste', 'cut',
                                                             'substitute_state', 'group_states', 'ungroup_state']:
            # print("\n\nIN AFTER\n\n", info['arg'].action, type(info['arg'].result), self.count_before)

            overview = NotificationOverview(info, self.with_verbose, "History state_machine_AFTER")
            if info['arg'].action in ['change_state_type', 'paste', 'cut',
                                      'substitute_state', 'group_states', 'ungroup_state']:

                if self.__buffered_root_state_model is not model:
                    self.relieve_model(model)
                    # logger.verbose("RELIEVE MODEL {0}".format(model))

            if isinstance(info['arg'].result, Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                else:
                    logger.warning("Count is wrong 1 != {0}".format(self.count_before))

            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    # print("\n\nAFTER\n\n")
                    self.finish_new_action(overview)
                    if info['arg'].action == 'change_root_state_type':
                        self._re_initiate_observation()
                    if self.with_verbose:
                        logger.verbose("HISTORY COUNT WAS OF SUCCESS FOR STATE MACHINE")
            else:
                logger.error("HISTORY after not count to 0 [action signal] -> For every before there should be a after."
                             "{0}".format(NotificationOverview(info)))

    @ModelMT.observe("states", before=True)
    def assign_notification_states_before(self, model, prop_name, info):
        # logger.verbose("states_before: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        if self.busy:  # if proceeding undo or redo
            return
        else:
            # avoid to vast computation time
            if 'kwargs' in info and 'method_name' in info['kwargs'] and \
                    info['kwargs']['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
                return

            overview = NotificationOverview(info, self.with_verbose, self.__class__.__name__)
            # logger.debug("History states_BEFORE {0}".format(overview))

            # skipped state modifications
            if not overview['method_name'][0] == 'state_change' or overview['method_name'][-1] == 'parent':
                return

            # logger.debug("History states_BEFORE {0}".format(overview))
            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
                if self.with_verbose:
                    logger.verbose("NEW HISTORY ELEMENT")
                if self.start_new_action(overview):
                    self.before_count()
                else:
                    logger.error("FAILED to start NEW HISTORY ELEMENT [states]")

    @ModelMT.observe("states", after=True)
    def assign_notification_states_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine modifications. This
        then typically requires a redraw of the graphical editor, to display these modifications immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        # logger.verbose("states_after: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        if self.busy or info.method_name == 'state_change' and \
                info.kwargs.prop_name == 'state' and \
                info.kwargs.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        else:
            # logger.debug("History states_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            # avoid to vast computation time
            if 'kwargs' in info and 'method_name' in info['kwargs'] and \
                    info['kwargs']['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
                return

            overview = NotificationOverview(info, self.with_verbose, self.__class__.__name__)

            # handle interrupts of action caused by exceptions
            if overview['result'][-1] == "CRASH in FUNCTION" or isinstance(overview['result'][-1], Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                pass

            # modifications of parent are not observed
            if not overview['method_name'][0] == 'state_change' or overview['method_name'][-1] == 'parent':
                return

            # logger.debug("History states_AFTER {0}".format(overview))
            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    if self.with_verbose:
                        logger.verbose("HISTORY COUNT WAS OF SUCCESS")
            else:
                logger.error("HISTORY after not count [states] -> For every before there should be a after.")

    @ModelMT.observe("state", before=True)
    @ModelMT.observe("income", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("output_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def assign_notification_root_state_before(self, model, prop_name, info):

        # logger.verbose("root_state_before: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        # execution_status-changes are not observed
        if self.busy or info.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        # first element should be prop_name="state_machine", instance=StateMachine and model=StateMachineModel
        # second element should be Prop_name="states" if root_state child elements are changed
        # --- for root_state elements it has to be prop_name in ["data_flows", "transitions", "input_data_ports",
        #                                                        "output_data_ports", "scoped_variables"]
        # third (and last element) should be prop_name in ["data_flow", "transition", ...
        else:
            overview = NotificationOverview(info, self.with_verbose, self.__class__.__name__)
            # modifications of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            # logger.debug("History BEFORE {0}".format(overview))  # \n%s \n%s \n%s" % (model, prop_name, info))

            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
                if self.with_verbose:
                    logger.verbose("NEW HISTORY ELEMENT")

                if self.start_new_action(overview):
                    self.before_count()
                else:
                    logger.error("FAILED to start NEW HISTORY ELEMENT [root_state]")

    @ModelMT.observe("state", after=True)
    @ModelMT.observe("income", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def assign_notification_root_state_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine modifications. This
        then typically requires a redraw of the graphical editor, to display these modifications immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        # logger.verbose("root_state_after: " + str(NotificationOverview(info, False, self.__class__.__name__)))

        # execution_status-changes are not observed
        if self.busy or info.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        else:
            overview = NotificationOverview(info, self.with_verbose, self.__class__.__name__)
            # logger.debug("History state_AFTER {0}".format(overview))

            # handle interrupts of action caused by exceptions
            if overview['result'][-1] == "CRASH in FUNCTION" or isinstance(overview['result'][-1], Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                pass

            # modifications of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            # logger.debug("History state_AFTER {0}".format(overview))

            # decrease counter and finish action when reaching count=0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    if self.with_verbose:
                        logger.verbose("HISTORY COUNT WAS OF SUCCESS")
            else:
                logger.error("HISTORY after not count [root_state] -> For every before there should be a after.")


class HistoryTreeElement(object):

    def __init__(self, prev_id, action=None, next_id=None):
        self._prev_id = None
        self._next_id = None
        if prev_id is not None:
            self.prev_id = prev_id
        self.action = action
        if next_id is not None:
            self.next_id = next_id
        self._old_next_ids = []

    def __str__(self):
        return "prev_id: {0} next_id: {1} and other next_ids: {2}".format(self._prev_id, self._next_id, self._old_next_ids)

    def prepare_destruction(self):
        self.action.prepare_destruction()

    @property
    def prev_id(self):
        return self._prev_id

    @prev_id.setter
    def prev_id(self, prev_id):
        # logger.info("new_prev_id is: {0}".format(prev_id))
        assert isinstance(prev_id, int)
        self._prev_id = prev_id

    @property
    def next_id(self):
        return self._next_id

    @next_id.setter
    def next_id(self, next_id):
        # logger.info("new_next_id is: {0}".format(next_id))
        assert isinstance(next_id, int)
        if self._next_id is not None:
            self._old_next_ids.append(self._next_id)
        self._next_id = next_id
        if next_id in self._old_next_ids:
            self._old_next_ids.remove(next_id)

    @property
    def old_next_ids(self):
        return self._old_next_ids


class ModificationsHistory(Observable):
    """The Class holds a all time history and a trail history. The trail history holds directly all modifications made
    since the last reset until the actual last active change and the undone modifications of this branch of
    modifications.
    So the all time history holds a list of all modifications ordered by time whereby the list elements are TreeElements
    that know respective previous action's list id and the possible next action list ids (multiple branches). Hereby a
    fast search from a actual active branch (trail history) to specific version_id (some branch) can be performed and
    all recovery steps collected.
    Additionally there will be implemented functionalities that never forget a single
    change that was insert for debugging reasons.
    - the pointer are pointing on the next undo ... so redo is pointer + 1
    - all_actions is a type of a tree # prev_id, action, next_id, old_next_ids
    """

    # TODO remove explicit trail-history -> next_id is holding the same information and old_next_ids the branching
    def __init__(self):
        Observable.__init__(self)
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None

        self.with_verbose = False

        # self.test_action_dumps = False
        # self._tmp_file = TEMP_PATH + '/test_mod_history.txt'

        # insert initial dummy element
        self.insert_action(ActionDummy())

    def prepare_destruction(self):
        del self.trail_history[:]
        for tree_element in self.all_time_history:
            tree_element.prepare_destruction()
        del self.all_time_history[:]

    @Observable.observed
    def insert_action(self, action):

        prev_id = None
        if self.all_time_history:
            prev_id = self.trail_history[self.trail_pointer].version_id

        action.version_id = len(self.all_time_history)
        self.all_time_history.append(HistoryTreeElement(prev_id=prev_id, action=action))

        # set pointer of previous element
        if prev_id is not None:
            prev_tree_elem = self.all_time_history[prev_id]
            prev_old_next_ids = copy.deepcopy(prev_tree_elem.old_next_ids)
            if self.with_verbose:
                logger.verbose("New pointer {0} element {1} -> new next_id {2}"
                               "".format(self.all_time_history[self.trail_pointer].action.version_id,
                                         prev_tree_elem,
                                         len(self.all_time_history) - 1))
            prev_tree_elem.next_id = len(self.all_time_history) - 1
            if not prev_old_next_ids == prev_tree_elem.old_next_ids:
                logger.info("This action has created a new branch in the state machine modification-history")

        # check single trail history and reduce trail history if the trail_pointer does not point on the last element
        if self.trail_pointer is not None:
            if self.trail_pointer > len(self.trail_history) - 1 or self.trail_pointer < 0:
                logger.error('History is broken may!!! %s' % self.trail_pointer)
            while not self.trail_pointer == len(self.trail_history) - 1:
                if self.with_verbose:
                    logger.verbose("pointer: {0} {1}".format(self.trail_pointer, len(self.trail_history)))
                self.trail_history.pop()
        # append new action to trail history and set actual trail pointer
        self.trail_history.append(action)
        self.trail_pointer = None if len(self.trail_history) == 0 else len(self.trail_history) - 1

        if self.with_verbose and action is not None:
            logger.verbose("New trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))
        # self.write_trail_history_to_file()

    # def write_trail_history_to_file(self):
    #     if self.test_action_dumps:
    #         with open(self._tmp_file, 'w+') as f:
    #             for a in self.trail_history:
    #                 h_elem = self.all_time_history[a.version_id]
    #                 s = str(h_elem.summary()) + "--{#}--" + h_elem.as_json_string() + '\n'
    #                 # print('\n'.join(s.split("--{#}--")))
    #                 f.write(s)

    @Observable.observed
    def undo(self):
        if not self.trail_history or self.trail_pointer == 0 or not self.trail_pointer < len(self.trail_history):
            logger.debug("There is no more action that can be undone")
            return
        # print("MODEHISTORY UNDO", self.trail_history[self.trail_pointer])
        self.trail_history[self.trail_pointer].undo()
        self.trail_pointer -= 1
        if self.with_verbose:
            logger.verbose("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    @Observable.observed
    def redo(self):
        if not self.trail_history or self.trail_history and not self.trail_pointer + 1 < len(self.trail_history):
            logger.debug("There is no more action that can be redone")
            return
        # print("MODEHISTORY REDO", self.trail_history[self.trail_pointer])
        self.trail_history[self.trail_pointer + 1].redo()
        self.trail_pointer += 1
        if self.with_verbose:
            logger.verbose("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    def single_trail_history(self):
        return self.trail_history

    def get_all_active_actions(self):
        active_action_id = 0 if self.trail_pointer < 0 else self.trail_pointer
        end_id = self.single_trail_history()[active_action_id].version_id
        return [a.version_id for a in self.single_trail_history() if a.version_id <= end_id]

    def reorganize_trail_history_for_version_id(self, version_id):

        # check if something is to do
        all_trail_action = [a.version_id for a in self.trail_history]
        intermediate_version_id = int(version_id)
        if intermediate_version_id in all_trail_action:
            return

        # search path back to actual trail history
        path = []
        if self.with_verbose:
            logger.verbose("Old trail: {0}".format(all_trail_action))
        while intermediate_version_id not in all_trail_action:
            path.insert(0, intermediate_version_id)
            intermediate_version_id = self.all_time_history[intermediate_version_id].prev_id
        # cut of not needed actions
        trail_index = all_trail_action.index(intermediate_version_id)
        self.trail_history = self.trail_history[:trail_index+1]
        if self.with_verbose:
            logger.verbose("Cut of trail: {0}".format([a.version_id for a in self.trail_history]))

        # append all actions of the path -> active actions of the branch
        for version_id in path:
            # set default next_id to active trail
            self.all_time_history[self.trail_history[-1].version_id].next_id = version_id
            self.trail_history.append(self.all_time_history[version_id].action)
        if self.with_verbose:
            logger.verbose("New active trail: {0}".format([a.version_id for a in self.trail_history]))

        # adjust trail history point to new active id
        self.trail_pointer = len(self.trail_history) - 1

        # append all inactive actions of the branch
        while self.all_time_history[self.trail_history[-1].version_id].next_id:
            insert_version_id = self.all_time_history[self.trail_history[-1].version_id].next_id
            self.trail_history.append(self.all_time_history[insert_version_id].action)
        if self.with_verbose:
            logger.verbose("New trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    def get_undo_redo_list_from_active_trail_history_item_to_version_id(self, version_id):
        """Perform fast search from currently active branch to specific version_id and collect all recovery steps.
        """
        all_trail_action = [a.version_id for a in self.single_trail_history() if a is not None]
        all_active_action = self.get_all_active_actions()
        undo_redo_list = []
        _undo_redo_list = []

        intermediate_version_id = version_id
        if self.with_verbose:
            logger.verbose("Version_id    : {0} in".format(intermediate_version_id))
            logger.verbose("Active actions: {0} in: {1}".format(all_active_action,
                                                                intermediate_version_id in all_active_action))
            logger.verbose("Trail actions : {0} in: {1}".format(all_trail_action,
                                                                intermediate_version_id in all_trail_action))

        if intermediate_version_id not in all_trail_action:
            # get undo to come from version_id to trail_action
            while intermediate_version_id not in all_trail_action:
                _undo_redo_list.insert(0, (intermediate_version_id, 'redo'))
                intermediate_version_id = self.all_time_history[intermediate_version_id].prev_id
            intermediate_goal_version_id = intermediate_version_id
        else:
            intermediate_goal_version_id = version_id
        intermediate_version_id = self.trail_history[self.trail_pointer].version_id
        if self.with_verbose:
            logger.verbose("Version_id    : {0} {1}".format(intermediate_goal_version_id, intermediate_version_id))
            logger.verbose("Active actions: {0} in: {1}".format(all_active_action,
                                                                intermediate_version_id in all_active_action))
            logger.verbose("Trail actions : {0} in: {1}".format(all_trail_action,
                                                                intermediate_version_id in all_trail_action))

        # collect undo and redo on trail
        if intermediate_goal_version_id in all_active_action:
            # collect needed undo to reach intermediate version
            while not intermediate_version_id == intermediate_goal_version_id:
                undo_redo_list.append((intermediate_version_id, 'undo'))
                intermediate_version_id = self.all_time_history[intermediate_version_id].prev_id

        elif intermediate_goal_version_id in all_trail_action:
            # collect needed redo to reach intermediate version
            while not intermediate_version_id == intermediate_goal_version_id:
                intermediate_version_id = self.all_time_history[intermediate_version_id].next_id
                undo_redo_list.append((intermediate_version_id, 'redo'))

        for elem in _undo_redo_list:
            undo_redo_list.append(elem)

        return undo_redo_list

    def is_end(self):
        return len(self.trail_history) - 1 == self.trail_pointer

    @Observable.observed
    def reset(self):
        logger.debug("################ RESET ChangeHistory PUT ALL TO INITIATION")
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None

        # insert initial dummy element
        self.insert_action(ActionDummy())

