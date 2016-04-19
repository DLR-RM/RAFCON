""" The module provides classes to document, undo or redo state machine edit steps.

The History-Class provides the observation functionalities to register and identify all core or mvc (graphical) edit
actions that are a actual change to the state machine. Those changes are stored as Action-Objects in the
ModificationsHistory-Class.

The HistoryChanges-Class provides the functionalities to organize and access all actions of the edit process.
Hereby the branching of the edit process is stored and should be accessible, too.
"""
import copy
import time
import threading

from gtkmvc import ModelMT, Observable

from rafcon.utils import log

from rafcon.statemachine.scope import ScopedVariable
from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.data_flow import DataFlow
from rafcon.statemachine.transition import Transition
from rafcon.statemachine.states.state import State
from rafcon.statemachine.data_port import DataPort

from rafcon.statemachine.data_port import InputDataPort
from rafcon.statemachine.state_machine import StateMachine

from rafcon.mvc.utils.notification_overview import NotificationOverview
from rafcon.mvc.config import global_gui_config
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE

from rafcon.statemachine.storage import storage
from rafcon.mvc.models.abstract_state import AbstractStateModel


from rafcon.mvc.action import ActionDummy, Action, StateMachineAction, StateAction, DataPortAction, \
    ScopedVariableAction, OutcomeAction, TransitionAction, DataFlowAction, AddObjectAction, RemoveObjectAction, \
    MetaAction, get_state_element_meta

logger = log.get_logger(__name__)

HISTORY_DEBUG_LOG_FILE = RAFCON_TEMP_PATH_BASE + '../test_file.txt'


class ModificationsHistoryModel(ModelMT):
    state_machine_model = None
    modifications = None

    __observables__ = ("modifications",)

    def __init__(self, state_machine_model):
        ModelMT.__init__(self)

        # assert isinstance(state_machine_model, StateMachineModel)
        self.state_machine_model = state_machine_model
        self.__state_machine_id = state_machine_model.state_machine.state_machine_id
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

        self.observe_model(self.state_machine_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

        self.actual_action = None
        self.actual_root_state_action = None
        self.locked = False
        self.busy = False
        self.count_before = 0

        self.modifications = ModificationsHistory()

        self.fake = False

        self.refactored_history = True
        self.with_prints = False
        self.with_debug_logs = False
        self.with_meta_data_actions = True

        self.timed_temp_storage_enabled = global_gui_config.get_config_value('TIMED_TEMPORARY_STORAGE_ENABLED')
        self.force_temp_storage_interval = global_gui_config.get_config_value('FORCED_TEMPORARY_STORAGE_INTERVAL')
        self.timed_temp_storage_interval = global_gui_config.get_config_value('TIMED_TEMPORARY_STORAGE_INTERVAL')
        self.last_storage_time = time.time()
        self.storage_lock = threading.Lock()
        self.check_for_temp_storage(force=True)
        self.timer_request_lock = threading.Lock()
        self.timer_request_time = None
        self.tmp_storage_timed_thread = None

    def __destroy__(self):
        self.destroy()

    def destroy(self):
        if self.tmp_storage_timed_thread is not None:
            self.tmp_storage_timed_thread.cancel()

    def get_state_element_meta_from_tmp_storage(self, state_path):
        path_elements = state_path.split('/')
        path_elements.pop(0)
        # print path_elements
        act_state_elements_meta = self.tmp_meta_storage
        for path_elem in path_elements:
            act_state_elements_meta = act_state_elements_meta['states'][path_elem]
        # print act_state_elements_meta
        return act_state_elements_meta

    def recover_specific_version(self, pointer_on_version_to_recover):
        """ Recovers a specific version of the all_time_history element by doing several undos and redos.

        :param pointer_on_version_to_recover: the id of the list element which is to recover
        :return:
        """
        # search for traceable path -> list of action to undo and list of action to redo
        logger.info("Going to history status #{0}".format(pointer_on_version_to_recover))
        undo_redo_list = self.modifications.undo_redo_list_from_actual_trail_history_to_version_id(pointer_on_version_to_recover)
        logger.debug("Multiple undo and redo to reach modification history element of version {0} "
                    "-> undo-redo-list is: {1}".format(pointer_on_version_to_recover, undo_redo_list))
        for elem in undo_redo_list:
            if elem[1] == 'undo':
                # do undo
                self._undo(elem[0])
            else:
                # do redo
                self._redo(elem[0])

        self.modifications.reorganize_trail_history_for_version_id(pointer_on_version_to_recover)
        self.check_for_temp_storage()

    def _undo(self, version_id):
        self.busy = True
        self.modifications.all_time_history[version_id].action.undo()
        self.modifications.trail_pointer -= 1
        self.modifications.all_time_pointer -= 1
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()
        self.check_for_temp_storage()
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

    def undo(self):
        if not self.modifications.trail_history or self.modifications.trail_pointer == 0 \
                or not self.modifications.trail_pointer < len(self.modifications.trail_history):
            logger.debug("There is no more action that can be undone")
            return
        # else:
        #     logger.debug("do Undo %s %s %s" % (bool(self.modifications.trail_history), self.modifications.trail_history, (self.modifications.trail_pointer, len(self.modifications.trail_history))))
        self.busy = True
        self.modifications.undo()
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()
        self.check_for_temp_storage()
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

    def _redo(self, version_id):
        self.busy = True
        self.modifications.all_time_history[version_id].action.redo()
        self.modifications.trail_pointer += 1
        self.modifications.all_time_pointer += 1
        self.busy = False
        if self.modifications.trail_history is not None \
                and self.modifications.trail_pointer < len(self.modifications.trail_history) \
                and isinstance(self.modifications.trail_history[self.modifications.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()
        self.check_for_temp_storage()
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

    def redo(self):
        if not self.modifications.trail_history or self.modifications.trail_history and not self.modifications.trail_pointer + 1 < len(
                self.modifications.trail_history):
            logger.debug("There is no more action that can be redone")
            return
        # else:
        #     logger.debug("do Redo %s %s %s" % (bool(self.modifications.trail_history), self.modifications.trail_history, (self.modifications.trail_pointer, len(self.modifications.trail_history))))
        self.busy = True
        self.modifications.redo()
        self.busy = False
        if isinstance(self.modifications.trail_history[self.modifications.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()
        self.check_for_temp_storage()
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

    def _interrupt_actual_action(self):
        # self.busy = True
        # self.actual_action.undo()
        # self.busy = False
        self.locked = False
        self.count_before = 0

    def _re_initiate_observation(self):
        # logger.info("re initiate root_state observation")
        self.relieve_model(self.__buffered_root_state_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

    @staticmethod
    def store_test_log_file(string):
        with open(HISTORY_DEBUG_LOG_FILE, 'a+') as f:
            f.write(string)
        f.closed

    def start_new_action(self, overview):
        """

        :param overview:
        :return:
        """
        if self.fake:
            self.actual_action = ActionDummy()
            return True

        result = True
        cause = overview['method_name'][-1]
        if self.with_prints:
            logger.info("create Action for: {0} for prop_name: {1}".format(overview['method_name'][-1], overview['prop_name'][-1]))

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
                self.actual_action = action_class(parent_path=overview['instance'][-1].parent.get_path(),
                                                  state_machine_model=self.state_machine_model,
                                                  overview=overview)
            elif isinstance(overview['instance'][-1], Outcome):
                assert overview['instance'][-1] is overview['model'][-1].outcome
                if self.with_debug_logs:
                    self.store_test_log_file("#2 Outcome \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.actual_action = OutcomeAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                   state_machine_model=self.state_machine_model,
                                                   overview=overview)
            elif isinstance(overview['instance'][-1], DataPort):
                if isinstance(overview['instance'][-1], InputDataPort):
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                else:
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                if self.with_debug_logs:
                    self.store_test_log_file("#3 DataPort \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.actual_action = DataPortAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            elif isinstance(overview['instance'][-1], State):
                assert overview['instance'][-1] is overview['model'][-1].state
                if "add_" in cause:
                    if self.with_debug_logs:
                        self.store_test_log_file("#3 ADD \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.actual_action = AddObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                         state_machine_model=self.state_machine_model,
                                                         overview=overview)
                elif "remove_" in cause:
                    assert cause in ["remove_transition", "remove_data_flow", "remove_outcome", "remove_input_data_port",
                                     "remove_output_data_port", "remove_scoped_variable", "remove_state"]
                    if ("transition" in cause or "data_flow" in cause or "scoped_variable" in cause or "state" in cause) or\
                            (("data_port" in cause or "outcome" in cause) and not isinstance(overview['model'][-1].state.parent, State)):
                        if self.with_debug_logs:
                            self.store_test_log_file("#4 REMOVE1 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                        # if "transition" in cause:
                        #     return self.start_new_action_old(overview)
                        self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                state_machine_model=self.state_machine_model,
                                                                overview=overview)
                    elif "data_port" in cause or "outcome" in cause:

                        if isinstance(overview['instance'][-1].parent, State):
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE2 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                        else:
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE3 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                    else:
                        logger.warning("un foreseen cause: {0} in remove state element".format(cause))
                        assert False
                else:
                    if self.with_debug_logs:
                        self.store_test_log_file("#6 STATE \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.actual_action = StateAction(parent_path=overview['instance'][-1].get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
            elif isinstance(overview['instance'][-1], StateMachine):
                assert overview['instance'][-1] is overview['model'][-1].state_machine
                assert False  # should never happen
            else:  # FAILURE
                logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                            "\n%s \n%s \n%s \n%s",
                            overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
                assert False  # should never happen

            return result

        else:
            return self.start_new_action_old(overview)

    def start_new_action_old(self, overview):

        result = True

        if isinstance(overview['instance'][-1], DataFlow) or \
                isinstance(overview['instance'][-1], Transition) or \
                isinstance(overview['instance'][-1], ScopedVariable):  # internal modifications No Add or Remove Actions
            if self.with_debug_logs:
                self.store_test_log_file("$1 DataFlow, Transition, ScopedVariable Change\n model_path: {0}{1}\nparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
            if self.with_prints:
                print "CHANGE OF OBJECT", overview['info'][-1]
            # the model should be StateModel or ContainerStateModel and "info" from those model notification
            self.actual_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                        state_machine_model=self.state_machine_model,
                                        overview=overview)

        elif overview['model'][-1].parent and (isinstance(overview['instance'][-1], DataPort) or
                                               isinstance(overview['instance'][-1], Outcome) or
                                               overview['method_name'][-1] in ['add_outcome', 'remove_outcome',
                                                                               'add_output_data_port',
                                                                               'remove_output_data_port',
                                                                               'add_input_data_port',
                                                                               'remove_input_data_port']):

            if self.with_prints:
                if isinstance(overview['instance'][-1], State):
                    print "Path_root1: ", overview['instance'][-1].get_path()
                else:
                    print "Path_root1: ", overview['instance'][-1].parent.get_path()

            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):
                    level_status = 'State'
                    self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                elif not isinstance(overview['model'][-1].parent.state.parent, State):  # is root_state
                    level_status = 'ParentState'
                    self.actual_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                else:
                    level_status = 'ParentParentState'
                    self.actual_action = Action(parent_path=overview['instance'][-1].parent.parent.get_path(),
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
            if self.with_prints:
                print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            if "add_" in overview['method_name'][-1]:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 add Outcome,In-OutPut in root and State, ScopedVariable, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)
            else:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 remove Outcome,In-OutPut in root and State, ScopedVariables, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)

        else:  # FAILURE
            logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                        "\n%s \n%s \n%s \n%s",
                        overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
            return False

        return result

    def finish_new_action(self, overview):
        # logger.debug("History stores AFTER")
        if self.with_debug_logs:
            self.store_test_log_file(str(overview) + "\n")

        try:
            self.actual_action.set_after(overview)
            self.state_machine_model.history.modifications.insert_action(self.actual_action)
            # logger.debug("history is now: %s" % self.state_machine_model.history.modifications.single_trail_history())
            self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)
        except:
            logger.exception("Failure occurred while finishing action")
            # traceback.print_exc(file=sys.stdout)

        self.check_for_temp_storage()

    def _check_for_timed_auto_temp_storage(self):
        """ The method implements the timed storage feature.

         The method re-initiating a new timed thread if the state-machine not already stored to backup
         (what could be caused by the force_temp_storage_interval) or force the storing of the state-machine if there
         is no new request for a timed backup. New timed backup request are intrinsically represented by
         self.timer_request_time and initiated by the check_for_temp_storage-method.
         The feature uses only one thread for each ModificationHistoryModel and lock to be thread save.
        """
        actual_time = time.time()
        self.timer_request_lock.acquire()
        sm = self.state_machine_model.state_machine
        if self.last_storage_time > self.timer_request_time:
            # logger.info('{1} quit_thread {0}'.format(sm.state_machine_id, time.time()))
            self.timer_request_time = None
            self.tmp_storage_timed_thread = None
        elif self.timed_temp_storage_interval < actual_time - self.timer_request_time:
            # logger.info("{0} Perform timed auto-backup of state-machine {1}.".format(time.time(),
            #                                                                          sm.state_machine_id))
            self.check_for_temp_storage(force=True)
            self.timer_request_time = None
            self.tmp_storage_timed_thread = None
        else:
            duration_to_wait = self.timed_temp_storage_interval - (actual_time - self.timer_request_time)
            # logger.info('{2} restart_thread {0} time to go {1}'.format(sm.state_machine_id,
            #                                                            duration_to_wait, time.time()))
            self.tmp_storage_timed_thread = threading.Timer(duration_to_wait, self._check_for_timed_auto_temp_storage)
            self.tmp_storage_timed_thread.start()
        self.timer_request_lock.release()

    def perform_temp_storage(self):
        sm = self.state_machine_model.state_machine
        logger.info('Perform auto-backup of state-machine {} to tmp-folder'.format(sm.state_machine_id))
        if sm.file_system_path is None:
            tmp_sm_system_path = RAFCON_TEMP_PATH_BASE + '/runtime_backup/not_stored_' + str(sm.state_machine_id)
        else:
            tmp_sm_system_path = RAFCON_TEMP_PATH_BASE + '/runtime_backup/' + sm.file_system_path
        self.storage_lock.acquire()
        storage.save_statemachine_to_path(sm, tmp_sm_system_path, delete_old_state_machine=False,
                                          save_as=True, temporary_storage=True)
        self.state_machine_model.store_meta_data(temp_path=tmp_sm_system_path)
        self.storage_lock.release()

    def check_for_temp_storage(self, force=False):
        """ The method implements the checks for possible temporary backup of the state-machine according duration till
        the last change together with the private method _check_for_timed_auto_temp_storage.

        :param force: is a flag that force the temporary backup of the state-machine to the tmp-folder
        :return:
        """
        if not self.timed_temp_storage_enabled:
            return
        sm = self.state_machine_model.state_machine
        actual_time = time.time()
        if sm.marked_dirty and actual_time - self.last_storage_time > self.force_temp_storage_interval or force:

            self.perform_temp_storage()
            self.last_storage_time = actual_time
        else:
            self.timer_request_lock.acquire()
            if self.timer_request_time is None:
                # logger.info('{0} start_thread {1}'.format(actual_time, sm.state_machine_id))
                self.timer_request_time = actual_time
                self.tmp_storage_timed_thread = threading.Timer(self.timed_temp_storage_interval,
                                                                self._check_for_timed_auto_temp_storage)
                self.tmp_storage_timed_thread.start()
            else:
                # logger.info('{0} update_thread {1}'.format(actual_time, sm.state_machine_id))
                self.timer_request_time = actual_time
            self.timer_request_lock.release()

    @ModelMT.observe("meta_signal", signal=True)  # meta data of root_state_model changed
    # @ModelMT.observe("state_meta_signal", signal=True)  # meta data of state_machine_model changed
    def meta_changed_notify_after(self, changed_model, prop_name, info):
        if not self.with_meta_data_actions:
            return
        overview = NotificationOverview(info)
        # logger.info("meta_changed: \n{0}".format(overview))
        # WORKAROUND: avoid multiple signals of the root_state, by comparing first and last model in overview
        if len(overview['model']) > 1 and overview['model'][0] is overview['model'][-1] or \
                overview['meta_signal'][-1]['change'] == 'all':  # avoid strange change: 'all' TODO test why those occur
            return
        if self.busy:
            return

        if overview['meta_signal'][-1]['change'] == 'append_to_last_change':
             # update last actions after_storage -> meta-data
            self.actual_action.after_storage = self.actual_action.get_storage()
            self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)
        else:
            if isinstance(overview['model'][0], AbstractStateModel):
                changed_parent_model = overview['model'][0]
            else:
                changed_parent_model = overview['model'][0].parent
            self.actual_action = MetaAction(changed_parent_model.state.get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)
            # b_tuple = self.actual_action.before_storage
            meta_dict = self.get_state_element_meta_from_tmp_storage(changed_parent_model.state.get_path())
            self.actual_action.before_storage = meta_dict
            self.finish_new_action(overview)

    def manual_changed_notify_before(self, change_type, changed_parent_model, changed_model, recursive_changes):
        pass

    def manual_changed_notify_after(self, change_type, changed_parent_model, changed_model, recursive_changes):
        pass

    def before_count(self):
        if self.count_before == 0:
            self.storage_lock.acquire()
            self.locked = True
        self.count_before += 1
        if self.with_prints:
            print "LOCKED count up", self.count_before

    def after_count(self):
        self.count_before -= 1
        if self.with_prints:
            print "LOCKED count down", self.count_before
        if self.count_before == 0:
            self.locked = False
            self.storage_lock.release()

    @ModelMT.observe("state_machine", before=True)
    def assign_notification_change_type_root_state_before(self, model, prop_name, info):
        if info.method_name != "root_state_change":
            return
        if self.busy:  # if proceeding undo or redo
            return

        if info['kwargs']['method_name'] == "change_root_state_type":
            if self.with_prints:
                print "ROOT_STATE is NEW", model, prop_name, info
            overview = NotificationOverview(info)
            if self.with_debug_logs:
                self.store_test_log_file(str(overview) + "\n")
            assert overview['method_name'][-1]
            self.actual_action = StateMachineAction(parent_path=overview['instance'][-1].root_state.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            self.before_count()

    @ModelMT.observe("state_machine", after=True)
    def assign_notification_change_type_root_state_after(self, model, prop_name, info):
        if info.method_name != "root_state_change":
            return
        if info.result == "CRASH in FUNCTION" or isinstance(info.result, Exception):
            if self.with_prints:
                logger.warning("function crash detected sm_after")
            return self._interrupt_actual_action()

        if self.busy:  # if proceeding undo or redo
            return

        if info['kwargs']['method_name'] == "change_root_state_type":
            overview = NotificationOverview(info)
            assert overview['method_name'][-1] == "change_root_state_type"
            if self.with_prints:
                print "History state_machine_AFTER {0}".format(overview)

            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    self._re_initiate_observation()
                    if self.with_prints:
                        print "HISTORY COUNT WAS OF SUCCESS FOR STATE MACHINE"
            else:
                logger.error("HISTORY after not count [state_machine] -> For every before there should be a after.")

    @ModelMT.observe("states", before=True)
    def assign_notification_states_before(self, model, prop_name, info):
        if self.with_prints:
            print "states_before: ", model, prop_name, info
        if self.busy:  # if proceeding undo or redo
            return
        else:
            overview = NotificationOverview(info, with_prints=self.with_prints)
            # logger.debug("History states_BEFORE {0}".format(overview)

            # skipped state modifications
            if (overview['method_name'][0] == 'state_change' and
                    overview['method_name'][-1] in ['active',
                                                    'child_execution',
                                                    'state_execution_status']) or \
                    not overview['method_name'][0] == 'state_change' or \
                    overview['method_name'][-1] == 'parent':
                return

            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT"
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
        if self.with_prints:
            print "states_after: ", model, prop_name, info

        if self.busy or info.method_name == 'state_change' and \
                info.kwargs.prop_name == 'state' and \
                info.kwargs.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        else:
            # logger.debug("History states_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = NotificationOverview(info, with_prints=self.with_prints)
            # handle interrupts of action caused by exceptions
            if overview['result'][-1] == "CRASH in FUNCTION" or isinstance(overview['result'][-1], Exception):
                if self.with_prints:
                    logger.warning("function crash detected states_after")
                return self._interrupt_actual_action()

            # modifications of parent are not observed
            if overview['method_name'][0] == 'state_change' and \
                    overview['method_name'][-1] in ['active', 'child_execution', 'state_execution_status'] or \
                    not overview['method_name'][0] == 'state_change' or \
                    overview['method_name'][-1] == 'parent':
                return

            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    if self.with_prints:
                        print "HISTORY COUNT WAS OF SUCCESS {}".format(overview)
            else:
                logger.error("HISTORY after not count [states] -> For every before there should be a after.")


    @ModelMT.observe("state", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("output_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def assign_notification_root_state_before(self, model, prop_name, info):

        if self.with_prints:
            print "root_state_before: ", NotificationOverview(info), "\n"

        # execution_status-changes are not observed
        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        # first element should be prop_name="state_machine", instance=StateMachine and model=StateMachineModel
        # second element should be Prop_name="states" if root_state child elements are changed
        # --- for root_state elements it has to be prop_name in ["data_flows", "transitions", "input_data_ports",
        #                                                        "output_data_ports", "scoped_variables"]
        # third (and last element) should be prop_name in ["data_flow", "transition", ...
        else:
            overview = NotificationOverview(info, with_prints=self.with_prints)
            # logger.debug("History BEFORE {0}".format(overview))  # \n%s \n%s \n%s" % (model, prop_name, info))

            # modifications of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT"

                if self.start_new_action(overview):
                    self.before_count()
                else:
                    logger.error("FAILED to start NEW HISTORY ELEMENT [root_state]")

    @ModelMT.observe("state", after=True)
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
        if self.with_prints:
            print "root_state_after: ", NotificationOverview(info)

        # execution_status-changes are not observed
        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        else:
            overview = NotificationOverview(info, with_prints=self.with_prints)
            # logger.debug("History state_AFTER {0}".format(overview))

            # handle interrupts of action caused by exceptions
            if overview['result'][-1] == "CRASH in FUNCTION" or isinstance(overview['result'][-1], Exception):
                if self.with_prints:
                    logger.warning("function crash detected state_after")
                return self._interrupt_actual_action()

            # modifications of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            # decrease counter and finish action when reaching count=0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    if self.with_prints:
                        print "HISTORY COUNT WAS OF SUCCESS {}".format(overview)
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
    """The Class holds a all time history and a trail history. The trail history holds directly all modifications made since
    the last reset until the actual last active change and the undone modifications of this branch of modifications.
    So the all time history holds a list of all modifications ordered by time whereby the list elements are TreeElements
    that now respective previous action's list id and the possible next action list ids (multiple branches). Hereby a
    fast search from a actual active branch (trail history) to specific version_id (some branch) can be performed and
    all recovery steps collected.
    Additionally there will be implemented functionalities that never forget a single
    change that was insert for debugging reasons.
    - the pointer are pointing on the next undo ... so redo is pointer + 1
    - all_actions is a type of a tree # prev_id, action, next_id, old_next_ids
    """

    # TODO remove explizit trail-history -> next_id is holding the same information and old_next_ids the branching
    def __init__(self):
        Observable.__init__(self)
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None
        self.all_time_pointer = None
        self.counter = 0

        self.with_prints = False

        # insert initial dummy element
        self.insert_action(ActionDummy())

    @Observable.observed
    def insert_action(self, action):
        # insert new element in
        action.version_id = self.counter
        if self.counter == 0:
            prev_id = None
        else:
            prev_id = self.trail_history[self.trail_pointer].version_id

        self.all_time_history.append(HistoryTreeElement(prev_id=prev_id, action=action))
        self.counter += 1

        # set pointer of previous element
        if self.all_time_pointer is not None:
            prev_tree_elem = self.all_time_history[prev_id]
            prev_old_next_ids = copy.deepcopy(prev_tree_elem.old_next_ids)
            if self.with_prints:
                logger.info("new pointer {0} element {1}\nnew next_id {2}".format(self.all_time_history[self.trail_pointer].action.version_id,
                                                                                  prev_tree_elem,
                                                                                  len(self.all_time_history) - 1))
            prev_tree_elem.next_id = len(self.all_time_history) - 1
            if not prev_old_next_ids == prev_tree_elem.old_next_ids:
                logger.info("This action has been created a new branch in the state machine modification-history")

        # do single trail history
        if self.trail_pointer is not None:
            if self.trail_pointer > len(self.trail_history) - 1 or self.trail_pointer < 0:
                logger.error('History is broken may!!! %s' % self.trail_pointer)
            while not self.trail_pointer == len(self.trail_history) - 1:
                if self.with_prints:
                    print "pointer: %s %s" % (self.trail_pointer, len(self.trail_history))
                self.trail_history.pop()
        self.trail_history.append(action)

        self.trail_pointer = len(self.trail_history) - 1
        if self.trail_pointer == -1:
            self.trail_pointer = None
        self.all_time_pointer = len(self.all_time_history) - 1  # general should be equal to self.counter and version_id
        if self.with_prints and action is not None:
            logger.info("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    @Observable.observed
    def undo(self):
        # logger.debug("try undo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_pointer is not None and not self.trail_pointer < 0 and \
                self.trail_history[self.trail_pointer] is not None:
            self.trail_history[self.trail_pointer].undo()
            self.trail_pointer -= 1
            self.all_time_pointer -= 1
        elif self.trail_pointer is not None or self.trail_history[self.trail_pointer].action is None:
            logger.warning("No UNDO left over!!!")
        else:
            logger.error("History undo FAILURE")
        if self.with_prints:
            logger.info("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    @Observable.observed
    def redo(self):
        # logger.debug("try redo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_history is not None and self.trail_pointer + 1 < len(self.trail_history) and \
                self.trail_history[self.trail_pointer + 1] is not None:
            self.trail_history[self.trail_pointer + 1].redo()
            self.trail_pointer += 1
            self.all_time_pointer += 1
        elif self.trail_history is not None or self.trail_history[self.trail_pointer + 1] is None:
            logger.warning("No REDO left over!!!")
        else:
            logger.error("History redo FAILURE")
        if self.with_prints:
            logger.info("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

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
        if self.with_prints:
            logger.info("old trail: {0}".format(all_trail_action))
        while intermediate_version_id not in all_trail_action:
            path.insert(0, intermediate_version_id)
            intermediate_version_id = self.all_time_history[intermediate_version_id].prev_id
        # cut of not needed actions
        trail_index = all_trail_action.index(intermediate_version_id)
        self.trail_history = self.trail_history[:trail_index+1]
        if self.with_prints:
            logger.info("cut of trail: {0}".format([a.version_id for a in self.trail_history]))

        # append all actions of the path -> active actions of the branch
        for version_id in path:
            # set default next_id to active trail
            self.all_time_history[self.trail_history[-1].version_id].next_id = version_id
            self.trail_history.append(self.all_time_history[version_id].action)
        if self.with_prints:
            logger.info("new active trail: {0}".format([a.version_id for a in self.trail_history]))

        # adjust trail history point to new active id
        self.trail_pointer = len(self.trail_history) - 1

        # append all inactive actions of the branch
        while self.all_time_history[self.trail_history[-1].version_id].next_id:
            insert_version_id = self.all_time_history[self.trail_history[-1].version_id].next_id
            self.trail_history.append(self.all_time_history[insert_version_id].action)
        if self.with_prints:
            logger.info("new trail: {0} with trail_pointer: {1}".format([a.version_id for a in self.trail_history], self.trail_pointer))

    def undo_redo_list_from_actual_trail_history_to_version_id(self, version_id):
        """Perform fast search from actual active branch to specific version_id and collect all recovery steps.
        """
        all_trail_action = [a.version_id for a in self.single_trail_history() if a is not None]
        all_active_action = self.get_all_active_actions()
        undo_redo_list = []
        _undo_redo_list = []

        intermediate_version_id = version_id
        if self.with_prints:
            logger.info("\n\nactive_action: {0} in: {3}"
                        "\ntrail_actions: {1} in: {4}"
                        "\nversion_id   : {2}".format(all_active_action, all_trail_action, intermediate_version_id,
                                                      intermediate_version_id in all_active_action,
                                                      intermediate_version_id in all_trail_action
                                                      ))

        if intermediate_version_id not in all_trail_action:
            # get undo to come from version_id to trail_action
            while intermediate_version_id not in all_trail_action:
                _undo_redo_list.insert(0, (intermediate_version_id, 'redo'))
                intermediate_version_id = self.all_time_history[intermediate_version_id].prev_id
            intermediate_goal_version_id = intermediate_version_id
        else:
            intermediate_goal_version_id = version_id
        intermediate_version_id = self.trail_history[self.trail_pointer].version_id
        if self.with_prints:
            logger.info("\n\nactive_action: {0} in: {3}"
                        "\ntrail_actions: {1} in: {4}"
                        "\nversion_id   : {5} {2}".format(all_active_action, all_trail_action, intermediate_version_id,
                                                      intermediate_version_id in all_active_action,
                                                      intermediate_version_id in all_trail_action, intermediate_goal_version_id
                                                      ))
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
        self.all_time_pointer = None
        self.counter = 0

        # insert initial dummy element
        self.insert_action(ActionDummy())

