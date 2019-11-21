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
    MetaDataAction, get_state_element_meta, StateImage
from rafcon.gui.models.signals import ActionSignalMsg

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
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE, BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS

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
        act_state_elements_meta = self.tmp_meta_storage
        for path_elem in path_elements:
            act_state_elements_meta = act_state_elements_meta['states'][path_elem]
        return act_state_elements_meta

    def reset_to_history_id(self, target_history_id):
        """ Recovers a specific target_history_id of the _full_history element by doing several undos and redos.

        :param target_history_id: the id of the list element which is to recover
        :return:
        """
        with self.state_machine_model.storage_lock:
            self.busy = True
            self.modifications.go_to_history_element(target_history_id)
            self.busy = False

            self._re_initiate_observation()
            self.update_internal_tmp_storage()
            self.change_count += 1

    def undo(self):
        with self.state_machine_model.storage_lock:
            action = self.modifications.current_history_element.action
            self.busy = True
            self.modifications.undo()
            self.busy = False
            if isinstance(action, StateMachineAction):
                self._re_initiate_observation()
            self.update_internal_tmp_storage()
            self.change_count += 1

    def redo(self):
        with self.state_machine_model.storage_lock:
            action = None
            if self.modifications.get_next_element():
                action = self.modifications.get_next_element().action
            self.busy = True
            self.modifications.redo()
            self.busy = False
            if isinstance(action, StateMachineAction):
                self._re_initiate_observation()
            self.update_internal_tmp_storage()
            self.change_count += 1

    def _interrupt_active_action(self, info=None):
        logger.info("Active Action {} is interrupted and removed.".format(info['prop_name']))
        self.active_action.prepare_destruction()
        self.locked = False
        self.count_before = 0
        if self.state_machine_model.storage_lock.locked():
            self.state_machine_model.storage_lock.release()

    def _re_initiate_observation(self):
        self.relieve_model(self.__buffered_root_state_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

    def start_new_action(self, overview):
        if self.fake:
            self.active_action = ActionDummy()
            return True

        result = True
        cause = overview.get_cause()

        if self.refactored_history:
            if isinstance(overview.get_affected_core_element(), DataFlow) or \
                    isinstance(overview.get_affected_core_element(), Transition) or \
                    isinstance(overview.get_affected_core_element(), ScopedVariable):
                if isinstance(overview.get_affected_core_element(), DataFlow):
                    assert overview.get_affected_core_element() is overview.get_affected_model().data_flow
                    action_class = DataFlowAction
                elif isinstance(overview.get_affected_core_element(), Transition):
                    assert overview.get_affected_core_element() is overview.get_affected_model().transition
                    action_class = TransitionAction
                else:
                    assert overview.get_affected_core_element() is overview.get_affected_model().scoped_variable
                    action_class = ScopedVariableAction  # is a DataPort too
                self.active_action = action_class(parent_path=overview.get_affected_core_element().parent.get_path(),
                                                  state_machine_model=self.state_machine_model,
                                                  overview=overview)
            elif isinstance(overview.get_affected_core_element(), Outcome):
                assert overview.get_affected_core_element() is overview.get_affected_model().outcome
                self.active_action = OutcomeAction(parent_path=overview.get_affected_core_element().parent.get_path(),
                                                   state_machine_model=self.state_machine_model,
                                                   overview=overview)
            elif isinstance(overview.get_affected_core_element(), DataPort):
                if isinstance(overview.get_affected_core_element(), InputDataPort):
                    assert overview.get_affected_core_element() is overview.get_affected_model().data_port
                else:
                    assert overview.get_affected_core_element() is overview.get_affected_model().data_port
                self.active_action = DataPortAction(parent_path=overview.get_affected_core_element().parent.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            elif isinstance(overview.get_affected_core_element(), State):
                assert overview.get_affected_core_element() is overview.get_affected_model().state
                if "semantic_data" in overview.get_cause():
                    self.active_action = StateAction(parent_path=overview.get_affected_core_element().get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
                elif "add_" in cause:
                    self.active_action = AddObjectAction(parent_path=overview.get_affected_core_element().get_path(),
                                                         state_machine_model=self.state_machine_model,
                                                         overview=overview)
                elif "remove_" in cause:
                    assert cause in ["remove_transition", "remove_data_flow", "remove_income", "remove_outcome",
                                     "remove_input_data_port", "remove_output_data_port", "remove_scoped_variable",
                                     "remove_state"]
                    if ("transition" in cause or "data_flow" in cause or "scoped_variable" in cause or "state" in cause) or\
                            (("data_port" in cause or "outcome" in cause or "income" in cause) and not isinstance(overview.get_affected_model().state.parent, State)):
                        self.active_action = RemoveObjectAction(parent_path=overview.get_affected_core_element().get_path(),
                                                                state_machine_model=self.state_machine_model,
                                                                overview=overview)
                    elif "data_port" in cause or "outcome" in cause or "income" in cause:

                        if isinstance(overview.get_affected_core_element().parent, State):
                            self.active_action = RemoveObjectAction(parent_path=overview.get_affected_core_element().parent.get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                        else:
                            self.active_action = RemoveObjectAction(parent_path=overview.get_affected_core_element().get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                    else:
                        logger.warning("un foreseen cause: {0} in remove state element".format(cause))
                        assert False
                else:
                    self.active_action = StateAction(parent_path=overview.get_affected_core_element().get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
            elif isinstance(overview.get_affected_core_element(), StateMachine):
                assert overview.get_affected_core_element() is overview.get_affected_model().state_machine
                assert False  # should never happen
            else:  # FAILURE
                logger.warning("History may need update, tried to start observation of new action that is not "
                               "classifiable: \n{}".format(str(overview)))
                assert False  # should never happen

        else:
            result = self.start_new_action_old(overview)

        return result

    def start_new_action_old(self, overview):

        result = True

        if isinstance(overview.get_affected_core_element(), DataFlow) or \
                isinstance(overview.get_affected_core_element(), Transition) or \
                isinstance(overview.get_affected_core_element(), ScopedVariable):  # internal modifications No Add or Remove Actions
            # the model should be StateModel or ContainerStateModel and "info" from those model notification
            self.active_action = Action(parent_path=overview.get_affected_core_element().parent.get_path(),
                                        state_machine_model=self.state_machine_model,
                                        overview=overview)

        elif overview.get_affected_model().parent and (isinstance(overview.get_affected_core_element(), DataPort) or
                                               isinstance(overview.get_affected_core_element(), Outcome) or
                                               overview.get_cause() in ['add_outcome', 'remove_outcome',
                                                                               'add_output_data_port',
                                                                               'remove_output_data_port',
                                                                               'add_input_data_port',
                                                                               'remove_input_data_port']):

            if overview.get_affected_model().parent:
                if not isinstance(overview.get_affected_model().parent.state, State):
                    level_status = 'State'
                    self.active_action = Action(parent_path=overview.get_affected_core_element().get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                elif not isinstance(overview.get_affected_model().parent.state.parent, State):  # is root_state
                    level_status = 'ParentState'
                    self.active_action = Action(parent_path=overview.get_affected_core_element().parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                else:
                    level_status = 'ParentParentState'
                    self.active_action = Action(parent_path=overview.get_affected_core_element().parent.parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
            else:
                assert False

        elif overview.get_affected_property() == 'state':
            if "add_" in overview.get_cause():
                self.active_action = Action(parent_path=overview.get_affected_core_element().get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)
            else:
                self.active_action = Action(parent_path=overview.get_affected_core_element().get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)

        else:  # FAILURE
            logger.warning("History may need update, tried to start observation of new action that is not"
                           "classifiable: \n{}".format(str(overview)))
            return False

        return result

    def finish_new_action(self, overview):
        if isinstance(self.active_action, MetaDataAction) and self.check_gaphas_consistency:
            check_gaphas_state_machine_meta_data_consistency(self.state_machine_model, with_logger_messages=True)

        try:
            self.active_action.set_after(overview)
            self.state_machine_model.history.modifications.insert_action(self.active_action)
            self.update_internal_tmp_storage()
        except:
            logger.exception("Failure occurred while finishing action")
            raise

        self.change_count += 1

    def re_initiate_meta_data(self):
        self.active_action = []
        self.update_internal_tmp_storage()

    @ModelMT.observe("state_meta_signal", signal=True)  # meta data of root_state_model changed
    def meta_changed_notify_after(self, changed_model, prop_name, info):
        if not self.with_meta_data_actions:
            return
        overview = NotificationOverview(info)
        if self.busy:
            return
        if overview.get_signal_message().origin == 'load_meta_data':
            return

        if self.active_action is None or overview.get_signal_message().change in ['append_initial_change']:
            # update last actions after_state_image -> meta-data
            self.re_initiate_meta_data()
        elif self.active_action is None or \
                overview.get_signal_message().change in ['append_to_last_change'] or \
                overview.get_signal_message().origin in ['group_states', 'ungroup_state', 'substitute_state']:
            # update last actions after_state_image -> meta-data
            self.active_action.after_state_image = self.active_action.get_state_image()
            self.update_internal_tmp_storage()
        else:
            if isinstance(overview.get_affected_model(), AbstractStateModel):
                changed_parent_model = overview.get_affected_model()
            else:
                changed_parent_model = overview.get_affected_model().parent
            changed_parent_state_path = changed_parent_model.state.get_path()

            # TODO think about to remove this work around again
            # ignore meta data changes inside of library states
            changed_parent_state = self.state_machine_model.get_state_model_by_path(changed_parent_state_path).state
            if changed_parent_state.get_next_upper_library_root_state():
                return

            if self.count_before == 0:
                self.active_action = MetaDataAction(changed_parent_model.state.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
                meta_data = self.get_state_element_meta_from_internal_tmp_storage(changed_parent_state_path)
                state_image = StateImage(meta_data=meta_data)
                self.active_action.before_state_image = state_image

                self.finish_new_action(overview)
            else:
                logger.info('Meta data change signal was emitted while other action was is performed. \n{0}'.format(overview))

    def before_count(self):
        if self.count_before == 0:
            self.state_machine_model.storage_lock.acquire()
            self.locked = True
        self.count_before += 1

    def after_count(self):
        self.count_before -= 1
        if self.count_before == 0:
            self.locked = False
            self.state_machine_model.storage_lock.release()

    @ModelMT.observe("state_action_signal", signal=True)
    def state_action_signal(self, model, prop_name, info):
        if self.busy:  # if proceeding undo or redo
            return
        if isinstance(model, StateMachineModel) and isinstance(info['arg'], ActionSignalMsg) and \
                not info['arg'].after and info['arg'].action in ['change_root_state_type', 'change_state_type',
                                                                 'paste', 'cut',
                                                                 'substitute_state', 'group_states', 'ungroup_state']:

            overview = NotificationOverview(info)

            if info['arg'].action == 'change_root_state_type':
                assert info['arg'].action_parent_m is self.state_machine_model
                self.active_action = StateMachineAction(parent_path=info['arg'].action_parent_m.root_state.state.get_path(),
                                                        state_machine_model=info['arg'].action_parent_m,
                                                        overview=overview)
            else:
                self.active_action = StateAction(parent_path=info['arg'].action_parent_m.state.get_path(),
                                                 state_machine_model=self.state_machine_model,
                                                 overview=overview)

            self.before_count()
            if info['arg'].action in ['group_states', 'paste', 'cut']:
                self.observe_model(info['arg'].action_parent_m)
            else:
                self.observe_model(info['arg'].affected_models[0])

    @ModelMT.observe("action_signal", signal=True)
    def action_signal_after_complex_action(self, model, prop_name, info):
        if self.busy:  # if proceeding undo or redo
            return
        if isinstance(model, AbstractStateModel) and isinstance(info['arg'], ActionSignalMsg) and \
                info['arg'].after and info['arg'].action in ['change_root_state_type', 'change_state_type',
                                                             'paste', 'cut',
                                                             'substitute_state', 'group_states', 'ungroup_state']:

            overview = NotificationOverview(info)
            if info['arg'].action in ['change_state_type', 'paste', 'cut',
                                      'substitute_state', 'group_states', 'ungroup_state']:

                if self.__buffered_root_state_model is not model:
                    self.relieve_model(model)

            if isinstance(info['arg'].result, Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                else:
                    logger.warning("Count is wrong 1 != {0}".format(self.count_before))

            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
                    if info['arg'].action == 'change_root_state_type':
                        self._re_initiate_observation()
            else:
                logger.error("HISTORY after not count to 0 [action signal] -> For every before there should be a after."
                             "{0}".format(NotificationOverview(info)))

    @ModelMT.observe("states", before=True)
    def assign_notification_states_before(self, model, prop_name, info):
        if self.busy:  # if proceeding undo or redo
            return
        else:
            # avoid to vast computation time
            if 'kwargs' in info and 'method_name' in info['kwargs'] and \
                    info['kwargs']['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
                return

            overview = NotificationOverview(info)

            # skipped state modifications
            if not overview.get_change() == 'state_change' or overview.get_cause() == 'parent':
                return

            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
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
        if self.busy or info.method_name == 'state_change' and \
                info.kwargs.prop_name == 'state' and \
                info.kwargs.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        else:
            # avoid to vast computation time
            if 'kwargs' in info and 'method_name' in info['kwargs'] and \
                    info['kwargs']['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
                return

            overview = NotificationOverview(info)

            # handle interrupts of action caused by exceptions
            if overview.get_result() == "CRASH in FUNCTION" or isinstance(overview.get_result(), Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                pass

            # modifications of parent are not observed
            if not overview.get_change() == 'state_change' or overview.get_cause() == 'parent':
                return

            # decrease counter and finish action if count_before = 0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
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
        # execution_status-changes are not observed
        if self.busy or info.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        # first element should be prop_name="state_machine", instance=StateMachine and model=StateMachineModel
        # second element should be Prop_name="states" if root_state child elements are changed
        # --- for root_state elements it has to be prop_name in ["data_flows", "transitions", "input_data_ports",
        #                                                        "output_data_ports", "scoped_variables"]
        # third (and last element) should be prop_name in ["data_flow", "transition", ...
        else:
            overview = NotificationOverview(info)
            # modifications of parent are not observed
            if overview.get_cause() == 'parent':
                return

            # increase counter and generate new action if not locked by action that is performed
            if self.locked:
                self.before_count()
            else:
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
        # execution_status-changes are not observed
        if self.busy or info.method_name in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return
        else:
            overview = NotificationOverview(info)

            # handle interrupts of action caused by exceptions
            if overview.get_result() == "CRASH in FUNCTION" or isinstance(overview.get_result(), Exception):
                if self.count_before == 1:
                    return self._interrupt_active_action(info)
                pass

            # modifications of parent are not observed
            if overview.get_cause() == 'parent':
                return

            # decrease counter and finish action when reaching count=0
            if self.locked:
                self.after_count()
                if self.count_before == 0:
                    self.finish_new_action(overview)
            else:
                logger.error("HISTORY after not count [root_state] -> For every before there should be a after.")


class HistoryTreeElement(object):
    __history_id = None
    __prev_id = None
    __next_id = None

    def __init__(self, prev_id, action=None, next_id=None):
        if prev_id is not None:
            self.prev_id = prev_id
        self.action = action
        if next_id is not None:
            self.next_id = next_id
        self._old_next_ids = []

    def __str__(self):
        return "prev_id: {0} next_id: {1} and other next_ids: {2}".format(self.__prev_id, self.__next_id, self._old_next_ids)

    def prepare_destruction(self):
        self.action.prepare_destruction()

    @property
    def history_id(self):
        return self.__history_id

    @history_id.setter
    def history_id(self, value):
        if self.__history_id is None:
            self.__history_id = value
        else:
            raise ValueError("The history_id of an action is not allowed to be modify after first assignment")

    @property
    def prev_id(self):
        return self.__prev_id

    @prev_id.setter
    def prev_id(self, prev_id):
        # logger.info("new_prev_id is: {0}".format(prev_id))
        assert isinstance(prev_id, int)
        self.__prev_id = prev_id

    @property
    def next_id(self):
        return self.__next_id

    @next_id.setter
    def next_id(self, next_id):
        assert isinstance(next_id, int)
        # move current next_id to to old_next_ids before assigning a new one
        # => creates new branch
        if self.__next_id is not None:
            self._old_next_ids.append(self.__next_id)
        self.__next_id = next_id
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
    fast search from a actual active branch (trail history) to specific history_id (some branch) can be performed and
    all recovery steps collected.
    Additionally there will be implemented functionalities that never forget a single
    change that was insert for debugging reasons.
    - the pointer are pointing on the next undo ... so redo is pointer + 1
    - all_actions is a type of a tree # prev_id, action, next_id, old_next_ids
    """

    def __init__(self):
        Observable.__init__(self)
        self._full_history = []
        self._current_history_element = None

        # insert initial dummy element
        self.insert_action(ActionDummy())

    def __len__(self):
        return len(self.get_executed_history_ids())

    @property
    def current_history_element(self):
        return self._current_history_element

    def prepare_destruction(self):
        for tree_element in self._full_history:
            tree_element.prepare_destruction()
        del self._full_history[:]

    def is_undo_possible(self):
        return self.current_history_element.prev_id is not None

    def is_redo_possible(self):
        return self.current_history_element.next_id is not None

    def get_element_for_history_id(self, history_id):
        return self._full_history[history_id]

    def get_next_element(self, for_history_element=None):
        for_history_element = for_history_element or self.current_history_element
        if for_history_element and for_history_element.next_id is not None:
            return self._full_history[for_history_element.next_id]
        return None

    def get_previous_element(self, for_history_element=None):
        for_history_element = for_history_element or self.current_history_element
        if for_history_element and for_history_element.prev_id is not None:
            return self._full_history[for_history_element.prev_id]
        return None

    @Observable.observed
    def insert_action(self, action):

        prev_id = None if not self.current_history_element else self.current_history_element.history_id

        self._current_history_element = HistoryTreeElement(prev_id=prev_id, action=action)
        self._current_history_element.history_id = len(self._full_history)
        self._full_history.append(self.current_history_element)

        # set pointer of previous element
        if prev_id is not None:
            prev_tree_elem = self._full_history[prev_id]
            prev_old_next_ids = copy.deepcopy(prev_tree_elem.old_next_ids)
            prev_tree_elem.next_id = self._current_history_element.history_id
            if not prev_old_next_ids == prev_tree_elem.old_next_ids:
                logger.verbose("This action has created a new branch in the state machine modification-history")

    @Observable.observed
    def undo(self):
        if not self.is_undo_possible():
            logger.warning("There is no more action that can be undone")
            return
        history_element = self.current_history_element
        history_element.action.undo()
        self._current_history_element = self.get_previous_element()
        return history_element

    @Observable.observed
    def redo(self):
        if not self.is_redo_possible():
            logger.warning("There is no more action that can be redone")
            return
        self._current_history_element = self.get_next_element()
        history_element = self.current_history_element
        history_element.action.redo()
        return history_element

    def go_to_history_element(self, target_history_id):
        undo_history_ids, redo_history_ids, branching_element = self.get_history_path_from_current_to_target_history_id(target_history_id)
        for history_id in undo_history_ids:
            history_element = self.get_element_for_history_id(history_id)
            # must be set before undo (because only undo/redo are observable)
            self._current_history_element = self.get_previous_element(history_element)
            history_element.action.undo()
        for history_id in redo_history_ids:
            history_element = self.get_element_for_history_id(history_id)
            # must be set before redo (because only undo/redo are observable)
            self._current_history_element = history_element
            history_element.action.redo()
        if branching_element:
            branching_element.next_id = redo_history_ids[0]

    def get_executed_history_ids(self):
        executed_history_ids = []
        history_element = self.current_history_element
        while history_element.prev_id is not None:
            executed_history_ids.append(history_element.history_id)
            history_element = self.get_previous_element(history_element)
        executed_history_ids.append(history_element.history_id)
        return executed_history_ids

    def get_current_branch_history_ids(self):
        current_branch_history_ids = self.get_executed_history_ids()
        history_element = self.current_history_element
        while history_element.next_id is not None:
            history_element = self.get_next_element(history_element)
            current_branch_history_ids.append(history_element.history_id)
        return current_branch_history_ids

    def get_history_path_from_current_to_target_history_id(self, target_history_id):
        if not (0 <= target_history_id < len(self._full_history)):
            raise ValueError("target_history_id does not exist")
        undo_history_ids = []
        redo_history_ids = []
        executed_history_ids = self.get_executed_history_ids()
        target_element = self.get_element_for_history_id(target_history_id)
        pointer_element = target_element
        while pointer_element.history_id not in executed_history_ids:
            redo_history_ids.insert(0, pointer_element.history_id)
            pointer_element = self.get_previous_element(pointer_element)
        # element only becomes branching element, if there will be undo operations
        # => the target_history_id is in a different branch
        possible_branching_element = pointer_element
        while pointer_element is not self.current_history_element:
            pointer_element = self.get_next_element(pointer_element)
            undo_history_ids.insert(0, pointer_element.history_id)
        branching_element = None
        if redo_history_ids and undo_history_ids:
            branching_element = possible_branching_element
        return undo_history_ids, redo_history_ids, branching_element

    @Observable.observed
    def reset(self):
        self._full_history = []
        self._current_history_element = None

        # insert initial dummy element
        self.insert_action(ActionDummy())

