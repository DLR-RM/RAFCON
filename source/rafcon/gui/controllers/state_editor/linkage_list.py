# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

from rafcon.gui.controllers.utils.tree_view_controller import ListViewController
from rafcon.gui.utils.notification_overview import NotificationOverview

from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log

logger = log.get_logger(__name__)


class LinkageListController(ListViewController):
    no_update = True  # used to reduce the update cost of the widget (e.g while no focus or complex changes)
    no_update_state_destruction = True
    no_update_self_or_parent_state_destruction = True
    _model_observed = []
    CORE_ELEMENT_CLASS = None

    def __init__(self, model, view, tree_view, list_store, logger):
        self.no_update = False  # used to reduce the update cost of the widget (e.g while no focus or complex changes)
        self.no_update_state_destruction = False
        self.no_update_self_or_parent_state_destruction = False
        self._model_observed = []
        super(LinkageListController, self).__init__(model, view, tree_view, list_store, logger)
        self._model_observed.append(self.model)
        self.register_models_to_observe()

    def register_view(self, view):
        """Called when the View was registered
        """
        super(LinkageListController, self).register_view(view)

    def register_models_to_observe(self):

        model_to_observe = []
        state_m_4_get_sm_m_from = self.model
        if not self.model.state.is_root_state:
            # add self model to observe
            if self.model.get_state_machine_m(two_factor_check=False) is not None and \
                    self.model.state.state_id in self.model.parent.states:
                model_to_observe.append(self.model.parent.states[self.model.state.state_id])
                if self.model.parent.states[self.model.state.state_id] is not self.model:
                    self.model = self.model.parent.states[self.model.state.state_id]
                # add parent model to observe
                model_to_observe.append(self.model.parent)
                state_m_4_get_sm_m_from = self.model.parent
            else:
                logger.warning("State model has no state machine model as expected -> state model: {0}".format(self.model))

            # TODO maybe reduce the observation by this and add the check for sibling- and child-states
            if not self.model.parent.state.is_root_state:
                model_to_observe.append(self.model.parent.parent)
                state_m_4_get_sm_m_from = self.model.parent.parent
        else:
            if self.model.get_state_machine_m(two_factor_check=False) is not None:
                model_to_observe.append(self.model.get_state_machine_m(two_factor_check=False).root_state)
            else:
                logger.warning("State model has no state machine model as expected -> state model: {0}".format(self.model))

        # observe state machine model
        two_factor_check = False if state_m_4_get_sm_m_from.state.is_root_state else True
        if state_m_4_get_sm_m_from.get_state_machine_m(two_factor_check) is not None:
            model_to_observe.append(state_m_4_get_sm_m_from.get_state_machine_m(two_factor_check))
        else:
            logger.warning("State model has no state machine model as expected -> state model: {0}".format(self.model))

        [self.relieve_model(model) for model in self._model_observed if model not in model_to_observe]
        [self.observe_model(model) for model in model_to_observe if model not in self._model_observed]
        self._model_observed = model_to_observe

    def check_info_on_no_update_flags(self, info):
        """Stop updates while multi-actions"""
        # TODO that could need a second clean up
        # avoid updates because of state destruction
        if 'before' in info and info['method_name'] == "remove_state":
            if info.instance is self.model.state:
                self.no_update_state_destruction = True
            else:
                # if the state it self is removed lock the widget to never run updates and relieve all models
                removed_state_id = info.args[1] if len(info.args) > 1 else info.kwargs['state_id']
                if removed_state_id == self.model.state.state_id or \
                        not self.model.state.is_root_state and removed_state_id == self.model.parent.state.state_id:
                    self.no_update_self_or_parent_state_destruction = True
                    self.relieve_all_models()

        elif 'after' in info and info['method_name'] == "remove_state":
            if info.instance.state_id == self.model.state.state_id:
                self.no_update_state_destruction = False

        # reduce NotificationOverview generations by the fact that after could cause False and before could cause True
        if not self.no_update_state_destruction and not self.no_update_self_or_parent_state_destruction and \
                (not self.no_update and 'before' in info or 'after' in info and self.no_update):
            return
        overview = NotificationOverview(info)

        # The method causing the change raised an exception, thus nothing was changed and updates are allowed
        if 'after' in info and isinstance(overview.get_result(), Exception):
            self.no_update = False
            self.no_update_state_destruction = False
            # self.no_update_self_or_parent_state_destruction = False
            return

        if overview.get_cause() in ['group_states', 'ungroup_state', "change_state_type",
                                           "change_root_state_type"]:
            instance_is_self = self.model.state is overview.get_affected_core_element()
            instance_is_parent = self.model.parent and self.model.parent.state is overview.get_affected_core_element()
            instance_is_parent_parent = self.model.parent and self.model.parent.parent and self.model.parent.parent.state is overview.get_affected_core_element()
            # print("no update flag: ", True if 'before' in info and (instance_is_self or instance_is_parent or instance_is_parent_parent) else False)
            if instance_is_self or instance_is_parent or instance_is_parent_parent:
                self.no_update = True if 'before' in info else False

            if overview.get_affected_property() == 'state' and overview.get_cause() in ["change_state_type"] and \
                    self.model.get_state_machine_m() is not None:
                changed_model = self.model.get_state_machine_m().get_state_model_by_path(overview.get_method_args()[1].get_path())
                if changed_model not in self._model_observed:
                    self.observe_model(changed_model)

    def check_no_update_flags_and_return_combined_flag(self, prop_name, info):
        # avoid updates because of execution status updates
        overview = NotificationOverview(info)
        if not overview.caused_modification():
            return

        self.check_info_on_no_update_flags(info)

        # avoid updates while remove or multi-actions
        if self.no_update or self.no_update_state_destruction or self.no_update_self_or_parent_state_destruction:
            return True

    @ListViewController.observe("action_signal", signal=True)
    def notification_state_type_changed(self, model, prop_name, info):
        msg = info['arg']
        # print(self.__class__.__name__, "state_type_changed check", info)
        if msg.action in ['change_state_type', 'change_root_state_type', 'group_states', 'ungroup_state'] and msg.after:
            # print(self.__class__.__name__, msg.action)
            if model not in self._model_observed:
                self.relieve_model(model)
            self.register_models_to_observe()

        # TODO think about to remove this -> this a work around for the recreate state-editor assert __observer_threads
        if msg.action in ['change_state_type', 'change_root_state_type'] and not msg.after:
            self.relieve_all_models()

    @ListViewController.observe("state_machine", before=True)
    def before_notification_state_machine_observation_control(self, model, prop_name, info):
        """Check for multi-actions and set respective no update flags. """
        overview = NotificationOverview(info)
        if not overview.caused_modification():
            return
        # do not update while multi-actions
        self.check_info_on_no_update_flags(info)

    def store_debug_log_file(self, string):
        with open('{1}/{0}_debug_log_file.txt'.format(self.__class__.__name__, RAFCON_TEMP_PATH_BASE), 'a+') as f:
            f.write(string)
