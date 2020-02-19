# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>

"""
.. module:: execution_ticker
   :synopsis: A module that holds the controller to the execution ticker of all active state machines.

"""

import rafcon.core.singleton

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.state_machine_execution_engine import StateMachineExecutionEngineModel

# noinspection PyUnresolvedReferences
from rafcon.gui.mygaphas import guide
import rafcon.gui.singleton

from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionTickerController(ExtendedController):
    """Controller handling the execution ticker for all active state machines
    
     The class shows the last execution update in a text field. Currently this is only one state and one active state
     machine.

    :param rafcon.gui.models.state_machine_execution_engine.StateMachineExecutionEngineModel model:
        The execution engine model to observe
    :param view: currently None
        element
    """
    # TODO In future there will be multiple active state machines which have to be taken into account
    # TODO Currently concurrent states are also not taken into account what could be an extension or other feature
    _fix_text_of_label = ", CURRENT STATE:  "

    def __init__(self, model, view):
        """Constructor"""
        assert isinstance(model, StateMachineExecutionEngineModel)
        ExtendedController.__init__(self, model, view)
        self.observe_model(rafcon.gui.singleton.gui_config_model)
        self.current_observed_sm_m = None
        self._view_initialized = True

    def _idle_register_view(self, view):
        pass  # work around to avoid traceback because of view=None

    def register_view(self, view):
        pass  # not used yet because the view is still integrated in the main window view

    def destroy(self):
        self.disable()
        self.relieve_all_models()
        self._view_initialized = False
        # super(ExecutionTickerController, self).destroy()

    # TODO remove properties after own view was separated
    @property
    def ticker_text_label(self):
        return rafcon.gui.singleton.main_window_controller.view["execution_ticker_text"]

    @ExtendedController.observe('config', after=True)
    def on_config_value_changed(self, config_m, prop_name, info):
        """Callback when a config value has been changed

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'config'
        :param dict info: Information e.g. about the changed config key
        """
        config_key = info['args'][1]

        if config_key in ["EXECUTION_TICKER_ENABLED"]:
            self.check_configuration()

    def check_configuration(self):
        if rafcon.gui.singleton.global_gui_config.get_config_value("EXECUTION_TICKER_ENABLED", True):
            self.enable()
        else:
            self.disable()

    def disable(self):
        """ Relieve all state machines that have no active execution and hide the widget """

        self.ticker_text_label.hide()
        if self.current_observed_sm_m:
            self.stop_sm_m_observation(self.current_observed_sm_m)

    def enable(self):
        """ Observe all state machines that have an active execution and show the widget """

        self.ticker_text_label.show()
        if self.current_observed_sm_m is None:
            self.start_active_sm_m_observation()

    @ExtendedController.observe("state_machine", after=True)
    def on_state_execution_status_changed_after(self, model, prop_name, info):
        """ Show current execution status in the widget

        This function specifies what happens if the state machine execution status of a state changes
        
        :param model: the model of the state that has changed (most likely its execution status)
        :param prop_name: property name that has been changed
        :param info: notification info dictionary
        :return:
        """
        from rafcon.gui.utils.notification_overview import NotificationOverview
        from rafcon.core.states.state import State

        def name_and_next_state(state):
            assert isinstance(state, State)
            if state.is_root_state_of_library:
                return state.parent.parent, state.parent.name
            else:
                return state.parent, state.name

        def create_path(state, n=3, separator='/'):
            next_parent, name = name_and_next_state(state)
            path = separator + name
            n -= 1
            while n > 0 and isinstance(next_parent, State):
                next_parent, name = name_and_next_state(next_parent)
                path = separator + name + path
                n -= 1
            if isinstance(next_parent, State):
                path = separator + '..' + path
            return path

        if 'kwargs' in info and 'method_name' in info['kwargs']:
            overview = NotificationOverview(info)
            if overview.get_cause() == 'state_execution_status':
                active_state = overview.get_affected_model().state
                assert isinstance(active_state, State)

                path_depth = rafcon.gui.singleton.global_gui_config.get_config_value("EXECUTION_TICKER_PATH_DEPTH", 3)

                message = self._fix_text_of_label + create_path(active_state, path_depth)
                if rafcon.gui.singleton.main_window_controller.view is not None:
                    self.ticker_text_label.set_text(message)
                else:
                    logger.warning("Not initialized yet")

    def stop_sm_m_observation(self, sm_m):
        self.relieve_model(sm_m)
        self.ticker_text_label.set_text(self._fix_text_of_label + 'None')
        self.current_observed_sm_m = None

    def start_active_sm_m_observation(self):
        active_sm_id = rafcon.gui.singleton.state_machine_manager_model.state_machine_manager.active_state_machine_id
        if active_sm_id:
            self.current_observed_sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[active_sm_id]
            self.observe_model(self.current_observed_sm_m)

    @ExtendedController.observe("execution_engine", after=True)
    def execution_engine_model_changed(self, model, prop_name, info):
        """Active observation of state machine and show and hide widget. """
        if not self._view_initialized:
            return

        active_sm_id = rafcon.gui.singleton.state_machine_manager_model.state_machine_manager.active_state_machine_id
        if active_sm_id is None:
            # relieve all state machines that have no active execution and hide the widget
            self.disable()
        else:
            # observe all state machines that have an active execution and show the widget
            self.check_configuration()
