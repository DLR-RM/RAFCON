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
from rafcon.gui.utils import constants

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel

# noinspection PyUnresolvedReferences
from rafcon.gui.mygaphas import guide
import rafcon.gui.singleton

from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionTickerController(ExtendedController):
    """Controller handling the execution ticker for all active state machines
    
     The class shows the last execution update in a text field. Currently this is only one state and one active state
     machine.

    :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param rafcon.gui.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """
    # TODO In future there will be multiple active state machines which have to be taken into account
    # TODO Currently concurrent states are also not taken into account what could be an extension or other feature

    def __init__(self, model, view):
        """Constructor"""
        assert isinstance(model, StateMachineManagerModel)
        ExtendedController.__init__(self, model, view)
        self.observe_model(rafcon.gui.singleton.gui_config_model)
        self.current_observed_sm_m = None
        self.update_disabled = False
        self.state_machine_execution_model = rafcon.gui.singleton.state_machine_execution_model
        self.observe_model(self.state_machine_execution_model)

    def register_view(self, view):
        pass  # not used yet because the view is still integrated in the main window view

    # TODO remove properties after own view was separated
    @property
    def ticker_text_label(self):
        return rafcon.gui.singleton.main_window_controller.view["execution_ticker_text"]

    @property
    def ticker_label(self):
        return rafcon.gui.singleton.main_window_controller.view["execution_ticker_label"]

    @property
    def ticker_hbox(self):
        return rafcon.gui.singleton.main_window_controller.view["execution_ticker_hbox"]

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
        if rafcon.gui.singleton.global_gui_config.get_config_value("EXECUTION_TICKER_ENABLED"):
            self.enabled()
        else:
            self.disabled()

    def disabled(self):
        self.ticker_hbox.hide()
        self.update_disabled = True

    def enabled(self):
        self.ticker_hbox.show()
        self.ticker_label.show()
        self.ticker_text_label.show()
        self.update_disabled = False

    @ExtendedController.observe("state_machine", after=True)
    def on_state_execution_status_changed_after(self, model, prop_name, info):
        """
        This function specifies what happens if the state machine execution status of a state changes
        :param observable: the state whose execution status changed
        :param return_value: the new execution status
        :param args: a list of all arguments of the observed function
        :return:
        """
        from rafcon.gui.utils.notification_overview import NotificationOverview
        from rafcon.core.states.state import State

        if self.update_disabled:
            return

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
            if overview['method_name'][-1] == 'state_execution_status':
                observable = overview['model'][-1].state
                assert isinstance(observable, State)

                path_depth = rafcon.gui.singleton.global_gui_config.get_config_value("EXECUTION_TICKER_PATH_DEPTH")
                message = create_path(observable, path_depth)
                if rafcon.gui.singleton.main_window_controller.view is not None:
                    # TODO how to align always left
                    # self.execution_ticker_text.set_halign(Gtk.Align.START)
                    self.ticker_text_label.set_text(message)
                else:
                    logger.warn("Not initialized yet")

    @ExtendedController.observe("execution_engine", after=True)
    def execution_engine_model_changed(self, model, prop_name, info):
        """High light active state machine. """

        active_state_machine_id = self.model.state_machine_manager.active_state_machine_id
        if active_state_machine_id is None:
            # relieve all state machines that have no active execution
            if self.current_observed_sm_m:
                self.relieve_model(self.current_observed_sm_m)
                self.ticker_text_label.set_text('None')
                self.current_observed_sm_m = None
            else:
                logger.info("Execution engine notification without active state machine {0}".format(info))
        else:
            # observe all state machines that have no active execution
            self.current_observed_sm_m = self.model.state_machines[active_state_machine_id]
            self.observe_model(self.current_observed_sm_m)
