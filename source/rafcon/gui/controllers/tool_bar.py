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
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: tool_bar
   :synopsis: A module that holds the tool bar controller with respective functionalities or links for each button.

"""

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui import singleton as gui_singletons
from rafcon.utils import log

logger = log.get_logger(__name__)


class ToolBarController(ExtendedController):
    """The class to trigger all the action, available in the tool bar.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param view:
    """

    def __init__(self, state_machine_manager_model, view):
        ExtendedController.__init__(self, state_machine_manager_model, view)
        self.menu_bar_ctrl = gui_singletons.main_window_controller.get_controller('menu_bar_controller')
        self.shortcut_manager = None

    def register_view(self, view):
        """Called when the View was registered"""
        super(ToolBarController, self).register_view(view)
        self.view['button_new'].connect('clicked', self.on_button_new_clicked)
        self.view['button_open'].connect('clicked', self.on_button_open_clicked)
        self.view['button_save'].connect('clicked', self.on_button_save_clicked)
        self.view['button_refresh'].connect('clicked', self.on_button_refresh_clicked)
        self.view['button_refresh_selected'].connect('clicked', self.on_button_refresh_selected_clicked)
        self.view['button_refresh_libs'].connect('clicked', self.on_button_refresh_libs_clicked)
        self.view['button_bake_state_machine'].connect('clicked', self.on_button_bake_state_machine_clicked)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager:
        """
        pass

    def on_button_new_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_new_activate(widget, data)

    def on_button_open_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_open_activate(widget, data)

    def on_button_save_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_save_activate(widget, data)

    def on_button_refresh_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_refresh_all_activate(widget, data)

    def on_button_refresh_selected_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_refresh_selected_activate(widget, data)

    def on_button_refresh_libs_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_refresh_libraries_activate()

    def on_button_bake_state_machine_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_bake_state_machine_activate(widget, data)
