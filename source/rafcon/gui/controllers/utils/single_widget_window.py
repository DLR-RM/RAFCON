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

"""
.. module:: single_widget_window
   :synopsis: The module provides a template controller for single widget tests.

"""

from gi.repository import Gtk

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.shortcut_manager import ShortcutManager


class SingleWidgetWindowController(ExtendedController):
    """Controller handling the view of properties/attributes of ...
    """

    def __init__(self, model, view, ctrl_class, *args, **kwargs):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.shortcut_manager = None
        self.add_controller('widget_ctrl', ctrl_class(model, view.get_parent_widget(), *args, **kwargs))

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        super(SingleWidgetWindowController, self).register_view(view)
        self.shortcut_manager = ShortcutManager(self.view['main_window'])
        self.register_actions(self.shortcut_manager)

        view['main_window'].connect('destroy', Gtk.main_quit)
