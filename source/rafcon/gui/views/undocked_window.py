# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
from gi.repository import Gtk
from gi.repository import Gdk
from gtkmvc3.view import View

from rafcon.gui import glade
from rafcon.gui.utils import constants
from rafcon.gui.helpers import label


class UndockedWindowView(View):
    builder = glade.get_glade_path("undocked_window.glade")
    top = 'undock_window'

    def __init__(self, title):
        View.__init__(self)

        toolbar = Gtk.Toolbar()
        toolbar.props.show_arrow = False
        fullscreen_icon = label.create_button_label(constants.BUTTON_EXP)
        self['maximize_button'] = Gtk.ToolButton()
        self['maximize_button'].set_icon_widget(fullscreen_icon)
        redock_icon = label.create_button_label(constants.BUTTON_UNDOCK)
        self['redock_button'] = Gtk.ToolButton()
        self['redock_button'].set_icon_widget(redock_icon)
        self['redock_button'].set_tooltip_text("Redock")
        toolbar.insert(self['maximize_button'], 0)
        toolbar.insert(self['redock_button'], 1)

        self['headerbar'].props.title = title
        self['headerbar'].pack_end(toolbar)
        self['headerbar'].show_all()

        self.get_top_widget().set_titlebar(self['headerbar'])

    def initialize_title(self, window_title):
        """Initialize the title of the un-docked window

        :param window_title: The title of the window
        """
        self['headerbar'].props.title = window_title

    def reset_title(self, title, notebook_identifier):
        """Triggered whenever a notebook tab is switched in the left bar.

        Resets the title of the un-docked window to the format 'upper_open_tab / lower_open_tab'

        :param title: The name of the newly selected tab
        :param notebook: string taking one of two values 'upper' or 'lower' indicating which notebook was changed
        """
        current_title = self.get_top_widget().get_title()
        upper_title = current_title.split('/')[0].strip()
        lower_title = current_title.split('/')[1].strip()
        if notebook_identifier == 'upper':
            new_title = title + ' / ' + lower_title
        else:
            new_title = upper_title + ' / ' + title
        self['headerbar'].props.title = new_title
