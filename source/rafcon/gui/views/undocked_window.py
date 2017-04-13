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
import gtk
from gtkmvc import View

from rafcon.gui import glade
from rafcon.gui.views.top_tool_bar import TopToolBarView


class UndockedWindowView(View):
    builder = glade.get_glade_path("undocked_window.glade")
    top = 'undock_window'

    def __init__(self, title):
        View.__init__(self)

        self.top_tool_bar = TopToolBarView()
        self['top_menu_hbox'].remove(self['top_tool_bar_placeholder'])
        self['top_menu_hbox'].pack_end(self.top_tool_bar.get_top_widget(), expand=True, fill=True, padding=0)
        self['top_menu_hbox'].reorder_child(self.top_tool_bar.get_top_widget(), 1)

        self.get_top_widget().set_decorated(False)
        if not ('3.0.101' in os.uname()[2] and 'kde' == os.environ.get('DESKTOP_SESSION')):
            self.get_top_widget().set_type_hint(gtk.gdk.WINDOW_TYPE_HINT_UTILITY)

    def initialize_title(self, window_title):
        """Initialize the title of the un-docked window

        :param window_title: The title of the window
        """
        self.get_top_widget().set_title(window_title)

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
        self.get_top_widget().set_title(new_title)
