# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from gtkmvc import View

from rafcon.gui import glade
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class TopToolBarView(View):
    builder = glade.get_glade_path("top_tool_bar.glade")
    top = 'top_toolbar'

    def __init__(self):
        View.__init__(self)

        self.get_top_widget().set_events(gtk.gdk.POINTER_MOTION_MASK | gtk.gdk.POINTER_MOTION_HINT_MASK |
                                         gtk.gdk.BUTTON_PRESS_MASK)

        close_label = gui_helper_label.create_button_label(constants.BUTTON_CLOSE)
        self['close_button'].set_label_widget(close_label)
        self['close_button'].set_tooltip_text("Close RAFCON")

        maximize_label = gui_helper_label.create_button_label(constants.BUTTON_EXP)
        self['maximize_button'].set_label_widget(maximize_label)

        self['minimize_button'].set_label('_')
        self['minimize_button'].set_tooltip_text("Minimize main window")

        redock_label = gui_helper_label.create_button_label(constants.BUTTON_UNDOCK)
        self['redock_button'].set_label_widget(redock_label)
        self['redock_button'].set_tooltip_text("Re-Dock widget to main window")
