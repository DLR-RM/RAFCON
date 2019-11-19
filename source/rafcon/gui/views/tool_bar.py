# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.view import View

from rafcon.gui import glade
from rafcon.gui.helpers.label import create_label_widget_with_icon
from rafcon.gui.utils import constants


class ToolBarView(View):
    builder = glade.get_glade_path("tool_bar.glade")
    top = 'toolbar'

    def __init__(self):
        View.__init__(self)

        button_new = self['button_new']
        button_new.set_label_widget(create_label_widget_with_icon(constants.BUTTON_NEW, _("New state machine")))

        button_refresh = self['button_refresh']
        button_refresh.set_label_widget(create_label_widget_with_icon(constants.BUTTON_REFR, _("Refresh"),
                                                                      "Refresh all libraries and state machines"))

        button_refresh_selected = self['button_refresh_selected']
        button_refresh_selected.set_label_widget(create_label_widget_with_icon(constants.ICON_REFRESH_SELECTED,
                                                                               _("Refresh Selected"),
                                                                               "Refresh selected state machine"))

        button_open = self['button_open']
        button_open.set_label_widget(create_label_widget_with_icon(constants.BUTTON_OPEN, _("Open state machine")))

        button_save = self['button_save']
        button_save.set_label_widget(create_label_widget_with_icon(constants.BUTTON_SAVE, _("Save state machine")))

        button_refresh_libs = self['button_refresh_libs']
        button_refresh_libs.set_label_widget(
            create_label_widget_with_icon(constants.ICON_REFRESH_LIBS, _("Refresh Libraries"), "Refresh all libraries"))

        button_bake_state_machine = self['button_bake_state_machine']
        button_bake_state_machine.set_label_widget(
            create_label_widget_with_icon(
                constants.BUTTON_BAKE, _("Bake State Machine"),
                "Saves the currently selected state machine and all library folders it refers to"))
        # self.get_top_widget().set_border_width(constants.BORDER_WIDTH)
