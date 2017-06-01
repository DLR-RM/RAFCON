# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from gtkmvc import View
from rafcon.gui.utils import constants


class StateIconView(View, gtk.IconView):
    icon_label = ["HS", "ES", "PS", "BS"]
    tooltips = {"HS": "Hierarchy State", "ES": "Execution State",
                "PS": "Preemptive Concurrency State", "BS": "Barrier Concurrency State"}

    def __init__(self):
        View.__init__(self)
        gtk.IconView.__init__(self)

        self.set_columns(len(self.icon_label))
        self.set_margin(0)
        self.set_spacing(0)
        self.set_row_spacing(0)
        self.set_column_spacing(0)

        liststore = gtk.ListStore(str, str)
        self.set_model(liststore)
        self.set_markup_column(0)
        self.set_tooltip_column(1)

        for state in self.icon_label:
            liststore.append(['<span font_desc="%s %s">&#x%s; ' % (constants.ICON_FONT, constants.FONT_SIZE_NORMAL,
                                                                   constants.BUTTON_ADD) + state + '</span>',
                              "Add/Drag and Drop " + self.tooltips[state]])

        self['state_icon_view'] = self
        self.top = 'state_icon_view'
