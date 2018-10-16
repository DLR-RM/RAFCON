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

from gi.repository import Gtk
from gi.repository import GObject
from gtkmvc3 import View
from rafcon.gui.utils import constants

from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState


class StateIconView(View, Gtk.IconView):
    states = [("HS", HierarchyState), ("ES", ExecutionState),
              ("PS", PreemptiveConcurrencyState), ("BS", BarrierConcurrencyState)]

    def __init__(self):
        View.__init__(self)
        GObject.GObject.__init__(self)

        self.set_columns(len(self.states))
        self.set_margin(0)
        self.set_spacing(0)
        self.set_row_spacing(0)
        self.set_column_spacing(0)

        liststore = Gtk.ListStore(str, str)
        self.set_model(liststore)
        self.set_markup_column(0)
        self.set_tooltip_column(1)

        for shorthand, state_class in self.states:
            liststore.append(['<span font_desc="%s %s">&#x%s; ' % (constants.ICON_FONT, constants.FONT_SIZE_NORMAL,
                                                                   constants.BUTTON_ADD) + shorthand + '</span>',
                              "Add/Drag and Drop " + state_class.__name__])

        self['state_icon_view'] = self
        self.top = 'state_icon_view'
