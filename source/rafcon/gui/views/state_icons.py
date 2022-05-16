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
from rafcon.design_patterns.mvc.view import View
from rafcon.gui.utils import constants
from rafcon.gui.config import global_gui_config

from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState


class StateIconView(View, Gtk.IconView):
    states = [("HS", HierarchyState, constants.ICON_STATE_HIERARCHY),
              ("ES", ExecutionState, constants.ICON_STATE_EXECUTION),
              ("PS", PreemptiveConcurrencyState, constants.ICON_STATE_CONC_PREEMPTIVE),
              ("BS", BarrierConcurrencyState, constants.ICON_STATE_CONS_BARRIER)]

    def __init__(self):
        View.__init__(self, parent='state_icon_view')
        Gtk.IconView.__init__(self)
        self.props.item_orientation = Gtk.Orientation.HORIZONTAL

        self.set_columns(len(self.states))
        self.set_margin(0)
        self.set_item_width(23)
        self.set_spacing(0)
        self.set_row_spacing(0)
        self.set_column_spacing(0)

        liststore = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING)
        self.set_model(liststore)
        self.set_markup_column(0)
        self.set_tooltip_column(1)

        for shorthand, state_class, icon in self.states:
            # use this line if the shorthand description of the state type should be included
            liststore.append(['<span font_desc="{font} {size}" color="{color}">{icon}</span>'.format(
                font=constants.ICON_FONT_RAFCON,
                size=constants.FONT_SIZE_HUGE,
                color=global_gui_config.colors['BUTTON_TEXT_COLOR'],
                icon=icon,
                text=shorthand
            ), "Add/Drag and Drop " + state_class.__name__])

        self['state_icon_view'] = self
