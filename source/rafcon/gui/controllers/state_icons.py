# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_icons
   :synopsis: A module that holds the state icon controller with its add-state and drag & drop functionalities.

"""

from gi.repository import Gtk
from gi.repository import Gdk

from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateIconController(ExtendedController):

    def __init__(self, model=None, view=None, shortcut_manager=None):
        ExtendedController.__init__(self, model, view)

        self.shortcut_manager = shortcut_manager

        view.drag_source_set(Gdk.ModifierType.BUTTON1_MASK, None, Gdk.DragAction.COPY)
        view.drag_source_add_text_targets()

    def register_view(self, view):
        super(StateIconController, self).register_view(view)
        self.view.connect("drag-data-get", self.on_drag_data_get)
        self.view.connect("drag-begin", self.on_drag_begin)
        self.view.connect("drag-end", self.on_drag_end)
        self.view.connect("button-release-event", self.on_mouse_click)
        self.view.connect("motion-notify-event", self.on_mouse_motion)

    def on_drag_data_get(self, widget, context, data, info, time):
        """dragged state is inserted and its state_id sent to the receiver

        :param widget:
        :param context:
        :param data: SelectionData: contains state_id
        :param info:
        :param time:
        """
        import rafcon.gui.helpers.state_machine as gui_helper_state_machine
        state = self._get_state()
        gui_helper_state_machine.add_state_by_drag_and_drop(state, data)

    def on_drag_begin(self, widget, context):
        """replace drag icon

        :param widget:
        :param context:
        """
        pass

    def on_drag_end(self, widget, context):
        """if the drag is finished, all icons are unselected

        :param widget:
        :param context:
        """
        self.view.unselect_all()

    def on_mouse_click(self, widget, event):
        """state insertion on mouse click

        :param widget:
        :param Gdk.Event event: mouse click event
        """
        import rafcon.gui.helpers.state_machine as gui_helper_state_machine
        if self.view.get_path_at_pos(int(event.x), int(event.y)) is not None \
                and len(self.view.get_selected_items()) > 0:
            return gui_helper_state_machine.insert_state_into_selected_state(self._get_state(), False)

    def on_mouse_motion(self, widget, event):
        """selection on mouse over

        :param widget:
        :param Gdk.Event event: mouse motion event
        """
        path = self.view.get_path_at_pos(int(event.x), int(event.y))
        if path is not None:
            self.view.select_path(path)
        else:
            self.view.unselect_all()

    def _get_state(self):
        """get state instance which was clicked on

        :return: State that represents the icon which was clicked on
        :rtype: rafcon.core.states.State
        """

        selected = self.view.get_selected_items()
        if not selected:
            return
        _, state_class, _ = self.view.states[selected[0][0]]
        return state_class()
