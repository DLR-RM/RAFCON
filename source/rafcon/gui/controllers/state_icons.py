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

    def register_view(self, view):
        super(StateIconController, self).register_view(view)
        # GTK4: pointer events come from event controllers/gestures
        release_gesture = Gtk.GestureClick()
        release_gesture.connect("released", self.on_mouse_click)
        self.view.add_controller(release_gesture)
        motion_controller = Gtk.EventControllerMotion()
        motion_controller.connect("motion", self.on_mouse_motion)
        self.view.add_controller(motion_controller)

        # Drag & drop of new states onto the graphical editor: the editor's drop target
        # performs the insertion; the state to insert is handed over via the payload registry
        drag_source = Gtk.DragSource.new()
        drag_source.set_actions(Gdk.DragAction.COPY)
        drag_source.connect("prepare", self._on_drag_prepare)
        drag_source.connect("drag-end", self._on_drag_end)
        drag_source.connect("drag-cancel", self._on_drag_cancel)
        self.view.add_controller(drag_source)

    def _on_drag_prepare(self, drag_source, x, y):
        from rafcon.gui.utils import dnd
        path = self.view.get_path_at_pos(int(x), int(y))
        if path is None:
            return None
        self.view.select_path(path)
        dnd.set_drag_payload_provider(self._get_state)
        return Gdk.ContentProvider.new_for_value("rafcon-new-state")

    def _on_drag_end(self, drag_source, drag, delete_data):
        from rafcon.gui.utils import dnd
        dnd.clear_drag_payload_provider()
        self.view.unselect_all()

    def _on_drag_cancel(self, drag_source, drag, reason):
        from rafcon.gui.utils import dnd
        dnd.clear_drag_payload_provider()
        return False

    def on_mouse_click(self, gesture, n_press, x, y):
        """state insertion on mouse click

        :param Gtk.GestureClick gesture: click gesture of the icon view
        """
        import rafcon.gui.helpers.state_machine as gui_helper_state_machine
        if self.view.get_path_at_pos(int(x), int(y)) is not None \
                and len(self.view.get_selected_items()) > 0:
            return gui_helper_state_machine.insert_state_into_selected_state(self._get_state(), False)

    def on_mouse_motion(self, motion_controller, x, y):
        """selection on mouse over

        :param Gtk.EventControllerMotion motion_controller: motion controller of the icon view
        """
        path = self.view.get_path_at_pos(int(x), int(y))
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
