"""
.. module:: state_icons
   :platform: Unix, Windows
   :synopsis: A module that holds the state icon controller with its add-state and drag & drop functionalities.

.. moduleauthor:: Annika Wollschlaeger


"""

import gtk

import gui.helpers.state_machine as state_machine_helper
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
        view.drag_source_set(gtk.gdk.BUTTON1_MASK, [('STRING', 0, 0)], gtk.gdk.ACTION_COPY)

    def register_adapters(self):
        pass

    def register_view(self, view):
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
        state_id = self._insert_state()
        if state_id is not None:
            data.set_text(state_id)

    def on_drag_begin(self, widget, context):
        """replace drag icon

        :param widget:
        :param context:
        """
        self.view.drag_source_set_icon_stock(gtk.STOCK_NEW)

    def on_drag_end(self, widget, context):
        """if the drag is finished, all icons are unselected

        :param widget:
        :param context:
        """
        self.view.unselect_all()

    def on_mouse_click(self, widget, event):
        """state insertion on mouse click

        :param widget:
        :param gtk.gdk.Event event: mouse click event
        """
        if self.view.get_path_at_pos(int(event.x), int(event.y)) is not None \
                and len(self.view.get_selected_items()) > 0:
            self._insert_state()

    def on_mouse_motion(self, widget, event):
        """selection on mouse over

        :param widget:
        :param gtk.gdk.Event event: mouse motion event
        """
        path = self.view.get_path_at_pos(int(event.x), int(event.y))
        if path is not None:
            self.view.select_path(path)
        else:
            self.view.unselect_all()

    def _insert_state(self):
        """insert state

        :return: state ID
        :rtype: String
        """

        selected = self.view.get_selected_items()
        if not selected:
            return
        selected_path = self.view.states[selected[0][0]]

        if selected_path == "ES":
            state = ExecutionState()
        elif selected_path == "HS":
            state = HierarchyState()
        elif selected_path == "PS":
            state = PreemptiveConcurrencyState()
        elif selected_path == "BS":
            state = BarrierConcurrencyState()

        if state_machine_helper.insert_state(state, False):
            return state.state_id
