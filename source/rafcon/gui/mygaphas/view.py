# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from contextlib import contextmanager
from weakref import ref

from gi.repository import GObject

from gaphas.view import GtkView
from gaphas.view.gtkview import transform_rectangle
from gaphas.selection import Selection

from rafcon.design_patterns.observer.observer import Observer

from rafcon.gui.mygaphas.items.state import StateView
from rafcon.gui.mygaphas.utils.cache.value_cache import ValueCache


class RAFCONSelection(Selection):
    """Selection adapter bridging gaphas' selection protocol to RAFCON's selection model

    gaphas (view internals and painters) reads/writes selection state through this object;
    all calls are mapped onto the RAFCON state machine selection via the ExtendedGtkView.
    Only the hovered item is kept locally, as RAFCON has no notion of hovering.
    """

    def __init__(self, view):
        super(RAFCONSelection, self).__init__()
        self._view = ref(view)

    @property
    def selected_items(self):
        view = self._view()
        return view._get_selected_items() if view else set()

    def select_items(self, *items):
        view = self._view()
        if view:
            view.select_item(items)

    def unselect_item(self, item):
        view = self._view()
        if view:
            try:
                view.unselect_item(item)
            except (AttributeError, KeyError):
                pass  # model behind the item may already be gone during destruction

    def unselect_all(self):
        view = self._view()
        if view:
            view.unselect_all()

    @property
    def focused_item(self):
        view = self._view()
        return view._get_focused_item() if view else None

    @focused_item.setter
    def focused_item(self, item):
        view = self._view()
        if view:
            view._set_focused_item(item)

    def clear(self):
        # Called by gaphas when the model is unset (destruction); only reset local state,
        # the RAFCON selection model must not be altered here
        self._hovered_item = None
        self._focused_item = None


class ExtendedGtkView(GtkView, Observer):

    __gsignals__ = {
        'selection-changed': (GObject.SignalFlags.RUN_FIRST, None, (GObject.TYPE_PYOBJECT,)),
        'focus-changed': (GObject.SignalFlags.RUN_FIRST, None, (GObject.TYPE_PYOBJECT,)),
    }

    hovered_handle = None
    _selection_m = None

    def __init__(self, graphical_editor_v, state_machine_m):
        GtkView.__init__(self)
        Observer.__init__(self)
        self._selection_m = state_machine_m.selection
        self.value_cache = ValueCache()
        self.observe_model(self._selection_m)
        self.observe_model(state_machine_m.root_state)
        self._graphical_editor = ref(graphical_editor_v)
        # Replace the default gaphas selection with the adapter around RAFCON's selection model
        self._selection = RAFCONSelection(self)
        self._selection.add_handler(self.on_selection_update)

    def prepare_destruction(self):
        """Get rid of circular references"""
        self.relieve_model(self._selection_m)
        self._selection_m = None
        self.observable_to_methods.clear()
        self.model = None

    @property
    def graphical_editor(self):
        return self._graphical_editor()

    @property
    def canvas(self):
        """The canvas is the gaphas 5 model of this view"""
        return self._model

    @canvas.setter
    def canvas(self, canvas):
        self.model = canvas

    def update(self):
        """Update view status according to the items updated in the model

        Extends the base method with a synchronous fallback for contexts without a running
        (GLib-backed asyncio) event loop, e.g. simple test setups.
        """
        try:
            return super(ExtendedGtkView, self).update()
        except RuntimeError:
            model = self._model
            if not model:
                return None
            dirty_items = self.all_dirty_items()
            model.update_now(dirty_items)
            dirty_items |= self.all_dirty_items()
            old_bb = self._qtree.soft_bounds
            self.update_bounding_box(dirty_items)
            if self._qtree.soft_bounds != old_bb:
                self.update_scrolling()
            self.update_back_buffer()
            return None

    def get_items_in_rectangle(self, rect, contain=False, intersect=None, reverse=False):
        """Compatibility wrapper supporting the gaphas 2.x keyword arguments"""
        if intersect is not None:
            contain = not intersect
        items = list(super(ExtendedGtkView, self).get_items_in_rectangle(rect, contain=contain))
        if reverse:
            items.reverse()
        return items

    def queue_draw_item(self, *items):
        """Trigger a redraw; gaphas 5 always repaints the visible area"""
        self.update_back_buffer()

    def queue_draw_area(self, *args):
        """Trigger a redraw; gaphas 5 has no partial damage regions anymore"""
        self.update_back_buffer()

    def get_port_at_point(self, vpos, distance=10, exclude=None, exclude_port_fun=None):
        """
        Find item with port closest to specified position.

        List of items to be ignored can be specified with `exclude`
        parameter.

        Tuple is returned

        - found item
        - closest, connectable port
        - closest point on found port (in view coordinates)

        :Parameters:
         vpos
            Position specified in view coordinates.
         distance
            Max distance from point to a port (default 10)
         exclude
            Set of items to ignore.
        """
        v2i = self.get_matrix_v2i
        vx, vy = vpos

        max_dist = distance
        port = None
        glue_pos = None
        item = None

        rect = (vx - distance, vy - distance, distance * 2, distance * 2)
        items = self.get_items_in_rectangle(rect, reverse=True)
        for i in items:
            if exclude and i in exclude:
                continue
            for p in i.ports():
                if not p.connectable:
                    continue
                if exclude_port_fun and exclude_port_fun(p):
                    continue

                ix, iy = v2i(i).transform_point(vx, vy)
                pg, d = p.glue((ix, iy))
                if d > max_dist:
                    continue

                max_dist = d
                item = i
                port = p

                # transform coordinates from connectable item space to view space
                i2v = self.get_matrix_i2v(i).transform_point
                glue_pos = i2v(*pg)

        return item, port, glue_pos

    def get_state_at_point(self, vpos, distance=10):
        vx, vy = vpos
        rect = (vx - distance, vy - distance, distance * 2, distance * 2)
        items = self.get_items_in_rectangle(rect, reverse=True)
        for item in items:
            if isinstance(item, StateView):
                return item
        return None

    def get_zoom_factor(self):
        """Returns the current zoom factor of the view

        The zoom factor can be read out from the view's matrix. _matrix[0] should be equal _matrix[3]. Index 0 is for
        the zoom in x direction, index 3 for the y direction
        :return: Current zoom factor
        """
        return self._matrix[0]

    def get_items_at_point(self, pos, selected=True, distance=0):
        """ Return the items located at ``pos`` (x, y).

         :param bool selected: if False returns first non-selected item
         :param float distance: Maximum distance to be considered as "at point" (in viewport pixel)
        """
        if not self._model:
            return []
        vx, vy = pos
        rect = (vx - distance, vy - distance, 2 * distance, 2 * distance)
        # quadtree bounds are in canvas coordinates in gaphas 5
        crect = transform_rectangle(self._matrix.inverse(), rect)
        items = self._qtree.find_intersect(crect)
        filtered_items = []
        for item in reversed(list(self._model.sort(items))):
            if not selected and item in self.selected_items:
                continue  # skip selected items

            v2i = self.get_matrix_v2i(item)
            i2v = self.get_matrix_i2v(item)
            ix, iy = v2i.transform_point(*pos)
            distance_i = item.point(ix, iy)
            distance_v = i2v.transform_distance(distance_i, 0)[0]
            if distance_v <= distance:
                filtered_items.append(item)
        return filtered_items

    @Observer.observe("destruction_signal", signal=True)
    def _on_root_state_destruction(self, root_state_m, signal_name, signal_msg):
        """Ignore future selection changes when state machine is being destroyed"""
        self.relieve_model(root_state_m)

    @Observer.observe("selection_changed_signal", signal=True)
    def _on_selection_changed_externally(self, selection_m, signal_name, signal_msg):
        selected_items = self._get_selected_items()
        previously_selected_items = set(self.canvas.get_view_for_model(model) for model in signal_msg.arg.old_selection)
        affected_items = selected_items ^ previously_selected_items
        self.queue_draw_item(*affected_items)
        self.emit('selection-changed', selected_items)

    @contextmanager
    def _suppress_selection_events(self):
        self.relieve_model(self._selection_m)
        try:
            yield
        finally:
            self.observe_model(self._selection_m)

    def select_item(self, items):
        """ Select an items. This adds `items` to the set of selected items. """
        if not items:
            return
        elif not hasattr(items, "__iter__"):
            items = (items,)
        selection_changed = False
        with self._suppress_selection_events():
            for item in items:
                self.queue_draw_item(item)
                if item is not None and item.model not in self._selection_m:
                    self._selection_m.add(item.model)
                    selection_changed = True
        if selection_changed:
            self.emit('selection-changed', self._get_selected_items())

    def unselect_item(self, item):
        """ Unselect an item. """
        self.queue_draw_item(item)
        if item.model in self._selection_m:
            with self._suppress_selection_events():
                self._selection_m.remove(item.model)
            self.emit('selection-changed', self._get_selected_items())

    def unselect_all(self):
        """ Clearing the selected_item also clears the focused_item. """
        items = self._get_selected_items()
        with self._suppress_selection_events():
            self._selection_m.clear()
        self.queue_draw_item(*items)
        self.emit('selection-changed', self._get_selected_items())

    def _get_selected_items(self):
        """ Return an Item (e.g. StateView) for each model (e.g. StateModel) in the current selection """
        if self._selection_m is None or not self._model:
            return set()
        return set(self.canvas.get_view_for_model(model) for model in self._selection_m)

    def handle_new_selection(self, items):
        """ Determines the selection

        The selection is based on the previous selection, the currently pressed keys and the passes newly selected items

        :param items: The newly selected item(s)
        """
        if items is None:
            items = ()
        elif not hasattr(items, "__iter__"):
            items = (items,)
        models = set(item.model for item in items)
        self._selection_m.handle_new_selection(models)

    selected_items = property(_get_selected_items, select_item, unselect_all, "Items selected by the view")

    @property
    def hovered_item(self):
        return self._selection.hovered_item

    @hovered_item.setter
    def hovered_item(self, item):
        self._selection.hovered_item = item

    @Observer.observe("focus_signal", signal=True)
    def _on_focus_changed_externally(self, selection_m, signal_name, signal_msg):
        previous_focus = self.canvas.get_view_for_model(signal_msg.arg.old_focus)
        current_focus = self.canvas.get_view_for_model(signal_msg.arg.new_focus)
        self.queue_draw_item(previous_focus, current_focus)
        self.emit('focus-changed', current_focus)

    def _get_focused_item(self):
        """ Returns the currently focused item """
        if self._selection_m is None:
            return None
        focused_model = self._selection_m.focus
        if not focused_model:
            return None
        return self.canvas.get_view_for_model(focused_model)

    def _set_focused_item(self, item):
        """ Sets the focus to the passed item"""
        if not item:
            return self._del_focused_item()

        if item.model is not self._selection_m.focus:
            self.queue_draw_item(self._get_focused_item(), item)
            self._selection_m.focus = item.model
            self.emit('focus-changed', item)

    def _del_focused_item(self):
        """ Clears the focus """
        del self._selection_m.focus

    focused_item = property(_get_focused_item, _set_focused_item, _del_focused_item,
                            "The item with focus (receives key events a.o.)")
