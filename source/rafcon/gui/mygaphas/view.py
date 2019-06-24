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
from gtkmvc3.observer import Observer

from gaphas.view import GtkView
from gaphas.item import Element

from rafcon.gui.mygaphas.painter import BoundingBoxPainter
from rafcon.gui.mygaphas.utils.cache.value_cache import ValueCache



class ExtendedGtkView(GtkView, Observer):

    hovered_handle = None
    _selection = None
    _widget_pos = None

    def __init__(self, graphical_editor_v, state_machine_m, *args):
        GtkView.__init__(self, *args)
        Observer.__init__(self)
        self._selection = state_machine_m.selection
        self.value_cache = ValueCache()
        self.observe_model(self._selection)
        self.observe_model(state_machine_m.root_state)
        self._bounding_box_painter = BoundingBoxPainter(self)
        self._graphical_editor = ref(graphical_editor_v)

    def prepare_destruction(self):
        """Get rid of circular references"""
        self._tool = None
        self._painter = None
        self.relieve_model(self._selection)
        self._selection = None
        # clear observer class attributes, also see ExtendenController.destroy()
        self._Observer__PROP_TO_METHS.clear()
        self._Observer__METH_TO_PROPS.clear()
        self._Observer__PAT_TO_METHS.clear()
        self._Observer__METH_TO_PAT.clear()
        self._Observer__PAT_METH_TO_KWARGS.clear()


    @property
    def graphical_editor(self):
        return self._graphical_editor()

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
        # Method had to be inherited, as the base method has a bug:
        # It misses the statement max_dist = d
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

                # transform coordinates from connectable item space to view
                # space
                i2v = self.get_matrix_i2v(i).transform_point
                glue_pos = i2v(*pg)

        return item, port, glue_pos

    def get_item_at_point_exclude(self, pos, selected=True, exclude=None):
        """
        Return the topmost item located at ``pos`` (x, y).

        Parameters:
         - selected: if False returns first non-selected item
         - exclude: if specified don't check for these items
        """
        items = self._qtree.find_intersect((pos[0], pos[1], 1, 1))
        for item in self._canvas.sort(items, reverse=True):
            if not selected and item in self.selected_items:
                continue  # skip selected items
            if item in exclude:
                continue

            v2i = self.get_matrix_v2i(item)
            ix, iy = v2i.transform_point(*pos)
            if item.point((ix, iy)) < 0.5:
                return item
        return None

    def redraw_complete_screen(self):
        self.queue_draw_area(0, 0, self.get_allocation().width, self.get_allocation().height)

    def get_zoom_factor(self):
        """Returns the current zoom factor of the view

        The zoom factor can be read out from the view's matrix. _matrix[0] should be equal _matrix[3]. Index 0 is for
        the zoom in x direction, index 3 for the y direction
        :return: Current zoom factor
        """
        return self._matrix[0]

    def pixel_to_cairo(self, pixel):
        """Helper function to convert pixels to cairo units

        The conversion is depending on the view. The critical parameter is the current zooming factor. The equation is:
        cairo units = pixels / zoom factor

        :param float pixel: Number of pixels to convert
        :return: Number of cairo units corresponding to given number of pixels
        :rtype: float
        """
        zoom = self.get_zoom_factor()
        return pixel / zoom

    def queue_draw_item(self, *items):
        """Extends the base class method to allow Ports to be passed as item

        :param items: Items that are to be redrawn
        """
        gaphas_items = []
        for item in items:
            if isinstance(item, Element):
                gaphas_items.append(item)
            else:
                try:
                    gaphas_items.append(item.parent)
                except AttributeError:
                    pass
        super(ExtendedGtkView, self).queue_draw_item(*gaphas_items)

    def get_items_at_point(self, pos, selected=True, distance=0):
        """ Return the items located at ``pos`` (x, y).

         :param bool selected: if False returns first non-selected item
         :param float distance: Maximum distance to be considered as "at point" (in viewport pixel)
        """
        items = self._qtree.find_intersect((pos[0] - distance, pos[1] - distance, 2 * distance, 2 * distance))
        filtered_items = []
        for item in self._canvas.sort(items, reverse=True):
            if not selected and item in self.selected_items:
                continue  # skip selected items

            v2i = self.get_matrix_v2i(item)
            i2v = self.get_matrix_i2v(item)
            ix, iy = v2i.transform_point(*pos)
            distance_i = item.point((ix, iy))
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
        self.relieve_model(self._selection)
        try:
            yield
        finally:
            self.observe_model(self._selection)

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
                if item is not None and item.model not in self._selection:
                    self._selection.add(item.model)
                    selection_changed = True
        if selection_changed:
            self.emit('selection-changed', self._get_selected_items())

    def unselect_item(self, item):
        """ Unselect an item. """
        self.queue_draw_item(item)
        if item.model in self._selection:
            with self._suppress_selection_events():
                self._selection.remove(item.model)
            self.emit('selection-changed', self._get_selected_items())

    def unselect_all(self):
        """ Clearing the selected_item also clears the focused_item. """
        items = self._get_selected_items()
        with self._suppress_selection_events():
            self._selection.clear()
        self.queue_draw_item(*items)
        self.emit('selection-changed', self._get_selected_items())

    def _get_selected_items(self):
        """ Return an Item (e.g. StateView) for each model (e.g. StateModel) in the current selection """
        return set(self.canvas.get_view_for_model(model) for model in self._selection)

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
        self._selection.handle_new_selection(models)

    selected_items = property(_get_selected_items, select_item, unselect_all, "Items selected by the view")

    @Observer.observe("focus_signal", signal=True)
    def _on_focus_changed_externally(self, selection_m, signal_name, signal_msg):
        previous_focus = self.canvas.get_view_for_model(signal_msg.arg.old_focus)
        current_focus = self.canvas.get_view_for_model(signal_msg.arg.new_focus)
        self.queue_draw_item(previous_focus, current_focus)
        self.emit('focus-changed', current_focus)

    def _get_focused_item(self):
        """ Returns the currently focused item """
        focused_model = self._selection.focus
        if not focused_model:
            return None
        return self.canvas.get_view_for_model(focused_model)

    def _set_focused_item(self, item):
        """ Sets the focus to the passed item"""
        if not item:
            return self._del_focused_item()

        if item.model is not self._selection.focus:
            self.queue_draw_item(self._focused_item, item)
            self._selection.focus = item.model
            self.emit('focus-changed', item)

    def _del_focused_item(self):
        """ Clears the focus """
        del self._selection.focus

    focused_item = property(_get_focused_item, _set_focused_item, _del_focused_item,
                            "The item with focus (receives key events a.o.)")

    def do_configure_event(self, event):
        if hasattr(self, "_back_buffer"):
            GtkView.do_configure_event(self, event)

        # Keep position of state machine fixed within the window, also when size of left sidebar changes
        window = self.get_toplevel()
        if window:
            new_widget_pos = self.translate_coordinates(window, 0, 0)
            if self._widget_pos:
                delta_pos = new_widget_pos[0] - self._widget_pos[0], new_widget_pos[1] - self._widget_pos[1]

                self._matrix.translate(-delta_pos[0] / self._matrix[0], -delta_pos[1] / self._matrix[3])
                # Make sure everything's updated
                self.request_update((), self._canvas.get_all_items())
            self._widget_pos = new_widget_pos
