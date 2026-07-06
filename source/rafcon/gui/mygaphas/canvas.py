# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gaphas.canvas
from gaphas.canvas import ancestors, all_children
from gaphas.item import matrix_i2i

from rafcon.utils import log
logger = log.get_logger(__name__)


class MyCanvas(gaphas.canvas.Canvas):

    _core_view_map = None
    _model_view_map = None

    def __init__(self):
        super(MyCanvas, self).__init__()
        self._core_view_map = {}
        self._model_view_map = {}

    def _add_view_maps(self, view):
        model = view.model
        if model.core_element in self._core_view_map:
            raise RuntimeError("Core element is already existing in _core_view_map")
        if model in self._model_view_map:
            raise RuntimeError("Model is already existing in _model_view_map")
        self._core_view_map[model.core_element] = view
        self._model_view_map[model] = view

    def _remove_view_maps(self, view):
        model = view.model
        del self._model_view_map[model]
        # Do not retrieve core element from model, as the model could have already been destroyed
        core_element = list(self._core_view_map.keys())[list(self._core_view_map.values()).index(view)]
        del self._core_view_map[core_element]

    def add(self, item, parent=None, index=None):
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import ConnectionView, ConnectionPlaceholderView
        if isinstance(item, (StateView, ConnectionView)) and not isinstance(item, ConnectionPlaceholderView):
            self._add_view_maps(item)
        super(MyCanvas, self).add(item, parent, index)
        # gaphas 5 removed the setup_canvas() hook, RAFCON items still rely on it
        setup_canvas = getattr(item, 'setup_canvas', None)
        if setup_canvas:
            setup_canvas()

    def remove(self, item):
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import ConnectionView, ConnectionPlaceholderView
        if isinstance(item, (StateView, ConnectionView)) and not isinstance(item, ConnectionPlaceholderView):
            self._remove_view_maps(item)
        try:
            super(MyCanvas, self).remove(item)
        except KeyError:
            logger.info("The destruct of gaphas items has to be fixed!")

    def add_port(self, port_v):
        # The LibraryState and its state_copy share the same port core_elements
        if not port_v.parent.is_root_state_of_library:
            self._add_view_maps(port_v)

    def remove_port(self, port_v):
        # The LibraryState and its state_copy share the same port core_elements
        if not port_v.parent.is_root_state_of_library:
            self._remove_view_maps(port_v)

    def exchange_model(self, old_model, new_model):
        view = self._core_view_map[old_model.core_element]
        del self._core_view_map[old_model.core_element]
        del self._model_view_map[old_model]
        self._core_view_map[new_model.core_element] = view
        self._model_view_map[new_model] = view

    def update_root_items(self):
        for root_item in self.get_root_items():
            self.request_update(root_item)

    def get_parent(self, item):
        from gaphas.item import Item
        if not isinstance(item, Item):
            # e.g. PortViews, which are no real gaphas items but have a parent state
            return item.parent
        return super(MyCanvas, self).get_parent(item)

    # ------------------------------------------------------------------
    # Compatibility layer for the gaphas 2.x Canvas API used within RAFCON
    # ------------------------------------------------------------------

    def request_update(self, item, matrix=True):
        """gaphas 5 dropped the `matrix` flag; accepted and ignored here"""
        super(MyCanvas, self).request_update(item)

    def update_now(self, dirty_items=None):
        if dirty_items is None:
            dirty_items = list(self.get_all_items())
        super(MyCanvas, self).update_now(dirty_items)

    def update(self):
        self.update_now()

    def get_all_children(self, item):
        return list(all_children(self, item))

    def get_ancestors(self, item):
        return list(ancestors(self, item))

    def get_matrix_i2i(self, from_item, to_item):
        return matrix_i2i(from_item, to_item)

    def connect_item(self, item, handle, connected, port, constraint=None, callback=None):
        self.connections.connect_item(item, handle, connected, port, constraint, callback)

    def disconnect_item(self, item, handle=None):
        self.connections.disconnect_item(item, handle)

    def get_first_view(self):
        """Return first registered view object
        """
        if len(self._registered_views) == 0:
            return None
        return next(iter(self._registered_views))

    def get_view_for_model(self, model):
        """Searches and return the View for the given model

        :param model: The model of the searched view
        :return: The view for the given model or None if not found
        """
        return self._model_view_map.get(model)

    def get_view_for_core_element(self, core_element, parent_item=None):
        """Searches and returns the View for the given core element

        :param core_element: The core element of the searched view
        :param parent_item: Restrict the search to this parent item
        :return: The view for the given core element or None if not found
        """
        return self._core_view_map.get(core_element)

    def wait_for_update(self, trigger_update=False):
        """Update canvas and handle all events in the gtk queue

        :param bool trigger_update: Whether to call update_now() or not
        """
        if trigger_update:
            self.update_now()

        from gi.repository import GLib
        ctx = GLib.MainContext.default()
        # Process all pending events; this also drives the asyncio view update
        # tasks, which are dispatched via the GLib event loop (gi.events)
        while ctx.pending():
            ctx.iteration(False)
        # Make sure all view update tasks have completed. Never block here: during
        # shutdown (event loop not running) the tasks can no longer be dispatched.
        for view in list(self._registered_views):
            for _ in range(1000):  # upper bound to prevent infinite loops
                task = getattr(view, '_update_task', None)
                if task is None or task.done() or not ctx.pending():
                    break
                ctx.iteration(False)

    def resolve_constraint(self, constraints):
        constraints = constraints if hasattr(constraints, "__iter__") else [constraints]
        for constraint in constraints:
            self.solver.request_resolve_constraint(constraint)
        if not self.solver._solving:
            self.solver.solve()

    def resolve_item_constraints(self, item):
        for constraint in item.constraints:
            self.solver.request_resolve_constraint(constraint)
        if not self.solver._solving:
            self.solver.solve()


class ProjectedVariable(object):
    """Variable-like object projecting one coordinate of a point between items

    Provides the small protocol subset (value, strength, add_handler,
    remove_handler) required by gaphas' BaseConstraint, so ItemProjection
    entries can be passed to constraints like real solver Variables.
    """

    def __init__(self, projection, index):
        self._projection = projection
        self._index = index

    @property
    def strength(self):
        return self._projection.point[self._index].strength

    def add_handler(self, handler):
        self._projection.point[self._index].add_handler(handler)

    def remove_handler(self, handler):
        self._projection.point[self._index].remove_handler(handler)

    @property
    def value(self):
        return self._projection.get_projected()[self._index]

    @value.setter
    def value(self, new_value):
        self._projection.set_projected(self._index, new_value)

    def __float__(self):
        return float(self.value)


class ItemProjection(object):
    """Project a point of item A into the coordinate system of item B.

    Replacement for the gaphas 2.x CanvasProjection-based implementation;
    projection happens on access using the items' item-to-canvas matrices.
    """

    def __init__(self, point, item_point, item_target):
        self.point = point
        self._item_point = item_point
        self._item_target = item_target

    def get_projected(self):
        matrix = matrix_i2i(self._item_point, self._item_target)
        return matrix.transform_point(self.point.x.value, self.point.y.value)

    def set_projected(self, index, value):
        projected = list(self.get_projected())
        projected[index] = value
        matrix = matrix_i2i(self._item_target, self._item_point)
        self.point.x.value, self.point.y.value = matrix.transform_point(*projected)
        canvas = self._item_point.canvas
        if canvas:
            canvas.request_update(self._item_point, matrix=False)

    @property
    def pos(self):
        return ProjectedVariable(self, 0), ProjectedVariable(self, 1)

    def __getitem__(self, key):
        return self.pos[key]

    def __iter__(self):
        return iter(self.pos)
