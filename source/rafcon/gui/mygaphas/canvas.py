# Copyright (C) 2015-2017 DLR
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
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gaphas.canvas
from gaphas.item import Item

from rafcon.utils import log
logger = log.get_logger(__name__)


class MyCanvas(gaphas.canvas.Canvas):

    _core_view_map = None
    _model_view_map = None

    def __init__(self):
        super(MyCanvas, self).__init__()
        self._core_view_map = {}
        self._model_view_map = {}

    def add(self, item, parent=None, index=None):
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import ConnectionView
        if isinstance(item, (StateView, ConnectionView)):
            model = item.model
            self._core_view_map[model.core_element] = item
            self._model_view_map[model] = item
        super(MyCanvas, self).add(item, parent, index)

    def remove(self, item):
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import ConnectionView

        def delete_model_from_maps(model):
            try:
                view = self._model_view_map.pop(model)
                core_element = self._core_view_map.keys()[self._core_view_map.values().index(view)]
                del self._core_view_map[core_element]
            except KeyError:
                from rafcon.gui.models.library_state import LibraryStateModel
                if not isinstance(model.parent, LibraryStateModel):
                    # TODO: Check if this exception can really be ignored
                    raise
        if isinstance(item, StateView):
            delete_model_from_maps(item.model)
            map(delete_model_from_maps, [outcome.model for outcome in item.outcomes])
            map(delete_model_from_maps, [input.model for input in item.inputs])
            map(delete_model_from_maps, [output.model for output in item.outputs])
        elif isinstance(item, ConnectionView):
            delete_model_from_maps(item.model)
        super(MyCanvas, self).remove(item)

    def add_port(self, port_v):
        port_m = port_v.model
        self._core_view_map[port_m.core_element] = port_v
        self._model_view_map[port_m] = port_v

    def remove_port(self, port_v):
        port_m = port_v.model
        del self._core_view_map[port_m.core_element]
        del self._model_view_map[port_m]

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
        if not isinstance(item, Item):
            return item.parent
        return super(MyCanvas, self).get_parent(item)
            

    def get_first_view(self):
        """Return first registered view object
        """
        return next(iter(self._registered_views))

    def get_view_for_model(self, model):
        """Searches and return the View for the given model

        :param gtkmvc.ModelMT model: The model of the searched view
        :return: The view for the given model or None if not found
        """
        return self._model_view_map.get(model)

    def get_view_for_core_element(self, core_element, parent_item=None):
        """Searches and returns the View for the given core element

        :param core_element: The core element of the searched view
        :param gaphas.item.Item parent_item: Restrict the search to this parent item
        :return: The view for the given core element or None if not found
        """
        return self._core_view_map.get(core_element)

    def get_view_for_id(self, view_class, element_id, parent_item=None):
        """Searches and returns the View for the given id and type

        :param view_class: The view type to search for
        :param element_id: The id of element of the searched view
        :param gaphas.item.Item parent_item: Restrict the search to this parent item
        :return: The view for the given id or None if not found
        """
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import DataFlowView, TransitionView
        if parent_item is None:
            items = self.get_all_items()
        else:
            items = self.get_children(parent_item)
        for item in items:
            if view_class is StateView and isinstance(item, StateView) and item.model.state.state_id == element_id:
                return item
            if view_class is TransitionView and isinstance(item, TransitionView) and \
                    item.model.transition.transition_id == element_id:
                return item
            if view_class is DataFlowView and isinstance(item, DataFlowView) and \
                    item.model.data_flow.data_flow_id == element_id:
                return item
        return None

    def perform_update(self):
        """Update canvas and handle all events in the gtk queue
        """
        self.update_now()

        import gtk
        import gobject
        from threading import Event
        event = Event()

        # Handle all events from gaphas, but not from gtkmvc
        # Make use of the priority, which is higher for gaphas then for gtkmvc
        def priority_handled(event):
            event.set()
        priority = (gobject.PRIORITY_HIGH_IDLE + gobject.PRIORITY_DEFAULT_IDLE) / 2
        # idle_add is necessary here, as we do not want to block the user from interacting with the GUI
        # while gaphas is redrawing
        gobject.idle_add(priority_handled, event, priority=priority)
        while not event.is_set():
            gtk.main_iteration(False)


class ItemProjection(object):
    """Project a point of item A into the coordinate system of item B.

    The class os based on the implementation of gaphas.canvas.CanvasProjection.
    """

    def __init__(self, point, item_point, item_target):
        self._point = point
        self._item_point = item_point
        self._item_target = item_target

    def _on_change_x(self, value):
        canvas = self._item_point.canvas
        self._px = value
        self._point.x.value, self._point.y.value = canvas.get_matrix_i2i(self._item_target,
                                                                         self._item_point).transform_point(value,
                                                                                                           self._py)
        canvas.request_update(self._item_point, matrix=False)

    def _on_change_y(self, value):
        canvas = self._item_point.canvas
        self._py = value
        self._point.x.value, self._point.y.value = canvas.get_matrix_i2i(self._item_target,
                                                                         self._item_point).transform_point(self._px,
                                                                                                           value)
        canvas.request_update(self._item_point, matrix=False)

    def _get_value(self):
        """
        Return two delegating variables. Each variable should contain
        a value attribute with the real value.
        """
        x, y = self._point.x, self._point.y
        self._px, self._py = self._item_point.canvas.get_matrix_i2i(self._item_point,
                                                                    self._item_target).transform_point(x, y)
        return self._px, self._py

    pos = property(lambda self: map(gaphas.canvas.VariableProjection,
                                    self._point, self._get_value(),
                                    (self._on_change_x, self._on_change_y)))

    def __getitem__(self, key):
        # Note: we can not use bound methods as callbacks, since that will
        #       cause pickle to fail.
        return self.pos[key]

    def __iter__(self):
        return iter(self.pos)
