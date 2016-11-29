from gaphas.canvas import Canvas, VariableProjection


class MyCanvas(Canvas):
    def update_root_items(self):
        for root_item in self.get_root_items():
            self.request_update(root_item)

    def get_first_view(self):
        """Return first registered view object
        """
        return next(iter(self._registered_views))

    def get_view_for_model(self, model):
        """Searches and return the View for the given model

        :param gtkmvc.ModelMT model: The model of the searched view
        :return: The view for the given model or None if not found
        """
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import DataFlowView, TransitionView
        from rafcon.gui.mygaphas.items.ports import OutcomeView
        from rafcon.gui.mygaphas.items.ports import ScopedVariablePortView, DataPortView

        for item in self.get_all_items():
            if isinstance(item, (StateView, TransitionView, DataFlowView, OutcomeView, DataPortView,
                                 ScopedVariablePortView)) and item.model is model:
                return item
        return None

    def get_view_for_core_element(self, core_element, parent_item=None):
        """Searches and returns the View for the given core element

        :param core_element: The core element of the searched view
        :param gaphas.item.Item parent_item: Restrict the search to this parent item
        :return: The view for the given core element or None if not found
        """
        from rafcon.gui.mygaphas.items.state import StateView
        from rafcon.gui.mygaphas.items.connection import DataFlowView, TransitionView
        if parent_item is None:
            items = self.get_all_items()
        else:
            items = self.get_children(parent_item)
        for item in items:
            if isinstance(item, StateView) and item.model.state is core_element:
                return item
            if isinstance(item, TransitionView) and item.model.transition is core_element:
                return item
            if isinstance(item, DataFlowView) and item.model.data_flow is core_element:
                return item
        return None

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

    pos = property(lambda self: map(VariableProjection,
                                    self._point, self._get_value(),
                                    (self._on_change_x, self._on_change_y)))

    def __getitem__(self, key):
        # Note: we can not use bound methods as callbacks, since that will
        #       cause pickle to fail.
        return self.pos[key]

    def __iter__(self):
        return iter(self.pos)
