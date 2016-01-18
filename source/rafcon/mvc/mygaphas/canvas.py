from gaphas.canvas import Canvas


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
        from rafcon.mvc.mygaphas.items.state import StateView
        from rafcon.mvc.mygaphas.items.connection import DataFlowView, TransitionView
        for item in self.get_all_items():
            if isinstance(item, (StateView, TransitionView, DataFlowView)) and item.model is model:
                return item
        return None

    def get_view_for_core_element(self, core_element, parent_item=None):
        """Searches and returns the View for the given core element

        :param core_element: The core element of the searched view
        :param gaphas.item.Item parent_item: Restrict the search to this parent item
        :return: The view for the given core element or None if not found
        """
        from rafcon.mvc.mygaphas.items.state import StateView
        from rafcon.mvc.mygaphas.items.connection import DataFlowView, TransitionView
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
        from rafcon.mvc.mygaphas.items.state import StateView
        from rafcon.mvc.mygaphas.items.connection import DataFlowView, TransitionView
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
