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
        from rafcon.mvc.mygaphas.items.state import StateView
        from rafcon.mvc.mygaphas.items.connection import DataFlowView, TransitionView
        for item in self.get_all_items():
            if isinstance(item, (StateView, TransitionView, DataFlowView)) and item.model is model:
                return item
        return None

    def get_view_for_core_element(self, core_element, parent_item=None):
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