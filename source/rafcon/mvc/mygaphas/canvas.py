from gaphas.canvas import Canvas


class MyCanvas(Canvas):

    def update_root_items(self):
        for root_item in self.get_root_items():
            self.request_update(root_item)

    def get_first_view(self):
        """Return first registered view object
        """
        return next(iter(self._registered_views))