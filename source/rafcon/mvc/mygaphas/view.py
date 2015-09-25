from gaphas.view import GtkView


class ExtendedGtkView(GtkView):

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
        self.queue_draw_area(0, 0, self.allocation[2], self.allocation[3])

    def get_zoom_factor(self):
        """Returns the current zoom factor of the view

        The zoom factor can be read out from the view's matrix. _matrix[0] should be equal _matrix[3]. Index 0 is for
        the zoom in x direction, index 3 for the y direction
        :return: Current zoom factor
        """
        return self._matrix[0]