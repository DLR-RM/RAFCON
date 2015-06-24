from gaphas.view import GtkView


class ExtendedGtkView(GtkView):

    def get_item_at_point_exclude(self, pos, selected=True, exclude=None):
        """
        Return the topmost item located at ``pos`` (x, y).

        Parameters:
         - selected: if False returns first non-selected item
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