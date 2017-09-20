# Copyright (C) 2015-2017 DLR
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

from gaphas.view import GtkView
from gaphas.item import Element

from rafcon.gui.mygaphas.painter import RAFCONBoundingBoxPainter


class ExtendedGtkView(GtkView):

    hovered_handle = None

    def __init__(self, graphical_editor_v, *args):
        super(ExtendedGtkView, self).__init__(*args)
        self._bounding_box_painter = RAFCONBoundingBoxPainter(self)
        self.graphical_editor = graphical_editor_v

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
        self.queue_draw_area(0, 0, self.allocation[2], self.allocation[3])

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
