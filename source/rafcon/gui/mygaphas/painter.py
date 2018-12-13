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
# Sebastian Brunner <sebastian.brunner@dlr.de>

# from cairo import Antialias
from cairo import Matrix
from builtins import zip

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.utils import constants

import gaphas.painter

from rafcon.gui.mygaphas.aspect import PaintHovered, ItemPaintHovered
from rafcon.gui.mygaphas.items.connection import ConnectionView, TransitionView
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.gui.mygaphas.utils.gap_draw_helper import get_col_rgba, get_side_length_of_resize_handle


# Use this to verify the calculated bounding boxes
# from gaphas import painter
# painter.DEBUG_DRAW_BOUNDING_BOX = True


class CornerHandlePainter(ItemPaintHovered):
    """Base class for drawing corner handles for resize operations
    """

    fill_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_BORDER']

    def _get_handle_side_length(self, item):
        return get_side_length_of_resize_handle(self.view, item)

    def _draw_handles(self, item, cairo, opacity=None):
        view = self.view
        cairo.save()
        i2v = view.get_matrix_i2v(item)
        if not opacity:
            opacity = 1

        side_length = self._get_handle_side_length(item)
        line_width = side_length / constants.BORDER_WIDTH_OUTLINE_WIDTH_FACTOR * 2
        cairo.set_line_width(line_width)

        for index, handle in enumerate(item.handles()):
            if index >= 4:
                break
            # Reset the current transformation
            cairo.identity_matrix()
            #cairo.set_antialias(Antialias.NONE)
            # Move to center of handle
            cairo.translate(*i2v.transform_point(*handle.pos))
            cairo.rectangle(-side_length / 2., -side_length / 2., side_length, side_length)
            # Fill
            cairo.set_source_rgba(*get_col_rgba(self.fill_color, opacity=opacity))
            cairo.fill_preserve()
            # Border
            cairo.set_source_rgba(*get_col_rgba(self.border_color, opacity=opacity))
            cairo.stroke()
        cairo.restore()

    def _paint_guides(self, context):
        # Code copied from gaphas.guide.GuidePainter
        try:
            guides = self.view.guides
        except AttributeError:
            return

        cr = context.cairo
        view = self.view
        allocation = view.get_allocation()
        w, h = allocation.width, allocation.height

        cr.save()
        try:
            cr.set_line_width(1)
            cr.set_source_rgba(0.0, 0.0, 1.0, 0.6)
            for g in guides.vertical():
                cr.move_to(g, 0)
                cr.line_to(g, h)
                cr.stroke()
            for g in guides.horizontal():
                cr.move_to(0, g)
                cr.line_to(w, g)
                cr.stroke()
        finally:
            cr.restore()

    def paint(self, context, selected):
        if selected:
            self._draw_handles(self.item, context.cairo)
        else:
            # Draw nice opaque handles when hovering an non-selected item:
            self._draw_handles(self.item, context.cairo, opacity=.25)

        self._paint_guides(context)


@PaintHovered.when_type(StateView)
class StateCornerHandlePainter(CornerHandlePainter):
    """ Draw corner handles of StateViews """

    fill_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_BORDER']


@PaintHovered.when_type(NameView)
class NameCornerHandlePainter(CornerHandlePainter):
    """ Draw corner handles of NameViews """

    fill_color = gui_config.gtk_colors['NAME_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['NAME_RESIZE_HANDLE_BORDER']


@PaintHovered.when_type(TransitionView)
class LineSegmentPainter(ItemPaintHovered):
    """
    This painter draws pseudo-handles on gaphas.item.Line objects. Each
    line can be split by dragging those points, which will result in
    a new handle.

    ConnectHandleTool take care of performing the user
    interaction required for this feature.
    """

    fill_color = gui_config.gtk_colors['TRANSITION_HANDLE_FILL']
    border_color = gui_config.gtk_colors['TRANSITION_HANDLE_BORDER']

    def paint(self, context, selected):
        view = self.view
        item = self.item
        cr = context.cairo
        h = item.handles()
        side_length = get_side_length_of_resize_handle(self.view, item.parent) / 1.5
        for h1, h2 in zip(h[1:-2], h[2:-1]):
            p1, p2 = h1.pos, h2.pos
            cx = (p1.x + p2.x) / 2
            cy = (p1.y + p2.y) / 2
            cr.save()
            cr.set_line_width(self.view.get_zoom_factor() / 4.)
            cr.identity_matrix()
            m = Matrix(*view.get_matrix_i2v(item))

            # cr.set_antialias(Antialias.NONE)
            cr.translate(*m.transform_point(cx, cy))
            cr.rectangle(-side_length / 2., -side_length / 2., side_length, side_length)
            cr.set_source_rgba(*get_col_rgba(self.fill_color))
            cr.fill_preserve()
            cr.set_source_rgba(*get_col_rgba(self.border_color))
            cr.set_line_width(1)
            cr.stroke()
            cr.restore()


class BoundingBoxPainter(gaphas.painter.BoundingBoxPainter):
    """
    This specific case of an ItemPainter is used to calculate the bounding
    boxes (in canvas coordinates) for the items.
    """

    draw_all = True

    def _draw_item(self, item, cairo, area=None):
        cairo = gaphas.painter.CairoBoundingBoxContext(cairo)
        super(gaphas.painter.BoundingBoxPainter, self)._draw_item(item, cairo)
        bounds = cairo.get_bounds()

        view = self.view
        if isinstance(item, (StateView, NameView)):
            i2v = view.get_matrix_i2v(item).transform_point
            for index, handle in enumerate(item.handles()):
                if index >= 4:
                    break
                side_length = get_side_length_of_resize_handle(view, item)
                cx, cy = i2v(*handle.pos)
                bounds += (cx - side_length / 2, cy - side_length / 2, side_length, side_length)
        elif isinstance(item, ConnectionView):
            i2v = view.get_matrix_i2v(item).transform_point
            for h in item.handles():
                cx, cy = i2v(*h.pos)
                bounds += (cx, cy, 1, 1)

        bounds.expand(1)
        view.set_item_bounding_box(item, bounds)


class HoveredItemPainter(gaphas.painter.Painter):
    """
    This painter allows for drawing on top off all other layers for the
    hovered item.
    """

    def paint(self, context):
        view = self.view
        item = view.hovered_item
        if item:
            selected = item in view.selected_items
            PaintHovered(item, view).paint(context, selected)
