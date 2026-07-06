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

"""Painters for the RAFCON graphical editor on the gaphas 5 painter protocol

gaphas 5 painters implement ``paint(items, cairo)``. Item painting no longer uses a
special bounding box context; bounding boxes are computed by painting single items
onto a recording surface (see ``BoundingBoxItemPainter``). RAFCON items receive a
``RAFCONDrawContext``, which extends the gaphas DrawContext by the ``draw_all`` flag.
"""

from dataclasses import dataclass

from cairo import LINE_JOIN_ROUND
from cairo import Context as CairoContext

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.utils import constants

from rafcon.gui.mygaphas.items.connection import ConnectionView
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.gui.mygaphas.utils.gap_draw_helper import get_col_rgba, get_side_length_of_resize_handle


@dataclass(frozen=True)
class RAFCONDrawContext:
    cairo: CairoContext
    selected: bool
    focused: bool
    hovered: bool
    draw_all: bool


class RAFCONItemPainter(object):
    """Draws all items with RAFCON's extended draw context"""

    def __init__(self, view, draw_all=False):
        self.view = view
        self.draw_all = draw_all

    def paint_item(self, item, cairo):
        view = self.view
        cairo.save()
        try:
            cairo.set_line_join(LINE_JOIN_ROUND)
            cairo.transform(item.matrix_i2c.to_cairo())

            if self.draw_all:
                selected = focused = hovered = False
            else:
                selected = item in view.selected_items
                focused = item is view.focused_item
                hovered = item is view.hovered_item

            item.draw(RAFCONDrawContext(cairo=cairo, selected=selected, focused=focused, hovered=hovered,
                                        draw_all=self.draw_all))
        finally:
            cairo.restore()

    def paint(self, items, cairo):
        """Draw the items"""
        for item in items:
            self.paint_item(item, cairo)


class BoundingBoxItemPainter(RAFCONItemPainter):
    """Item painter used for calculating item bounding boxes (paints onto a recording surface)"""

    def __init__(self, view):
        super(BoundingBoxItemPainter, self).__init__(view, draw_all=True)


class CornerHandlePainter(object):
    """Base class for drawing corner handles for resize operations"""

    fill_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_BORDER']

    def __init__(self, view):
        self.view = view

    def _get_handle_side_length(self, item):
        return get_side_length_of_resize_handle(self.view, item)

    def _draw_handles(self, item, cairo, opacity=None):
        if not opacity:
            opacity = 1

        side_length = self._get_handle_side_length(item)
        line_width = side_length / constants.BORDER_WIDTH_OUTLINE_WIDTH_FACTOR * 2

        for index, handle in enumerate(item.handles()):
            if index >= 4:
                break
            cairo.save()
            # Move to center of handle (in device/pixel space)
            vx, vy = cairo.user_to_device(*item.matrix_i2c.transform_point(*handle.pos))
            cairo.identity_matrix()
            cairo.set_line_width(line_width)
            cairo.translate(vx, vy)
            cairo.rectangle(-side_length / 2., -side_length / 2., side_length, side_length)
            # Fill
            cairo.set_source_rgba(*get_col_rgba(self.fill_color, opacity=opacity))
            cairo.fill_preserve()
            # Border
            cairo.set_source_rgba(*get_col_rgba(self.border_color, opacity=opacity))
            cairo.stroke()
            cairo.restore()

    def paint(self, item, cairo, selected):
        if selected:
            self._draw_handles(item, cairo)
        else:
            # Draw nice opaque handles when hovering a non-selected item:
            self._draw_handles(item, cairo, opacity=.25)


class StateCornerHandlePainter(CornerHandlePainter):
    """ Draw corner handles of StateViews """

    fill_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['STATE_RESIZE_HANDLE_BORDER']


class NameCornerHandlePainter(CornerHandlePainter):
    """ Draw corner handles of NameViews """

    fill_color = gui_config.gtk_colors['NAME_RESIZE_HANDLE_FILL']
    border_color = gui_config.gtk_colors['NAME_RESIZE_HANDLE_BORDER']


class LineSegmentPainter(object):
    """
    This painter draws pseudo-handles on gaphas.item.Line objects. Each
    line can be split by dragging those points, which will result in
    a new handle.

    The connection tools take care of performing the user
    interaction required for this feature.
    """

    fill_color = gui_config.gtk_colors['TRANSITION_HANDLE_FILL']
    border_color = gui_config.gtk_colors['TRANSITION_HANDLE_BORDER']

    def __init__(self, view):
        self.view = view

    def paint(self, item, cairo, selected):
        cr = cairo
        h = item.handles()
        side_length = get_side_length_of_resize_handle(self.view, item.parent) / 1.5
        for h1, h2 in zip(h[1:-2], h[2:-1]):
            p1, p2 = h1.pos, h2.pos
            cx = (p1.x + p2.x) / 2
            cy = (p1.y + p2.y) / 2
            cr.save()
            vx, vy = cr.user_to_device(*item.matrix_i2c.transform_point(cx, cy))
            cr.identity_matrix()
            cr.translate(vx, vy)
            cr.rectangle(-side_length / 2., -side_length / 2., side_length, side_length)
            cr.set_source_rgba(*get_col_rgba(self.fill_color))
            cr.fill_preserve()
            cr.set_source_rgba(*get_col_rgba(self.border_color))
            cr.set_line_width(1)
            cr.stroke()
            cr.restore()


class HoveredItemPainter(object):
    """
    This painter allows for drawing on top off all other layers for the
    hovered item.
    """

    def __init__(self, view):
        self.view = view

    def paint(self, items, cairo):
        view = self.view
        item = view.hovered_item
        if not item:
            return
        selected = item in view.selected_items
        if isinstance(item, StateView):
            StateCornerHandlePainter(view).paint(item, cairo, selected)
        elif isinstance(item, NameView):
            NameCornerHandlePainter(view).paint(item, cairo, selected)
        elif isinstance(item, ConnectionView):
            LineSegmentPainter(view).paint(item, cairo, selected)


class GuidePainter(object):
    """Draws the guides (alignment help lines), set on the view by the guided motion aspects"""

    def __init__(self, view):
        self.view = view

    def paint(self, items, cr):
        try:
            guides = self.view.guides
        except AttributeError:
            return

        view = self.view
        w, h = view.get_width(), view.get_height()

        cr.save()
        try:
            cr.identity_matrix()
            # a width of 1 is hardly visible when using dashed lines
            cr.set_line_width(2)
            cr.set_dash([4], 1)
            guide_color = gui_config.gtk_colors['GUIDE_COLOR']
            cr.set_source_rgba(*get_col_rgba(guide_color, 0.6))
            for g in guides.vertical:
                cr.move_to(g, 0)
                cr.line_to(g, h)
                cr.stroke()
            for g in guides.horizontal:
                cr.move_to(0, g)
                cr.line_to(w, g)
                cr.stroke()
        finally:
            cr.restore()
