from cairo import ANTIALIAS_NONE, Matrix

from gaphas.aspect import PaintFocused, ItemPaintFocused
from gaphas.painter import HandlePainter

from rafcon.mvc.mygaphas.items.connection import ConnectionView, ScopedVariableDataFlowView, DataFlowView
from rafcon.mvc.mygaphas.items.state import StateView


class CustomColorHandlePainter(HandlePainter):
    """
    This class overwrites the default HandlePainter Class in order to be able to adjust the color of the handles.
    """

    def __init__(self, view=None):
        super(HandlePainter, self).__init__(view)

    def _draw_handles(self, item, cairo, opacity=None, inner=False):
        view = self.view
        cairo.save()
        i2v = view.get_matrix_i2v(item)
        if not opacity:
            opacity = (item is view.focused_item) and .7 or .4

        cairo.set_line_width(1)

        get_connection = view.canvas.get_connection
        for h in item.handles():
            if not h.visible or isinstance(item, ConnectionView) and h in item.perp_waypoint_handles():
                continue
            if isinstance(item, ScopedVariableDataFlowView) and h not in item.end_handles():
                continue
            # connected and not being moved, see HandleTool.on_button_press
            if get_connection(h):
                r, g, b = 1, 0, 0
            # connected but being moved, see HandleTool.on_button_press
            elif get_connection(h):
                r, g, b = 1, 0.6, 0
            elif h.movable:
                r, g, b = 46./256., 154./256., 1
            else:
                r, g, b = 0, 0, 1

            cairo.identity_matrix()
            cairo.set_antialias(ANTIALIAS_NONE)
            cairo.translate(*i2v.transform_point(*h.pos))
            # if isinstance(item, StateView) and h in item.corner_handles:
            #     size = item.port_side_size * 2
            #     size_half = size / 2.
            #     cairo.rectangle(-size_half, -size_half, size, size)
            # else:
            cairo.rectangle(-4, -4, 8, 8)
            if inner:
                cairo.rectangle(-3, -3, 6, 6)
            cairo.set_source_rgba(r, g, b, opacity)
            cairo.fill_preserve()
            if h.connectable:
                cairo.move_to(-2, -2)
                cairo.line_to(2, 3)
                cairo.move_to(2, -2)
                cairo.line_to(-2, 3)
            cairo.set_source_rgba(r/4., g/4., b/4., opacity*1.3)
            cairo.stroke()
        cairo.restore()


@PaintFocused.when_type(ConnectionView)
class LineSegmentPainter(ItemPaintFocused):
    """
    This painter draws pseudo-handles on gaphas.item.Line objects. Each
    line can be split by dragging those points, which will result in
    a new handle.

    ConnectHandleTool take care of performing the user
    interaction required for this feature.
    """

    def paint(self, context):
        view = self.view
        item = view.hovered_item
        if isinstance(item, DataFlowView):
            return
        if item and item is view.focused_item:
            cr = context.cairo
            h = item.handles()
            for h1, h2 in zip(h[:-1], h[1:]):
                p1, p2 = h1.pos, h2.pos
                cx = (p1.x + p2.x) / 2
                cy = (p1.y + p2.y) / 2
                cr.save()
                cr.identity_matrix()
                m = Matrix(*view.get_matrix_i2v(item))

                cr.set_antialias(ANTIALIAS_NONE)
                cr.translate(*m.transform_point(cx, cy))
                cr.rectangle(-3, -3, 6, 6)
                cr.set_source_rgba(0, 0, 0.5, .4)
                cr.fill_preserve()
                cr.set_source_rgba(.25, .25, .25, .6)
                cr.set_line_width(1)
                cr.stroke()
                cr.restore()