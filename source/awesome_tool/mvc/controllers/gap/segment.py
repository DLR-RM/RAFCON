from cairo import Matrix, ANTIALIAS_NONE

from gaphas.geometry import distance_point_point_fast, distance_line_point

from gaphas.segment import LineSegment, Segment

from gaphas.aspect import HandleFinder, HandleSelection, PaintFocused
from gaphas.aspect import ItemHandleFinder, ItemHandleSelection, ItemPaintFocused

from awesome_tool.mvc.views.gap.connection import ConnectionView, DataFlowView


@Segment.when_type(ConnectionView)
class TransitionSegment(LineSegment):

    def split(self, pos):
        item = self.item
        if isinstance(item, DataFlowView):
            return
        handles = item.handles()
        x, y = self.view.get_matrix_v2i(item).transform_point(*pos)
        for h1, h2 in zip(handles, handles[1:]):
            if (h1 in item.end_handles() or h2 in item.end_handles()) and len(handles) > 2:
                continue
            xp = (h1.pos.x + h2.pos.x) / 2
            yp = (h1.pos.y + h2.pos.y) / 2
            if distance_point_point_fast((x, y), (xp, yp)) <= 2. / item.hierarchy_level:
                segment = handles.index(h1)
                handles, ports = self.split_segment(segment)
                return handles and handles[0]

    def split_segment(self, segment, count=2):
        return super(TransitionSegment, self).split_segment(segment, count)


@HandleFinder.when_type(ConnectionView)
class SegmentHandleFinder(ItemHandleFinder):
    """
    Find a handle on a line, create a new one if the mouse is located
    between two handles. The position aligns with the points drawn by
    the SegmentPainter.
    """

    def get_handle_at_point(self, pos):
        view = self.view
        item = view.hovered_item
        handle = None
        if self.item is view.focused_item:
            try:
                segment = Segment(self.item, self.view)
            except TypeError:
                pass
            else:
                handle = segment.split(pos)

        if not handle:
            item, handle = super(SegmentHandleFinder, self).get_handle_at_point(pos)
        return item, handle


@HandleSelection.when_type(ConnectionView)
class SegmentHandleSelection(ItemHandleSelection):
    """
    In addition to the default behaviour, merge segments if the handle is
    released.
    """

    def unselect(self):
        self.view.canvas.solver.solve()

        item = self.item
        handle = self.handle
        handles = item.handles()

        # don't merge using first or last handle
        if handles[0] is handle or handles[-1] is handle:
            return True

        handle_index = handles.index(handle)
        segment = handle_index - 1

        # cannot merge starting from last segment
        if segment == len(item.ports()) - 1:
            segment =- 1
        assert segment >= 0 and segment < len(item.ports()) - 1

        before = handles[handle_index - 1]
        after = handles[handle_index + 1]
        d, p = distance_line_point(before.pos, after.pos, handle.pos)

        if d < 1. / item.hierarchy_level:
            assert len(self.view.canvas.solver._marked_cons) == 0
            Segment(item, self.view).merge_segment(segment)

        if handle:
            item.request_update()


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