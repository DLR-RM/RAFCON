from gaphas.geometry import distance_line_point
from gaphas.segment import Segment
from simplegeneric import generic

from gaphas.aspect import HandleFinder, ItemHandleFinder, HandleSelection, ItemHandleSelection, ItemHandleInMotion
from rafcon.mvc.mygaphas.items.connection import ConnectionView
from rafcon.mvc.mygaphas.items.state import StateView


class ItemHandleInMotionExtended(ItemHandleInMotion):
    """
    Move a handle (role is applied to the handle).
    This class replaces the default one in order to adjust the snap distance in move()
    """

    distance = 0

    def move(self, pos, distance):
        self.distance = distance

        # print type(self), self
        return super(ItemHandleInMotionExtended, self).move(pos)

    def glue(self, pos, distance=None):
        if not distance:
            distance = self.distance

        return super(ItemHandleInMotionExtended, self).glue(pos, distance)


HandleInMotion = generic(ItemHandleInMotionExtended)


@HandleFinder.when_type(StateView)
class StateHandleFinder(ItemHandleFinder):
    """Find handles in state
    """

    def get_handle_at_point(self, pos, distance=None):
        if distance:
            return self.view.get_handle_at_point(pos, distance)
        return self.view.get_handle_at_point(pos)


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
            segment = - 1
        assert 0 <= segment < len(item.ports()) - 1

        before = handles[handle_index - 1]
        after = handles[handle_index + 1]
        d, p = distance_line_point(before.pos, after.pos, handle.pos)

        if d < 1. / item.hierarchy_level:
            assert len(self.view.canvas.solver._marked_cons) == 0
            Segment(item, self.view).merge_segment(segment)

        if handle:
            item.request_update()
