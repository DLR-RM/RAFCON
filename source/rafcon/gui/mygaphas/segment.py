from gaphas.geometry import distance_point_point
from gaphas.segment import LineSegment, Segment

from rafcon.gui.mygaphas.items.connection import ConnectionView, DataFlowView


@Segment.when_type(ConnectionView)
class TransitionSegment(LineSegment):
    """
    This class is used to redefine the behavior of transitions and how new waypoints may be added.
    It checks if the waypoint that should be created is not between the perpendicular connectors to the ports.
    """

    def split(self, pos):
        item = self.item
        if isinstance(item, DataFlowView):
            return
        handles = item.handles()
        x, y = self.view.get_matrix_v2i(item).transform_point(*pos)
        max_dist = item.parent.border_width / 4.
        for h1, h2 in zip(handles, handles[1:]):
            if (h1 in item.end_handles() or h2 in item.end_handles()) and len(handles) > 2:
                continue
            xp = (h1.pos.x + h2.pos.x) / 2.
            yp = (h1.pos.y + h2.pos.y) / 2.
            if distance_point_point((x, y), (xp, yp)) <= max_dist:
                segment = handles.index(h1)
                handles, ports = self.split_segment(segment)
                return handles and handles[0]

    def split_segment(self, segment, count=2):
        return super(TransitionSegment, self).split_segment(segment, 2)


