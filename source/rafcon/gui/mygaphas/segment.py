# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import zip
from weakref import ref

from gaphas.geometry import distance_point_point
from gaphas.segment import LineSegment, Segment

from rafcon.gui.mygaphas.items.connection import ConnectionView, DataFlowView


@Segment.when_type(ConnectionView)
class TransitionSegment(LineSegment):
    """
    This class is used to redefine the behavior of transitions and how new waypoints may be added.
    It checks if the waypoint that should be created is not between the perpendicular connectors to the ports.
    """

    _view = None
    _item = None

    def __init__(self, item, view):
        if item:
            self._item = ref(item)
        if view:
            self._view = ref(view)

    @property
    def item(self):
        if self._item:
            return self._item()
        return None

    @property
    def view(self):
        if self._view:
            return self._view()
        return None

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


