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

"""Replacements for the gaphas 2.x aspects used by RAFCON

gaphas dropped the generic-function based aspect module (HandleFinder, HandleInMotion,
HandleSelection, ...). This module provides equivalents with plain isinstance dispatch:

* ``HandleFinder(item, view)`` returns a finder object with ``get_handle_at_point()``
* ``HandleInMotion(item, handle, view)`` returns a motion object with the gaphas 2.x
  API (``start_move(pos)``, ``move(pos) -> sink``, ``stop_move()``)
* ``merge_connection_segments()`` replaces SegmentHandleSelection.unselect()
"""

from gaphas.geometry import distance_line_point, distance_rectangle_point
from gaphas.segment import Segment

from rafcon.gui.mygaphas.utils import gap_helper
from rafcon.gui.mygaphas.utils.gap_draw_helper import get_side_length_of_resize_handle
from rafcon.gui.mygaphas.items.connection import ConnectionView, TransitionView, DataFlowView, \
    TransitionPlaceholderView, DataFlowPlaceholderView
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.gui.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedVariablePortView


class ConnectionSink(object):
    """Sink (target item and port) a connection handle is glued/connected to"""

    def __init__(self, item, port=None):
        self.item = item
        self.port = port


class ItemHandleInMotion(object):
    """Move a handle (role is applied to the handle)"""

    GLUE_DISTANCE = 5

    def __init__(self, item, handle, view):
        self.item = item
        self.handle = handle
        self.view = view
        self.last_x = None
        self.last_y = None

    def start_move(self, pos):
        self.last_x, self.last_y = pos
        canvas = self.view.canvas
        cinfo = canvas.connections.get_connection(self.handle)
        if cinfo:
            canvas.solver.remove_constraint(cinfo.constraint)

    def move(self, pos):
        item = self.item
        view = self.view

        v2i = view.get_matrix_v2i(item)
        x, y = v2i.transform_point(*pos)

        self.handle.pos = (x, y)

        sink = self.glue(pos)

        # do not request matrix update as matrix recalculation will be
        # performed due to item normalization if required
        view.canvas.request_update(item, matrix=False)

        return sink

    def stop_move(self):
        pass

    def glue(self, pos, distance=None):
        """Glue to an item near a specific point; returns a ConnectionSink or None"""
        return None


_handle_in_motion_registry = []


def register_handle_in_motion(*types):
    """Register a HandleInMotion class for the given item types"""
    def decorator(cls):
        for type_ in types:
            _handle_in_motion_registry.append((type_, cls))
        return cls
    return decorator


def HandleInMotion(item, handle, view):
    """Create the HandleInMotion object suitable for the type of ``item``"""
    for type_, cls in _handle_in_motion_registry:
        if isinstance(item, type_):
            return cls(item, handle, view)
    return ItemHandleInMotion(item, handle, view)


class ConnectionHandleInMotion(ItemHandleInMotion):
    """
    Move a handle (role is applied to the handle).
    This class replaces the default one in order to adjust the snap distance in move()
    """

    def _exclude_port(self, port):
        return False

    def glue(self, pos, distance=None):
        """
        Glue to an item near a specific point.

        Returns a ConnectionSink or None.
        """
        item = self.item
        handle = self.handle
        view = self.view

        if distance is None:
            distance = self.GLUE_DISTANCE

        if not handle.connectable:
            return None

        state_v, port, glue_pos = view.get_port_at_point(pos, distance=distance, exclude=(item,),
                                                         exclude_port_fun=self._exclude_port)

        # check if item and found item can be connected on closest port
        if port is not None:
            assert state_v is not None

            # transform coordinates from view space to the item space and
            # update position of item's handle
            v2i = view.get_matrix_v2i(item).transform_point
            handle.pos = v2i(*glue_pos)
            return ConnectionSink(state_v, port)

        state_v = view.get_state_at_point(pos, distance=distance)
        if state_v is not None:
            return ConnectionSink(state_v, None)
        return None


@register_handle_in_motion(TransitionView, TransitionPlaceholderView)
class TransitionHandleInMotion(ConnectionHandleInMotion):

    def _exclude_port(self, port):
        port_v = getattr(port, "port_v", None)
        if not isinstance(port_v, (IncomeView, OutcomeView)):
            return True


@register_handle_in_motion(DataFlowView, DataFlowPlaceholderView)
class DataFlowHandleInMotion(ConnectionHandleInMotion):

    def _exclude_port(self, port):
        port_v = getattr(port, "port_v", None)
        if not isinstance(port_v, (InputPortView, OutputPortView, ScopedVariablePortView)):
            return True


class ItemHandleFinder(object):
    """Find handles of an item"""

    def __init__(self, item, view):
        self.item = item
        self.view = view

    def get_handle_at_point(self, pos, distance=None):
        if not distance:
            distance = 6
        item = self.item
        if item is None:
            return None, None
        v2i = self.view.get_matrix_v2i(item)
        d = abs(v2i.transform_distance(distance, 0)[0])
        x, y = v2i.transform_point(*pos)
        for handle in item.handles():
            if not handle.movable:
                continue
            hx, hy = handle.pos
            if -d < (hx - x) < d and -d < (hy - y) < d:
                return item, handle
        return None, None


class ElementHandleFinder(ItemHandleFinder):
    """Find handles in Elements"""

    def _check_for_resize_handle(self, handle, pos, distance):
        side_length_handle = get_side_length_of_resize_handle(self.view, self.item)

        v2i = self.view.get_matrix_v2i(self.item)
        side_length_handle = v2i.transform_distance(side_length_handle, 0)[0]
        x, y = v2i.transform_point(*pos)

        hx, hy = handle.pos
        max_center_distance = side_length_handle / 2 + distance
        if hx - max_center_distance <= x <= hx + max_center_distance and \
                hy - max_center_distance <= y <= hy + max_center_distance:
            return self.item, handle
        return None, None

    def get_handle_at_point(self, pos, distance=None):
        if not distance:
            distance = 0
        for handle in self.item.handles():
            item, handle = self._check_for_resize_handle(handle, pos, distance)
            if item:
                return item, handle
        return None, None


class StateHandleFinder(ElementHandleFinder):
    """Find handles in StateViews"""
    def get_handle_at_point(self, pos, distance=None):
        if not distance:
            distance = 0
        for handle in self.item.handles():
            port_v = self.item.get_port_for_handle(handle)
            if port_v:  # Either a data port or a logical port
                port_area = port_v.get_port_area(self.view)
                if distance_rectangle_point(port_area, pos) <= distance:
                    return self.item, handle
            else:
                item, handle = self._check_for_resize_handle(handle, pos, distance)
                if item:
                    return item, handle
        return None, None


class NameHandleFinder(ElementHandleFinder):
    """Find handles in NameViews"""
    pass


class SegmentHandleFinder(ItemHandleFinder):
    """
    Find a handle on a line, create a new one if the mouse is located
    between two handles. The position aligns with the points drawn by
    the SegmentPainter.
    """

    def get_handle_at_point(self, pos, split=True):
        view = self.view
        connection_v = view.hovered_item
        handle = None

        end_handles = connection_v.end_handles(include_waypoints=True)
        if len(end_handles) == 4:
            from_handle, from_handle_waypoint, to_handle_waypoint, to_handle = end_handles
            cur_pos = self.view.get_matrix_v2i(connection_v).transform_point(*pos)

            max_distance = connection_v.line_width / 2.

            distance_from_segment, _ = distance_line_point(from_handle.pos, from_handle_waypoint.pos, cur_pos)
            if distance_from_segment < max_distance:
                return connection_v, from_handle

            distance_to_segment, _ = distance_line_point(to_handle.pos, to_handle_waypoint.pos, cur_pos)
            if distance_to_segment < max_distance:
                return connection_v, to_handle

        if split:
            try:
                segment = Segment(self.item, self.view.model)
            except TypeError:
                pass
            else:
                # segment.split() expects canvas coordinates in gaphas 5
                cpos = self.view.matrix.inverse().transform_point(*pos)
                handle = segment.split(cpos)

        if not handle:
            connection_v, handle = super(SegmentHandleFinder, self).get_handle_at_point(pos)
        return connection_v, handle


def HandleFinder(item, view):
    """Create the handle finder suitable for the type of ``item``"""
    if isinstance(item, StateView):
        return StateHandleFinder(item, view)
    if isinstance(item, NameView):
        return NameHandleFinder(item, view)
    if isinstance(item, ConnectionView):
        return SegmentHandleFinder(item, view)
    return ItemHandleFinder(item, view)


def merge_connection_segments(view, item, handle):
    """Merge segments of a connection if the released handle lies (almost) on the neighboring segment

    Replacement for the gaphas 2.x SegmentHandleSelection.unselect() aspect.
    """
    view.canvas.solver.solve()

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

    # Checks how far the waypoint is from an imaginary line connecting the previous and next way/end point
    # If it is close, the two segments are merged to one
    merge_distance = item.line_width * 4
    if d < merge_distance:
        Segment(item, view.model).merge_segment(segment)
        # update meta data of waypoints
        if isinstance(item, ConnectionView):
            gap_helper.update_meta_data_for_connection_waypoints(view.graphical_editor, item, None)

    if handle:
        view.canvas.request_update(item)
