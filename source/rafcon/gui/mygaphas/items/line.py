# Copyright (C) 2015-2018 DLR
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

from builtins import zip
from math import atan2, pi, floor

from gaphas.item import Line, NW, SE
from gaphas.painter import CairoBoundingBoxContext
# from cairo import Antialias, LINE_CAP_ROUND, LINE_CAP_BUTT
from cairo import LINE_CAP_ROUND, LINE_CAP_BUTT
from gi.repository.Pango import SCALE
from gi.repository import PangoCairo

from rafcon.gui.config import global_gui_config

from rafcon.gui.mygaphas.canvas import ItemProjection
from rafcon.gui.mygaphas.constraint import KeepPointWithinConstraint, KeepPortDistanceConstraint
from rafcon.gui.mygaphas.items.ports import IncomeView, OutcomeView, ScopedVariablePortView, \
                                            InputPortView, OutputPortView, LogicPortView, PortView
from rafcon.gui.utils import constants
from rafcon.gui.mygaphas.utils.gap_draw_helper import get_text_layout, FONT_SIZE
from rafcon.gui.mygaphas.utils.cache.image_cache import ImageCache


class PerpLine(Line):
    def __init__(self, hierarchy_level):
        from rafcon.gui.mygaphas.segment import Segment
        super(PerpLine, self).__init__()
        self._from_handle = self.handles()[0]
        self._to_handle = self.handles()[1]
        self._segment = Segment(self, view=self.canvas)

        self.hierarchy_level = hierarchy_level

        self._from_port = None
        self._from_waypoint = None
        self._from_port_constraint = None
        self._to_port = None
        self._to_waypoint = None
        self._to_port_constraint = None
        self._waypoint_constraints = []

        self._arrow_color = None
        self._line_color = None

        self._parent = None
        self._parent_state_v = None
        self._view = None

        self._label_image_cache = ImageCache()
        self._last_label_size = 0, 0

    @property
    def name(self):
        if self.from_port:
            return self.from_port.name
        return None

    @property
    def waypoints(self):
        waypoints = []
        for handle in self.handles():
            if handle not in self.end_handles(include_waypoints=True):
                waypoints.append(handle)
        return waypoints

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    @property
    def from_port(self):
        return self._from_port

    @property
    def to_port(self):
        return self._to_port

    @from_port.setter
    def from_port(self, port):
        assert isinstance(port, PortView)
        self._from_port = port
        if not self._from_waypoint:
            self._from_waypoint = self.add_perp_waypoint()
            self._from_port_constraint = KeepPortDistanceConstraint(self.from_handle().pos, self._from_waypoint.pos,
                                                                    port, lambda: self._head_length(self.from_port) +
                                                                                  self._head_offset(self.from_port),
                                                                    self.is_out_port(port))
            self.canvas.solver.add_constraint(self._from_port_constraint)

    @to_port.setter
    def to_port(self, port):
        assert isinstance(port, PortView)
        self._to_port = port
        if not self._to_waypoint:
            self._to_waypoint = self.add_perp_waypoint(begin=False)
            self._to_port_constraint = KeepPortDistanceConstraint(self.to_handle().pos, self._to_waypoint.pos, port,
                                                                  lambda: self._head_length(self.to_port) +
                                                                          self._head_offset(self.to_port),
                                                                  self.is_in_port(port))
            self.canvas.solver.add_constraint(self._to_port_constraint)

    def remove(self):
        self.reset_from_port()
        self.reset_to_port()
        self.remove_all_waypoints()
        self.canvas.remove(self)

    @property
    def view(self):
        if not self._view:
            self._view = self.canvas.get_first_view()
        return self._view

    def end_handles(self, include_waypoints=False):
        end_handles = [self.from_handle(), self.to_handle()]
        if include_waypoints:
            if self._from_waypoint:
                end_handles.insert(1, self._from_waypoint)
            if self._to_waypoint:
                end_handles.insert(-1, self._to_waypoint)
        return end_handles

    def reset_from_port(self):
        if self._from_port:
            self._from_port = None
            self.canvas.solver.remove_constraint(self._from_port_constraint)
            self._from_port_constraint = None
            self._handles.remove(self._from_waypoint)
            self._from_waypoint = None

    def reset_to_port(self):
        if self._to_port:
            self._to_port = None
            self.canvas.solver.remove_constraint(self._to_port_constraint)
            self._to_port_constraint = None
            self._handles.remove(self._to_waypoint)
            self._to_waypoint = None

    def from_handle(self):
        return self._from_handle

    def to_handle(self):
        return self._to_handle

    def get_parent_state_v(self):
        if not self._parent_state_v:
            if not self.from_port:
                return None
            if isinstance(self.from_port, (IncomeView, InputPortView, ScopedVariablePortView)):
                self._parent_state_v = self.from_port.parent
            else:
                self._parent_state_v = self.from_port.parent.parent
        return self._parent_state_v

    def draw_head(self, context, port):
        offset = self._head_offset(port)
        length = self._head_length(port)
        cr = context.cairo
        cr.move_to(offset, 0)
        cr.line_to(offset + length, 0)
        cr.set_source_rgba(*self._arrow_color)
        cr.set_line_width(self._calc_line_width(port))
        cr.set_line_cap(LINE_CAP_BUTT)
        cr.stroke()

    def draw_tail(self, context, port):
        offset = self._head_offset(port)
        length = self._head_length(port)
        cr = context.cairo
        cr.move_to(offset, 0)
        cr.line_to(offset + length, 0)
        cr.set_source_rgba(*self._arrow_color)
        cr.set_line_width(self._calc_line_width(port))
        cr.set_line_cap(LINE_CAP_BUTT)
        cr.stroke()

    def draw(self, context):
        if self.parent and self.parent.moving:
            return

        def draw_line_end(pos, angle, port, draw):
            cr.save()
            cr.translate(*pos)
            cr.rotate(angle)
            draw(context, port)
            cr.restore()
        self.line_width = self._calc_line_width()
        cr = context.cairo
        cr.set_line_cap(LINE_CAP_ROUND)
        cr.set_line_width(self.line_width)

        # Draw connection tail (line perpendicular to from_port)
        start_segment_index = 0
        if self.from_port:
            draw_line_end(self._handles[0].pos, self._head_angle, self.from_port, self.draw_tail)
            start_segment_index = 1

        # Draw connection head (line perpendicular to to_port)
        end_segment_index = len(self._handles)
        if self.to_port:
            draw_line_end(self._handles[-1].pos, self._tail_angle, self.to_port, self.draw_head)
            end_segment_index -= 1

        # Draw connection line from waypoint to waypoint
        cr.move_to(*self._handles[start_segment_index].pos)
        for h in self._handles[start_segment_index+1:end_segment_index]:
            cr.line_to(*h.pos)
        cr.set_source_rgba(*self._line_color)
        cr.stroke()

        if self.name and (isinstance(self.from_port, LogicPortView) or
                          global_gui_config.get_config_value("SHOW_NAMES_ON_DATA_FLOWS", default=True)):
            self._draw_name(context)

    def _draw_name(self, context):
        c = context.cairo

        if len(self._handles) % 2:  # uneven
            index = int(floor(len(self._handles) / 2))
            cx, cy = self._handles[index].pos
            angle = 0
        else:
            index = int(len(self._handles) / 2) - 1

            p1, p2 = self._handles[index].pos, self._handles[index + 1].pos

            cx = (p1.x + p2.x) / 2
            cy = (p1.y + p2.y) / 2

            if global_gui_config.get_config_value("ROTATE_NAMES_ON_CONNECTIONS", default=False):
                angle = atan2(p2.y - p1.y, p2.x - p1.x)
                if angle < -pi / 2.:
                    angle += pi
                elif angle > pi / 2.:
                    angle -= pi
            else:
                angle = 0

        # c.set_antialias(Antialias.GOOD)

        parameters = {
            'name': self.name,
            'line_width': self.line_width,
            'color': self._arrow_color
        }

        upper_left_corner = cx, cy
        current_zoom = self.view.get_zoom_factor()
        from_cache, image, zoom = self._label_image_cache.get_cached_image(self._last_label_size[0],
                                                                           self._last_label_size[1],
                                                                           current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print("draw port name from cache")
            self._label_image_cache.copy_image_to_context(c, upper_left_corner, angle)

        # Parameters have changed or nothing in cache => redraw
        else:
            # First retrieve pango layout to determine and store size of label
            cairo_context = c
            if isinstance(c, CairoBoundingBoxContext):
                cairo_context = c._cairo
            layout = get_text_layout(cairo_context, self.name, FONT_SIZE)

            ink_extents, logical_extents = layout.get_extents()
            extents = [extent / float(SCALE) for extent in [logical_extents.x, logical_extents.y,
                                                            logical_extents.width, logical_extents.height]]
            real_label_size = extents[2], extents[3]
            desired_height = self.line_width * 2.5
            scale_factor = real_label_size[1] / desired_height
            label_size = real_label_size[0] / scale_factor, desired_height

            self._last_label_size = label_size

            # The size information is used to update the caching parameters and retrieve a new context with an image
            # surface of the correct size
            self._label_image_cache.get_cached_image(label_size[0], label_size[1], current_zoom, parameters, clear=True)
            c = self._label_image_cache.get_context_for_image(current_zoom)
            cairo_context = c
            layout = get_text_layout(cairo_context, self.name, FONT_SIZE)

            c.set_source_rgba(*self._arrow_color)
            c.scale(1. / scale_factor, 1. / scale_factor)
            PangoCairo.update_layout(cairo_context, layout)
            PangoCairo.show_layout(cairo_context, layout)

            self._label_image_cache.copy_image_to_context(context.cairo, upper_left_corner, angle, zoom=current_zoom)

    def _calc_line_width(self, for_port=None):
        parent_state_v = self.get_parent_state_v()
        if not parent_state_v:
            return 0
        line_width = parent_state_v.border_width / constants.BORDER_WIDTH_LINE_WIDTH_FACTOR
        if for_port:
            return min(line_width, for_port.port_size[0])
        return line_width

    def _head_length(self, port):
        """Distance from the center of the port to the perpendicular waypoint"""
        if not port:
            return 0.
        parent_state_v = self.get_parent_state_v()
        if parent_state_v is port.parent:  # port of connection's parent state
            return port.port_size[1]
        return max(port.port_size[1] * 1.5, self._calc_line_width() / 1.3)

    def _head_offset(self, port):
        """How far away from the port center does the line begin"""
        if not port:
            return 0.
        return port.port_size[1] / 2

    def _update_ports(self):
        assert len(self._handles) >= 2, 'Not enough segments'
        self._ports = []
        handles = self._handles
        for h1, h2 in zip(handles[:-1], handles[1:]):
            self._ports.append(self._create_port(h1.pos, h2.pos))

    def _reversible_insert_handle(self, index, handle):
        super(PerpLine, self)._reversible_insert_handle(index, handle)
        self._keep_handle_in_parent_state(handle)

    def add_waypoint(self, pos):
        pos = self.canvas.get_matrix_i2i(self.parent, self).transform_point(*pos)
        handle = self._create_handle(pos)
        if self._to_waypoint:
            self._handles.insert(-2, handle)
        else:
            self._handles.insert(-1, handle)
        self._keep_handle_in_parent_state(handle)
        self._update_ports()
        return handle

    def remove_all_waypoints(self):
        waypoints = self.waypoints
        for waypoint in waypoints:
            self._handles.remove(waypoint)
        self._update_ports()
        for constraint in self._waypoint_constraints:
            self.canvas.solver.remove_constraint(constraint)
        self._waypoint_constraints = []

    def add_perp_waypoint(self, pos=(0, 0), begin=True):
        handle = self._create_handle(pos)
        if begin:
            self._handles.insert(1, handle)
        else:
            self._handles.insert(len(self._handles) - 1, handle)
        self._update_ports()
        return handle

    @staticmethod
    def is_in_port(port):
        return isinstance(port, (IncomeView, InputPortView))

    @staticmethod
    def is_out_port(port):
        return isinstance(port, (OutcomeView, OutputPortView))

    def point(self, pos):
        distance = super(PerpLine, self).point(pos)
        return distance - self.line_width / 1.5

    def _keep_handle_in_parent_state(self, handle):
        canvas = self.canvas
        parent = canvas.get_parent(self)
        solver = canvas.solver
        if parent is None:
            return
        handle_pos = ItemProjection(handle.pos, self, self.parent)
        constraint = KeepPointWithinConstraint(parent.handles()[NW].pos, parent.handles()[SE].pos,
                                               handle_pos, lambda: parent.border_width)
        solver.add_constraint(constraint)
        self._waypoint_constraints.append(constraint)
