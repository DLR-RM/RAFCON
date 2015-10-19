from math import atan2, pi

from gaphas.item import Line, NW, SE
from cairo import ANTIALIAS_SUBPIXEL
from pango import SCALE

from rafcon.mvc.config import global_gui_config

from rafcon.mvc.mygaphas.constraint import KeepPointWithinConstraint, KeepPortDistanceConstraint
from rafcon.mvc.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, PortView
from rafcon.mvc.mygaphas.utils.enums import SnappedSide
from rafcon.mvc.mygaphas.utils.gap_draw_helper import get_text_layout
from rafcon.mvc.mygaphas.utils.cache.image_cache import ImageCache


class PerpLine(Line):

    def __init__(self, hierarchy_level):
        super(PerpLine, self).__init__()
        self._from_handle = self.handles()[0]
        self._to_handle = self.handles()[1]
        from rafcon.mvc.mygaphas.segment import Segment
        self._segment = Segment(self, view=self.canvas)

        self.hierarchy_level = hierarchy_level

        self._from_port = None
        self._from_waypoint = None
        self._from_port_constraint = None
        self._to_port = None
        self._to_waypoint = None
        self._to_port_constraint = None

        self._arrow_color = None
        self._line_color = None
        self._head_length = 0.
        self._to_head_length = 0.
        self._head_draw_offset = 0.

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
            if handle not in self.end_handles_perp():
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
        self._head_length = port.port_side_size
        if not self._from_waypoint:
            self._from_waypoint = self.add_perp_waypoint()
            self._from_port_constraint = KeepPortDistanceConstraint(self.from_handle().pos, self._from_waypoint.pos,
                                                                    port, self._head_length, self.is_out_port(port))
            self.canvas.solver.add_constraint(self._from_port_constraint)
        if self.to_port:
            self.line_width = min(self.to_port.port_side_size, port.port_side_size) * .2
        else:
            self.line_width = port.port_side_size * .2

    @to_port.setter
    def to_port(self, port):
        assert isinstance(port, PortView)
        self._to_port = port
        self._to_head_length = port.port_side_size
        if not self._to_waypoint:
            self._to_waypoint = self.add_perp_waypoint(begin=False)
            self._to_port_constraint = KeepPortDistanceConstraint(self.to_handle().pos, self._to_waypoint.pos,
                                                                  port, self._to_head_length, self.is_in_port(port))
            self.canvas.solver.add_constraint(self._to_port_constraint)
        if self.from_port:
            self.line_width = min(self.from_port.port_side_size, port.port_side_size) * .2

    def end_handles_perp(self):
        end_handles = [self.from_handle(), self.to_handle()]
        if self._from_waypoint:
            end_handles.append(self._from_waypoint)
        if self._to_waypoint:
            end_handles.append(self._to_waypoint)
        return end_handles

    def end_handles(self):
        return [self.from_handle(), self.to_handle()]

    def perp_waypoint_handles(self):
        waypoint_handles = []
        if self._from_waypoint:
            waypoint_handles.append(self._from_waypoint)
        if self._to_waypoint:
            waypoint_handles.append(self._to_waypoint)
        return waypoint_handles

    def reset_from_port(self):
        self._from_port = None
        self.canvas.solver.remove_constraint(self._from_port_constraint)
        self._from_port_constraint = None
        self._handles.remove(self._from_waypoint)
        self._from_waypoint = None

    def reset_to_port(self):
        self._to_port = None
        self.canvas.solver.remove_constraint(self._to_port_constraint)
        self._to_port_constraint = None
        self._handles.remove(self._to_waypoint)
        self._to_waypoint = None

    def from_handle(self):
        return self._from_handle

    def to_handle(self):
        return self._to_handle

    def draw_head(self, context):
        cr = context.cairo
        cr.set_source_rgba(*self._arrow_color)
        cr.move_to(0, 0)
        cr.line_to(self._head_length, 0)
        cr.stroke()
        cr.move_to(self._head_length, 0)

    def draw_tail(self, context):
        cr = context.cairo
        cr.set_source_rgba(*self._line_color)
        cr.line_to(self._to_head_length, 0)
        cr.stroke()
        cr.set_source_rgba(*self._arrow_color)
        cr.move_to(self._to_head_length, 0)
        cr.line_to(self._head_draw_offset, 0)
        cr.stroke()

    def draw(self, context):
        if self.parent and self.parent.moving:
            return

        def draw_line_end(pos, angle, draw):
            cr = context.cairo
            cr.save()
            cr.translate(*pos)
            cr.rotate(angle)
            draw(context)
            cr.restore()

        cr = context.cairo
        cr.set_line_width(self.line_width)
        draw_line_end(self._handles[0].pos, self._head_angle, self.draw_head)
        for h in self._handles[1:-1]:
            cr.line_to(*h.pos)
        if self.to_port is None:
            self._head_draw_offset = 0.
        else:
            self._head_draw_offset = self._to_head_length / 2.
        draw_line_end(self._handles[-1].pos, self._tail_angle, self.draw_tail)
        cr.stroke()

        if self.name:
            self._draw_name(context)

    def _draw_name(self, context):
        c = context.cairo

        if len(self._handles) % 2:
            index = len(self._handles) / 2
            cx, cy = self._handles[index].pos
            angle = 0
        else:
            index = len(self._handles) / 2 - 1

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

        if self.from_port:
            outcome_side = self.from_port.port_side_size
        else:  # self.to_port:
            outcome_side = self.to_port.port_side_size

        c.set_antialias(ANTIALIAS_SUBPIXEL)

        parameters = {
            'name': self.name,
            'side': outcome_side,
            'color': self._arrow_color
        }

        upper_left_corner = cx, cy
        current_zoom = self.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._label_image_cache.get_cached_image(self._last_label_size[0],
                                                                           self._last_label_size[1],
                                                                           current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "draw port name from cache"
            self._label_image_cache.copy_image_to_context(c, upper_left_corner, angle)

        # Parameters have changed or nothing in cache => redraw
        else:
            # First retrieve pango layout to determine and store size of label
            layout = get_text_layout(c, self.name, outcome_side)
            label_size = layout.get_size()[0] / float(SCALE), layout.get_size()[1] / float(SCALE)
            self._last_label_size = label_size

            # The size information is used to update the caching parameters and retrieve a new context with an image
            # surface of the correct size
            self._label_image_cache.get_cached_image(label_size[0], label_size[1], current_zoom, parameters, clear=True)
            c = self._label_image_cache.get_context_for_image(current_zoom)

            c.set_source_rgba(*self._arrow_color)
            c.update_layout(layout)
            c.show_layout(layout)

            self._label_image_cache.copy_image_to_context(context.cairo, upper_left_corner, angle, zoom=current_zoom)

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
        handle = self._create_handle(pos)
        self._handles.insert(-1, handle)
        self._keep_handle_in_parent_state(handle)
        self._update_ports()

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

    def _get_tail_pos(self):
        if self.to_port is None:
            return self._handles[-1].pos

        tail_pos = None

        to_port_left_side = (self._handles[-1].pos.x.value - self.to_port.port_side_size / 2., self._handles[-1].pos.y.value)
        to_port_right_side = (self._handles[-1].pos.x.value + self.to_port.port_side_size / 2., self._handles[-1].pos.y.value)
        to_port_top_side = (self._handles[-1].pos.x.value, self._handles[-1].pos.y.value - self.to_port.port_side_size / 2.)
        to_port_bottom_side = (self._handles[-1].pos.x.value, self._handles[-1].pos.y.value + self.to_port.port_side_size / 2.)

        if self.to_port.side is SnappedSide.RIGHT:
            tail_pos = to_port_left_side if self.is_out_port(self.to_port) else to_port_right_side
        elif self.to_port.side is SnappedSide.TOP:
            tail_pos = to_port_bottom_side if self.is_out_port(self.to_port) else to_port_top_side
        elif self.to_port.side is SnappedSide.LEFT:
            tail_pos = to_port_right_side if self.is_out_port(self.to_port) else to_port_left_side
        elif self.to_port.side is SnappedSide.BOTTOM:
            tail_pos = to_port_top_side if self.is_out_port(self.to_port) else to_port_bottom_side

        return tail_pos

    def _keep_handle_in_parent_state(self, handle):
        canvas = self.canvas
        parent = canvas.get_parent(self)
        solver = canvas.solver
        if parent is None:
            return
        handle_pos_abs = canvas.project(self, handle.pos)
        parent_nw_abs = canvas.project(parent, parent.handles()[NW].pos)
        parent_se_abs = canvas.project(parent, parent.handles()[SE].pos)
        constraint = KeepPointWithinConstraint(parent_nw_abs, parent_se_abs, handle_pos_abs)
        solver.add_constraint(constraint)