from gaphas.item import Line, NW, SE
from gaphas.segment import Segment

from awesome_tool.mvc.views.gap.constraint import KeepPointWithinConstraint
from awesome_tool.mvc.views.gap.ports import IncomeView, OutcomeView, InputPortView, OutputPortView,\
    ScopedDataInputPortView, ScopedDataOutputPortView, PortView
from awesome_tool.mvc.views.gap.ports import SnappedSide

from awesome_tool.utils import constants
from awesome_tool.mvc.config import global_gui_config

from gtk.gdk import Color, CairoContext
from pango import FontDescription
from cairo import ANTIALIAS_SUBPIXEL
from math import atan2, pi


class PerpLine(Line):

    def __init__(self, hierarchy_level, perpendicular_ends=True):
        super(PerpLine, self).__init__()
        self._from_handle = self.handles()[0]
        self._to_handle = self.handles()[1]
        self._segment = Segment(self, view=self.canvas)

        self.hierarchy_level = hierarchy_level

        self._from_port = None
        self._to_port = None

        self._perpendicular_ends = perpendicular_ends

        self._arrow_color = ""
        self._line_color = ""
        self._head_length = 0.

    @property
    def name(self):
        if self.from_port:
            return self.from_port.name
        return None

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
        if self.to_port:
            self.line_width = min(self.to_port.port_side_size, port.port_side_size) * .2
        else:
            self.line_width = port.port_side_size * .2
        self._head_length = port.port_side_size

    @to_port.setter
    def to_port(self, port):
        assert isinstance(port, PortView)
        self._to_port = port
        if self.from_port:
            self.line_width = min(self.from_port.port_side_size, port.port_side_size) * .2

    def from_handle(self):
        return self._from_handle

    def to_handle(self):
        return self._to_handle

    def draw_head(self, context):
        cr = context.cairo
        cr.set_source_color(Color(self._arrow_color))
        cr.move_to(0, 0)
        cr.line_to(self._head_length, 0)
        cr.stroke()
        cr.move_to(self._head_length, 0)

    def draw_tail(self, context):
        cr = context.cairo
        cr.set_source_color(Color(self._line_color))
        cr.line_to(2.5 / self.hierarchy_level, 0)
        cr.stroke()
        cr.set_source_color(Color(self._arrow_color))
        cr.move_to(2.5 / self.hierarchy_level, 0)
        cr.line_to(0, 0)
        cr.line_to(1.0 / self.hierarchy_level, 1.0 / self.hierarchy_level)
        cr.move_to(0, 0)
        cr.line_to(1.0 / self.hierarchy_level, -1.0 / self.hierarchy_level)
        cr.stroke()

    def draw(self, context):
        if self.parent.moving:
            return

        def draw_line_end(pos, angle, draw):
            cr = context.cairo
            cr.save()
            try:
                cr.translate(*pos)
                cr.rotate(angle)
                draw(context)
            finally:
                cr.restore()

        cr = context.cairo
        cr.set_line_width(self.line_width)
        draw_line_end(self._handles[0].pos, self._head_angle, self.draw_head)
        for h in self._handles[1:-1]:
            cr.line_to(*h.pos)
        if not self._perpendicular_ends:
            draw_line_end(self._handles[-1].pos, self._tail_angle, self.draw_tail)
        else:
            draw_line_end(self._get_tail_pos(), self._tail_angle, self.draw_tail)
        cr.stroke()

        if self.name:
            self._draw_name(context)

    def _draw_name(self, context):
        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        if len(self._handles) % 2:
            index = len(self._handles) / 2
            cx, cy = self._handles[index].pos
            angle = 0
        else:
            index1 = len(self._handles) / 2 - 1
            index2 = index1 + 1

            p1, p2 = self._handles[index1].pos, self._handles[index2].pos

            cx = (p1.x + p2.x) / 2
            cy = (p1.y + p2.y) / 2

            angle = atan2(p2.y - p1.y, p2.x - p1.x)
            if angle < -pi / 2.:
                angle += pi
            elif angle > pi / 2.:
                angle -= pi

        outcome_side = self.from_port.port_side_size

        pcc = CairoContext(cc)
        pcc.set_antialias(ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()
        layout.set_text(self.name)

        font_name = constants.FONT_NAMES[0]
        font_size = outcome_side

        font = FontDescription(font_name + " " + str(font_size))
        layout.set_font_description(font)

        cc.set_source_color(Color('#ededee'))
        pcc.update_layout(layout)
        c.save()

        c.move_to(cx, cy)
        if global_gui_config.get_config_value("ROTATE_NAMES_ON_CONNECTIONS", default=False):
            c.rotate(angle)

        pcc.show_layout(layout)

        c.restore()

    def post_update(self, context):
        if self.from_port is None:
            h0, h1 = self._handles[:2]
            p0, p1 = h0.pos, h1.pos
            self._head_angle = atan2(p1.y - p0.y, p1.x - p0.x)
        elif self.from_port.side is SnappedSide.RIGHT:
            self._head_angle = 0 if self.is_out_port(self.from_port) else pi
        elif self.from_port.side is SnappedSide.TOP:
            self._head_angle = pi * 3. / 2. if self.is_out_port(self.from_port) else pi / 2.
        elif self.from_port.side is SnappedSide.LEFT:
            self._head_angle = pi if self.is_out_port(self.from_port) else 0
        elif self.from_port.side is SnappedSide.BOTTOM:
            self._head_angle = pi / 2. if self.is_out_port(self.from_port) else pi * 3. / 2.

        if self.to_port is None:
            h1, h0 = self._handles[-2:]
            p1, p0 = h1.pos, h0.pos
            self._tail_angle = atan2(p1.y - p0.y, p1.x - p0.x)
        elif self.to_port.side is SnappedSide.RIGHT:
            self._tail_angle = pi if self.is_out_port(self.to_port) else 0
        elif self.to_port.side is SnappedSide.TOP:
            self._tail_angle = pi / 2. if self.is_out_port(self.to_port) else pi * 3. / 2.
        elif self.to_port.side is SnappedSide.LEFT:
            self._tail_angle = 0 if self.is_out_port(self.to_port) else pi
        elif self.to_port.side is SnappedSide.BOTTOM:
            self._tail_angle = pi * 3. / 2. if self.is_out_port(self.to_port) else pi / 2.

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

    @staticmethod
    def is_in_port(port):
        return isinstance(port, (IncomeView, InputPortView, ScopedDataInputPortView))

    @staticmethod
    def is_out_port(port):
        return isinstance(port, (OutcomeView, OutputPortView, ScopedDataOutputPortView))

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