from weakref import ref

from gaphas.connector import PointPort, Handle

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel

from awesome_tool.utils import constants

import cairo
from pango import SCALE, FontDescription
from gtk.gdk import Color, CairoContext

import math

from enum import Enum

Direction = Enum('DIRECTION', 'UP DOWN LEFT RIGHT')


class SnappedSide(Enum):
    LEFT = 1
    TOP = 2
    RIGHT = 3
    BOTTOM = 4

    def next(self):
        val = self.value + 1
        if val == 5:
            val = 1
        return SnappedSide(val)

    def prev(self):
        val = self.value - 1
        if val == 0:
            val = 4
        return SnappedSide(val)


class PortView(object):

    def __init__(self, in_port, name=None, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = PointPort(self.handle.pos)
        self.side = side
        self._parent = parent

        # self._connected_handles = {}
        # self._tmp_connected = False

        self._incoming_handles = []
        self._outgoing_handles = []
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

        self._name = name

        self._is_in_port = in_port

        self.port_side_size = 0.
        self.update_port_side_size()

    @property
    def name(self):
        return self._name

    @property
    def parent(self):
        return self._parent

    @property
    def pos(self):
        return self.handle.pos

    @property
    def handle_pos(self):
        return self.handle.pos

    @property
    def port_pos(self):
        return self.port.point

    # @property
    # def connected_handles(self):
    #     return self._connected_handles.iterkeys()

    def has_outgoing_connection(self):
        return len(self._outgoing_handles) > 0

    def has_incoming_connection(self):
        return len(self._incoming_handles) > 0

    def add_connected_handle(self, handle, connection_view, moving=False):
        from awesome_tool.mvc.views.gap.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if not moving and handle is connection_view.from_handle() and handle not in self._outgoing_handles:
            self._outgoing_handles.append(handle)
        elif not moving and handle is connection_view.to_handle() and handle not in self._incoming_handles:
            self._incoming_handles.append(handle)

    def remove_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle in self._incoming_handles:
            self._incoming_handles.remove(handle)
        elif handle in self._outgoing_handles:
            self._outgoing_handles.remove(handle)

    def tmp_connect(self, handle, connection_view):
        if handle is connection_view.from_handle():
            self._tmp_outgoing_connected = True
        elif handle is connection_view.to_handle():
            self._tmp_incoming_connected = True

    def tmp_disconnect(self):
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

    @property
    def connected_outgoing(self):
        if len(self._outgoing_handles) == 0:
            return self._tmp_outgoing_connected
        return True

    @property
    def connected_incoming(self):
        if len(self._incoming_handles) == 0:
            return self._tmp_incoming_connected
        return True

    def draw(self, context, state):
        raise NotImplementedError

    def draw_port(self, context, fill_color):
        self.update_port_side_size()
        c = context.cairo
        outcome_side = self.port_side_size
        c.set_line_width(outcome_side * 0.03)

        direction = None

        if self.side is SnappedSide.LEFT:
            direction = Direction.RIGHT if self._is_in_port else Direction.LEFT
        elif self.side is SnappedSide.TOP:
            direction = Direction.DOWN if self._is_in_port else Direction.UP
        elif self.side is SnappedSide.RIGHT:
            direction = Direction.LEFT if self._is_in_port else Direction.RIGHT
        elif self.side is SnappedSide.BOTTOM:
            direction = Direction.UP if self._is_in_port else Direction.DOWN

        # Outer part
        self._draw_triangle(self.pos, direction, c, outcome_side, draw_inner=False)
        # c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        c.move_to(0, 0)
        c.set_source_color(Color('#000'))
        c.fill_preserve()
        c.set_source_color(Color(fill_color))
        c.stroke()

        # Inner part
        if self.connected_incoming and self.connected_outgoing:
            self._draw_triangle(self.pos, direction, c, outcome_side, draw_inner=True)
        elif self.connected_incoming:
            self._draw_triangle_half(self.pos, direction, c, outcome_side, front_part=False)
        elif self.connected_outgoing:
            self._draw_triangle_half(self.pos, direction, c, outcome_side, front_part=True)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        if self.name and not self.has_outgoing_connection() and self.parent.parent:
            self.draw_name(context)

    def draw_name(self, context):
        outcome_side = self.port_side_size
        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        pcc = CairoContext(cc)
        pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()
        layout.set_text(self.name)

        font_name = constants.FONT_NAMES[0]
        font_size = outcome_side

        font = FontDescription(font_name + " " + str(font_size))
        layout.set_font_description(font)

        cc.set_source_color(Color('#ededee'))

        rot_angle = .0

        if self.side is SnappedSide.RIGHT:
            c.move_to(self.pos.x + outcome_side, self.pos.y - outcome_side / 2.)
        elif self.side is SnappedSide.TOP:
            c.move_to(self.pos.x - outcome_side / 2., self.pos.y - outcome_side)
            rot_angle = - math.pi / 2
        elif self.side is SnappedSide.LEFT:
            c.move_to(self.pos.x - (outcome_side + layout.get_size()[0] / float(SCALE)), self.pos.y - outcome_side / 2.)
        elif self.side is SnappedSide.BOTTOM:
            c.move_to(self.pos.x + outcome_side / 2., self.pos.y + outcome_side)
            rot_angle = math.pi / 2

        pcc.update_layout(layout)
        pcc.rotate(rot_angle)
        pcc.show_layout(layout)
        pcc.rotate(-rot_angle)

        c.move_to(outcome_side, outcome_side)

    @staticmethod
    def _draw_triangle(pos, direction, context_cairo, outcome_side, draw_inner):
        c = context_cairo

        side_half = outcome_side / 2.

        if draw_inner:
            multiplier = .8
            multiplier_comp_1 = multiplier * 1.125
            multiplier_comp_2 = multiplier * 1.05
        else:
            multiplier = 1.
            multiplier_comp_1 = 1.
            multiplier_comp_2 = 1.

        if direction is Direction.UP:
            c.move_to(pos.x, pos.y - side_half * multiplier)
            c.line_to(pos.x - side_half * multiplier_comp_2, pos.y + side_half * multiplier_comp_1)
            c.line_to(pos.x + side_half * multiplier_comp_2, pos.y + side_half * multiplier_comp_1)
            c.line_to(pos.x, pos.y - side_half * multiplier)
        elif direction is Direction.DOWN:
            c.move_to(pos.x, pos.y + side_half * multiplier)
            c.line_to(pos.x - side_half * multiplier_comp_2, pos.y - side_half * multiplier_comp_1)
            c.line_to(pos.x + side_half * multiplier_comp_2, pos.y - side_half * multiplier_comp_1)
            c.line_to(pos.x, pos.y + side_half * multiplier)
        elif direction is Direction.LEFT:
            c.move_to(pos.x - side_half * multiplier, pos.y)
            c.line_to(pos.x + side_half * multiplier_comp_1, pos.y - side_half * multiplier_comp_2)
            c.line_to(pos.x + side_half * multiplier_comp_1, pos.y + side_half * multiplier_comp_2)
            c.line_to(pos.x - side_half * multiplier, pos.y)
        elif direction is Direction.RIGHT:
            c.move_to(pos.x + side_half * multiplier, pos.y)
            c.line_to(pos.x - side_half * multiplier_comp_1, pos.y - side_half * multiplier_comp_2)
            c.line_to(pos.x - side_half * multiplier_comp_1, pos.y + side_half * multiplier_comp_2)
            c.line_to(pos.x + side_half * multiplier, pos.y)

    @staticmethod
    def _draw_triangle_half(pos, direction, context_cairo, outcome_side, front_part):
        c = context_cairo

        multiplier_comp_1 = 1.125
        multiplier_comp_2 = 1.05

        side_half = outcome_side / 2. * .8
        side_quarter = outcome_side / 4. * .8

        if direction is Direction.UP:
            if front_part:
                c.move_to(pos.x - side_quarter, pos.y)
                c.line_to(pos.x, pos.y - side_half)
                c.line_to(pos.x + side_quarter, pos.y)
                c.line_to(pos.x - side_quarter, pos.y)
            else:
                c.move_to(pos.x - side_quarter, pos.y)
                c.line_to(pos.x + side_quarter, pos.y)
                c.line_to(pos.x + side_half * multiplier_comp_2, pos.y + side_half * multiplier_comp_1)
                c.line_to(pos.x - side_half * multiplier_comp_2, pos.y + side_half * multiplier_comp_1)
                c.line_to(pos.x - side_quarter, pos.y)
        elif direction is Direction.DOWN:
            if front_part:
                c.move_to(pos.x - side_quarter, pos.y)
                c.line_to(pos.x, pos.y + side_half)
                c.line_to(pos.x + side_quarter, pos.y)
                c.line_to(pos.x - side_quarter, pos.y)
            else:
                c.move_to(pos.x - side_quarter, pos.y)
                c.line_to(pos.x + side_quarter, pos.y)
                c.line_to(pos.x + side_half * multiplier_comp_2, pos.y - side_half * multiplier_comp_1)
                c.line_to(pos.x - side_half * multiplier_comp_2, pos.y - side_half * multiplier_comp_1)
                c.line_to(pos.x - side_quarter, pos.y)
        elif direction is Direction.LEFT:
            if front_part:
                c.move_to(pos.x, pos.y - side_quarter)
                c.line_to(pos.x - side_half, pos.y)
                c.line_to(pos.x, pos.y + side_quarter)
                c.line_to(pos.x, pos.y - side_quarter)
            else:
                c.move_to(pos.x, pos.y - side_quarter)
                c.line_to(pos.x, pos.y + side_quarter)
                c.line_to(pos.x + side_half * multiplier_comp_1, pos.y + side_half * multiplier_comp_2)
                c.line_to(pos.x + side_half * multiplier_comp_1, pos.y - side_half * multiplier_comp_2)
                c.line_to(pos.x, pos.y - side_quarter)
        elif direction is Direction.RIGHT:
            if front_part:
                c.move_to(pos.x, pos.y - side_quarter)
                c.line_to(pos.x + side_half, pos.y)
                c.line_to(pos.x, pos.y + side_quarter)
                c.line_to(pos.x, pos.y - side_quarter)
            else:
                c.move_to(pos.x, pos.y - side_quarter)
                c.line_to(pos.x, pos.y + side_quarter)
                c.line_to(pos.x - side_half * multiplier_comp_1, pos.y + side_half * multiplier_comp_2)
                c.line_to(pos.x - side_half * multiplier_comp_1, pos.y - side_half * multiplier_comp_2)
                c.line_to(pos.x, pos.y - side_quarter)

    def update_port_side_size(self):
        if self._parent:
            self.port_side_size = min(self._parent.width, self._parent.height) / 20.
        else:
            self.port_side_size = 5.


class IncomeView(PortView):

    def __init__(self, parent):
        super(IncomeView, self).__init__(in_port=True, parent=parent, side=SnappedSide.LEFT)

    def draw(self, context, state):
        self.draw_port(context, "#fff")


class OutcomeView(PortView):

    def __init__(self, outcome_m, parent):
        super(OutcomeView, self).__init__(in_port=False, name=outcome_m.outcome.name, parent=parent)

        assert isinstance(outcome_m, OutcomeModel)
        self._outcome_m = ref(outcome_m)
        self.sort = outcome_m.outcome.outcome_id

    @property
    def outcome_m(self):
        return self._outcome_m()

    @property
    def outcome_id(self):
        return self.outcome_m.outcome.outcome_id

    @property
    def name(self):
        return self.outcome_m.outcome.name

    def draw(self, context, state):
        if self.outcome_id == -2:
            fill_color = '#00f'
        elif self.outcome_id == -1:
            fill_color = '#f00'
        else:
            fill_color = '#fff'

        self.draw_port(context, fill_color)


class ScopedDataPortView(PortView):

    def __init__(self, in_port, parent, scoped_variable_m, side):
        super(ScopedDataPortView, self).__init__(in_port=in_port, parent=parent, side=side)

        assert isinstance(scoped_variable_m, ScopedVariableModel)
        self._scoped_variable_m = ref(scoped_variable_m)

    @property
    def scoped_variable_m(self):
        return self._scoped_variable_m()

    @property
    def port_id(self):
        return self.scoped_variable_m.scoped_variable.data_port_id

    def draw(self, context, state):
        self.draw_port(context, '#ffc926')

    def update_port_side_size(self):
        if self._parent:
            self.port_side_size = min(self._parent.width, self._parent.height) / 5.
        else:
            self.port_side_size = 5.


class ScopedDataInputPortView(ScopedDataPortView):

    def __init__(self, parent, scoped_variable_m):
        super(ScopedDataInputPortView, self).__init__(True, parent, scoped_variable_m, SnappedSide.LEFT)


class ScopedDataOutputPortView(ScopedDataPortView):

    def __init__(self, parent, scoped_variable_m):
        super(ScopedDataOutputPortView, self).__init__(False, parent, scoped_variable_m, SnappedSide.RIGHT)


class DataPortView(PortView):

    def __init__(self, in_port, parent, port_m, side):
        assert isinstance(port_m, DataPortModel)
        super(DataPortView, self).__init__(in_port=in_port, name=port_m.data_port.name, parent=parent, side=side)

        self._port_m = ref(port_m)
        self.sort = port_m.data_port.data_port_id

    @property
    def port_m(self):
        return self._port_m()

    @property
    def port_id(self):
        return self.port_m.data_port.data_port_id

    @property
    def name(self):
        return self.port_m.data_port.name

    def draw(self, context, state):
        self.draw_port(context, "#ffc926")


class InputPortView(DataPortView):

    def __init__(self, parent, port_m):
        super(InputPortView, self).__init__(True, parent, port_m, SnappedSide.LEFT)

    # def add_connected_handle(self, handle, connection_view, moving=False):
    #     from awesome_tool.mvc.views.gap.connection import ConnectionView
    #     assert isinstance(handle, Handle)
    #     assert isinstance(connection_view, ConnectionView)
    #     if handle not in self._connected_handles.iterkeys() and not moving:
    #         if connection_view.to_handle() is handle:
    #             self._connected_handles[handle] = True
    #         else:
    #             self._connected_handles[handle] = False

    # def has_incoming_connection(self):
    #     return super(InputPortView, self).has_outgoing_connection()


class OutputPortView(DataPortView):

    def __init__(self, parent, port_m):
        super(OutputPortView, self).__init__(False, parent, port_m, SnappedSide.RIGHT)

    # def add_connected_handle(self, handle, connection_view, moving=False):
    #     from awesome_tool.mvc.views.gap.connection import ConnectionView
    #     assert isinstance(handle, Handle)
    #     assert isinstance(connection_view, ConnectionView)
    #     if handle not in self._connected_handles.iterkeys() and not moving:
    #         if connection_view.to_handle() is handle:
    #             self._connected_handles[handle] = True
    #         else:
    #             self._connected_handles[handle] = False

    # def has_incoming_connection(self):
    #     return super(OutputPortView, self).has_outgoing_connection()