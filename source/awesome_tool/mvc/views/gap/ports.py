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

SnappedSide = Enum('SIDE', 'LEFT TOP RIGHT BOTTOM')


class PortView(object):

    def __init__(self, name=None, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = PointPort(self.handle.pos)
        self.side = side
        self._parent = parent

        self._connected_handles = {}
        self._tmp_connected = False

        self._name = name

        self.port_side_size = 0.
        self.update_port_side_size()

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

    @property
    def connected_handles(self):
        return self._connected_handles.iterkeys()

    def has_outgoing_connection(self):
        for outgoing_connection in self._connected_handles.itervalues():
            if outgoing_connection:
                return True
        return False

    def add_connected_handle(self, handle, connection_view, moving=False):
        from awesome_tool.mvc.views.gap.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if handle not in self._connected_handles.iterkeys() and not moving:
            if connection_view.from_handle() is handle:
                self._connected_handles[handle] = True
            else:
                self._connected_handles[handle] = False

    def remove_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle in self._connected_handles:
            self._connected_handles.pop(handle)

    def tmp_connect(self):
        self._tmp_connected = True

    def tmp_disconnect(self):
        self._tmp_connected = False

    @property
    def connected(self):
        if len(self._connected_handles) == 0:
            return self._tmp_connected
        return True

    def draw(self, context, state):
        raise NotImplementedError

    def draw_port(self, context, state, fill_color):
        self.update_port_side_size()
        c = context.cairo
        outcome_side = self.port_side_size
        c.set_line_width(outcome_side * 0.03)

        # Outer part
        c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        # Inner part
        c.rectangle(self.pos.x - outcome_side * 0.85 / 2, self.pos.y - outcome_side * 0.85 / 2, outcome_side * 0.85, outcome_side * 0.85)
        if self.connected:
            c.set_source_color(Color(fill_color))
        else:
            c.set_source_color(Color('#000'))
        c.fill_preserve()
        c.set_source_color(Color('#000'))
        c.stroke()

        if self._name and not self.has_outgoing_connection() and self.parent.parent:
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
        layout.set_text(self._name)

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

    def update_port_side_size(self):
        if self._parent:
            self.port_side_size = min(self._parent.width, self._parent.height) / 20.
        else:
            self.port_side_size = 5.


class IncomeView(PortView):

    def __init__(self, parent):
        super(IncomeView, self).__init__(parent=parent, side=SnappedSide.LEFT)

    def draw(self, context, state):
        self.draw_port(context, state, "#aaaaaa")


class OutcomeView(PortView):

    def __init__(self, outcome_m, parent):
        super(OutcomeView, self).__init__(name=outcome_m.outcome.name, parent=parent)

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
            fill_color = '#0000ff'
        elif self.outcome_id == -1:
            fill_color = '#ff0000'
        else:
            fill_color = '#ffffff'

        self.draw_port(context, state, fill_color)


class ScopedDataPortView(PortView):

    def __init__(self, parent, scoped_variable_m, side):
        super(ScopedDataPortView, self).__init__(parent=parent, side=side)

        assert isinstance(scoped_variable_m, ScopedVariableModel)
        self._scoped_variable_m = ref(scoped_variable_m)

    @property
    def scoped_variable_m(self):
        return self._scoped_variable_m()

    @property
    def port_id(self):
        return self.scoped_variable_m.scoped_variable.data_port_id

    def draw(self, context, state):
        fill_color = "#ffc926"

        self.update_port_side_size()
        c = context.cairo
        outcome_side = self.port_side_size
        c.set_line_width(outcome_side * 0.03)

        # Outer part
        c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        # Inner part
        c.rectangle(self.pos.x - outcome_side * 0.85 / 2, self.pos.y - outcome_side * 0.85 / 2, outcome_side * 0.85, outcome_side * 0.85)
        if self.connected:
            c.set_source_color(Color(fill_color))
        else:
            c.set_source_color(Color('#000'))
        c.fill_preserve()
        c.set_source_color(Color('#000'))
        c.stroke()

    def update_port_side_size(self):
        if self._parent:
            self.port_side_size = min(self._parent.width, self._parent.height) / 5.
        else:
            self.port_side_size = 5.


class ScopedDataInputPortView(ScopedDataPortView):

    def __init__(self, parent, scoped_variable_m):
        super(ScopedDataInputPortView, self).__init__(parent, scoped_variable_m, SnappedSide.LEFT)


class ScopedDataOutputPortView(ScopedDataPortView):

    def __init__(self, parent, scoped_variable_m):
        super(ScopedDataOutputPortView, self).__init__(parent, scoped_variable_m, SnappedSide.RIGHT)


class DataPortView(PortView):

    def __init__(self, parent, port_m, side):
        assert isinstance(port_m, DataPortModel)
        super(DataPortView, self).__init__(name=port_m.data_port.name, parent=parent, side=side)

        self._port_m = ref(port_m)
        self.sort = port_m.data_port.data_port_id

    @property
    def port_m(self):
        return self._port_m()

    @property
    def port_id(self):
        return self.port_m.data_port.data_port_id

    def draw(self, context, state):
        self.draw_port(context, state, "#ffc926")


class InputPortView(DataPortView):

    def __init__(self, parent, port_m):
        super(InputPortView, self).__init__(parent, port_m, SnappedSide.LEFT)

    def add_connected_handle(self, handle, connection_view, moving=False):
        from awesome_tool.mvc.views.gap.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if handle not in self._connected_handles.iterkeys() and not moving:
            if connection_view.to_handle() is handle:
                self._connected_handles[handle] = True
            else:
                self._connected_handles[handle] = False

    def has_incoming_connection(self):
        return super(InputPortView, self).has_outgoing_connection()


class OutputPortView(DataPortView):

    def __init__(self, parent, port_m):
        super(OutputPortView, self).__init__(parent, port_m, SnappedSide.RIGHT)

    def add_connected_handle(self, handle, connection_view, moving=False):
        from awesome_tool.mvc.views.gap.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if handle not in self._connected_handles.iterkeys() and not moving:
            if connection_view.to_handle() is handle:
                self._connected_handles[handle] = True
            else:
                self._connected_handles[handle] = False

    def has_incoming_connection(self):
        return super(OutputPortView, self).has_outgoing_connection()