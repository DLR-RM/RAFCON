from weakref import ref

from gaphas.connector import PointPort, Handle

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.data_port import DataPortModel

import cairo
from pango import SCALE, FontDescription
from gtk.gdk import Color, CairoContext

from enum import Enum

SnappedSide = Enum('SIDE', 'LEFT TOP RIGHT BOTTOM')


class PortView(object):

    def __init__(self, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = PointPort(self.handle.pos)
        self.side = side
        self._parent = parent

        self._connected_handles = []

        self.port_side_size = 0.
        self.update_port_side_size()

    @property
    def pos(self):
        return self.handle.pos

    @property
    def handle_pos(self):
        return self.handle.pos

    @property
    def port_pos(self):
        return self.port.point

    def add_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle not in self._connected_handles:
            self._connected_handles.append(handle)

    def remove_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle in self._connected_handles:
            self._connected_handles.remove(handle)

    @property
    def connected(self):
        return len(self._connected_handles) > 0

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

    def update_port_side_size(self):
        if self._parent:
            self.port_side_size = min(self._parent.width, self._parent.height) / 20.
        else:
            self.port_side_size = 5.


class IncomeView(PortView):

    def __init__(self, parent):
        super(IncomeView, self).__init__(parent, SnappedSide.LEFT)

    def draw(self, context, state):
        self.draw_port(context, state, "#aaaaaa")


class OutcomeView(PortView):

    def __init__(self, outcome_m, parent):
        super(OutcomeView, self).__init__(parent)

        assert isinstance(outcome_m, OutcomeModel)
        self._outcome_m = ref(outcome_m)
        self.sort = outcome_m.outcome.outcome_id

    @property
    def outcome_m(self):
        return self._outcome_m()

    @property
    def outcome_id(self):
        return self.outcome_m.outcome.outcome_id

    def draw(self, context, state):
        if self.outcome_id == -2:
            fill_color = '#0000ff'
        elif self.outcome_id == -1:
            fill_color = '#ff0000'
        else:
            fill_color = '#ffffff'

        self.draw_port(context, state, fill_color)


class DataPortView(PortView):

    def __init__(self, parent, port_m, side):
        super(DataPortView, self).__init__(parent, side)

        assert isinstance(port_m, DataPortModel)
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


class OutputPortView(DataPortView):

    def __init__(self, parent, port_m):
        super(OutputPortView, self).__init__(parent, port_m, SnappedSide.RIGHT)