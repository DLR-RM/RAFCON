from weakref import ref

from gaphas.connector import PointPort, Handle
from gaphas.util import path_ellipse

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.data_port import DataPortModel

import cairo
from pango import SCALE, FontDescription
from gtk.gdk import Color, CairoContext

from awesome_tool.utils import constants

import copy

from enum import Enum

SnappedSide = Enum('SIDE', 'LEFT TOP RIGHT BOTTOM')


class PortView(object):

    def __init__(self, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = PointPort(self.handle.pos)
        self.side = side
        if parent:
            self.outcome_side_size = min(parent.width, parent.height) / 20.
        else:
            self.outcome_side_size = 5.

    @property
    def pos(self):
        return self.handle.pos

    @property
    def handle_pos(self):
        return self.handle.pos

    @property
    def port_pos(self):
        return self.port.point

    def draw(self, context, state):
        raise NotImplementedError

    def draw_port(self, context, state, fill_color):
        c = context.cairo
        outcome_side = self.outcome_side_size
        c.set_line_width(outcome_side * 0.03)

        # Outer part
        c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        # Inner part
        c.rectangle(self.pos.x - outcome_side * 0.85 / 2, self.pos.y - outcome_side * 0.85 / 2, outcome_side * 0.85, outcome_side * 0.85)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_color(Color('#000000'))
        c.stroke()


class DoublePortView(object):

    def __init__(self):
        self.left_handle = Handle(connectable=True, movable=False)
        left_port_pos = copy.deepcopy(self.left_handle.pos)
        self.left_port = PointPort(left_port_pos)

        self.right_handle = Handle(connectable=True, movable=False)
        right_port_pos = copy.deepcopy(self.right_handle.pos)
        self.right_port = PointPort(right_port_pos)

    @property
    def left_handle_pos(self):
        return self.left_handle.pos

    @property
    def left_port_pos(self):
        return self.left_port.point

    @property
    def right_handle_pos(self):
        return self.right_handle.pos

    @property
    def right_port_pos(self):
        return self.right_port.point

    def draw(self, context, state):
        raise NotImplementedError

    def draw_view(self, context, state, fill_color, name):
        c = context.cairo
        min_state_side = min(state.width, state.height)
        port_side_size = min_state_side / 20.
        c.set_line_width(port_side_size * .03)

        name_width = self.right_handle_pos.x - self.left_handle_pos.x - port_side_size

        c.rectangle(self.left_handle_pos.x - port_side_size / 2,
                    self.left_handle_pos.y - port_side_size / 2,
                    2 * port_side_size + name_width,
                    port_side_size)
        c.set_source_color(Color('#50555f'))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        c.move_to(self.left_handle_pos.x + port_side_size / 2 + port_side_size * .05, self.left_handle_pos.y - port_side_size / 4)

        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        pcc = CairoContext(cc)
        pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()
        layout.set_text(name)

        font_name = constants.FONT_NAMES[0]
        font_size = 20

        def set_font_description():
            font = FontDescription(font_name + " " + str(font_size))
            layout.set_font_description(font)

        set_font_description()
        while layout.get_size()[0] / float(SCALE) > name_width - port_side_size * .1:
            font_size *= 0.9
            set_font_description()

        cc.set_source_color(Color('#ededee'))
        pcc.update_layout(layout)
        pcc.show_layout(layout)

        c.move_to(0, 0)

        # Outer parts
        c.rectangle(self.left_handle_pos.x - port_side_size / 4, self.left_handle_pos.y - port_side_size / 4,
                    port_side_size / 2, port_side_size / 2)
        c.rectangle(self.right_handle_pos.x - port_side_size / 4, self.right_handle_pos.y - port_side_size / 4,
                    port_side_size / 2, port_side_size / 2)
        c.set_source_color(Color(fill_color))
        c.fill_preserve()
        c.set_source_rgba(0, 0, 0, 0)
        c.stroke()

        # Inner parts
        c.rectangle(self.left_handle_pos.x - port_side_size * 0.85 / 4,
                    self.left_handle_pos.y - port_side_size * 0.85 / 4,
                    port_side_size * 0.85 / 2, port_side_size * 0.85 / 2)
        c.rectangle(self.right_handle_pos.x - port_side_size * 0.85 / 4,
                    self.right_handle_pos.y - port_side_size * 0.85 / 4,
                    port_side_size * 0.85 / 2, port_side_size * 0.85 / 2)
        c.set_source_color(Color(fill_color))
        # c.set_source_color(Color('#000000'))
        c.fill_preserve()
        c.set_source_color(Color('#000000'))
        c.stroke()


class IncomeView(PortView):

    def __init__(self, parent):
        super(IncomeView, self).__init__(parent)

    def draw(self, context, state):
        self.draw_port(context, state, "#ffffff")


class OutcomeDoublePortView(DoublePortView):

    def __init__(self, outcome_m):
        super(OutcomeDoublePortView, self).__init__()

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
    def outcome_name(self):
        return self.outcome_m.outcome.name

    def draw(self, context, state):
        if self.outcome_id == -2:
            fill_color = '#0000ff'
        elif self.outcome_id == -1:
            fill_color = '#ff0000'
        else:
            fill_color = '#ffffff'

        self.draw_view(context, state, fill_color, self.outcome_name)


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

    def __init__(self, port_m, parent):
        super(DataPortView, self).__init__(parent)

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
    pass


class OutputPortView(DataPortView):
    pass