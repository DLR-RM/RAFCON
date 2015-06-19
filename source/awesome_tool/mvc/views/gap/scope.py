from weakref import ref

import cairo

from gaphas.item import Element, NW, NE, SW, SE

from gtk.gdk import Color, CairoContext
from pango import SCALE, FontDescription

from awesome_tool.mvc.views.gap.ports import ScopedDataInputPortView, ScopedDataOutputPortView

from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel

from awesome_tool.utils import constants


class ScopedVariableView(Element):
    """ A scoped variable has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    def __init__(self, scoped_variable_m, size, parent_state):
        super(ScopedVariableView, self).__init__(size[0], size[1])
        assert isinstance(scoped_variable_m, ScopedVariableModel)

        self._scoped_variable_m = ref(scoped_variable_m)

        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

        self._parent_state = parent_state
        self._hierarchy_level = parent_state.hierarchy_level

        input_port = ScopedDataInputPortView(self, scoped_variable_m)
        # input_port.handle.movable = False
        self._input_port = input_port
        self._handles.append(input_port.handle)
        self._ports.append(input_port.port)

        output_port = ScopedDataOutputPortView(self, scoped_variable_m)
        # output_port.handle.movable = False
        self._output_port = output_port
        self._handles.append(output_port.handle)
        self._ports.append(output_port.port)

        self._port_side_size = min(self.width, self.height) / 5.

        self.constraint(line=(input_port.handle_pos, (self._handles[NW].pos, self._handles[SW].pos)), align=0.5)
        self.constraint(line=(output_port.handle_pos, (self._handles[NE].pos, self._handles[SE].pos)), align=0.5)

    @property
    def hierarchy_level(self):
        return self._hierarchy_level

    @property
    def parent_state(self):
        return self._parent_state

    @property
    def input_port(self):
        return self._input_port

    @property
    def output_port(self):
        return self._output_port

    @property
    def input_port_port(self):
        return self._input_port.port

    @property
    def output_port_port(self):
        return self._output_port.port

    @property
    def model(self):
        return self._scoped_variable_m()

    @property
    def port_id(self):
        return self._scoped_variable_m().scoped_variable.data_port_id

    @property
    def name(self):
        return self._scoped_variable_m().scoped_variable.name

    def draw(self, context):
        c = context.cairo

        c.set_line_width(0.1 / self._hierarchy_level)
        nw = self._handles[NW].pos
        c.rectangle(nw.x, nw.y, self.width, self.height)
        # if context.hovered:
        #     c.set_source_rgba(.8, .8, 1, .8)
        # else:
        c.set_source_color(Color('#50555F'))
        c.fill_preserve()
        c.set_source_color(Color('#050505'))
        c.stroke()

        self.draw_name(context)

        self._input_port.draw(context, self)
        self._output_port.draw(context, self)

    def draw_name(self, context):
        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        c.move_to(self._port_side_size, self._port_side_size / 2.)

        pcc = CairoContext(cc)
        pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()
        layout.set_text(self.name)

        font_name = constants.FONT_NAMES[0]
        font_size = 20

        def set_font_description():
            font = FontDescription(font_name + " " + str(font_size))
            layout.set_font_description(font)

        set_font_description()
        while (layout.get_size()[0] / float(SCALE) > self.width - 2 * self._port_side_size or
                layout.get_size()[1] / float(SCALE) > self.height):
            font_size *= 0.9
            set_font_description()

        cc.set_source_color(Color('#ededee'))
        pcc.update_layout(layout)
        pcc.show_layout(layout)

    def remove_keep_rect_within_constraint_from_parent(self):
        from awesome_tool.mvc.views.gap.state import StateView
        canvas = self.canvas
        parent = canvas.get_parent(self)

        if parent is not None and isinstance(parent, StateView):
            constraint = parent.keep_rect_constraints[self]
            solver = canvas.solver
            solver.remove_constraint(constraint)