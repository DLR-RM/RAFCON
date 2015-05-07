from weakref import ref

from gaphas.item import Element, NW, NE, SW, SE
from gaphas.connector import PointPort, Handle

from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel


class ScopedVariableView(Element):
    """ A scoped variable has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    def __init__(self, scoped_variable_m, size):
        super(ScopedVariableView, self).__init__(size[0], size[1])
        assert isinstance(scoped_variable_m, ScopedVariableModel)

        self._scoped_variable_m = ref(scoped_variable_m)

        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

        left_center = (0, self.height / 2.)
        right_center = (self.width, self.height / 2.)
        
        input_handle = Handle(left_center, connectable=True, movable=False)
        self._handles.append(input_handle)
        self._input_handle = input_handle
        input_port = PointPort(input_handle.pos)
        self._input_port = input_port
        self._ports.append(input_port)
        
        output_handle = Handle(right_center, connectable=True, movable=False)
        self._handles.append(output_handle)
        self._output_handle = output_handle
        output_port = PointPort(output_handle.pos)
        self._output_port = output_port
        self._ports.append(output_port)

        self.constraint(line=(input_handle.pos, (self._handles[NW].pos, self._handles[SW].pos)), align=0.5)
        self.constraint(line=(output_handle.pos, (self._handles[NE].pos, self._handles[SE].pos)), align=0.5)

    @property
    def input_port(self):
        return self._input_port

    @property
    def output_port(self):
        return self._output_port

    @property
    def port_id(self):
        return self._scoped_variable_m().scoped_variable.data_port_id

    def draw(self, context):
        c = context.cairo
        c.set_line_width(0.5)
        nw = self._handles[NW].pos
        c.rectangle(nw.x, nw.y, self.width, self.height)
        if context.hovered:
            c.set_source_rgba(.8, .8, 1, .8)
        else:
            c.set_source_rgba(1, 1, 1, .8)
        c.fill_preserve()
        c.set_source_rgb(0, 0, 0.8)
        c.stroke()

        min_variable_side = min(self.width, self.height)
        port_side = min_variable_side / 25.
        c.set_line_width(0.15)
        c.rectangle(self._input_handle.pos.x - port_side / 2, self._input_handle.pos.y - port_side / 2,
                    port_side, port_side)
        c.rectangle(self._output_handle.pos.x - port_side / 2, self._output_handle.pos.y - port_side / 2,
                    port_side, port_side)