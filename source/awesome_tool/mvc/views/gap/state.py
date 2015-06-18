from weakref import ref

import cairo
# import pango
from pangocairo import CairoContext
from pango import SCALE, FontDescription

from gtk.gdk import CairoContext, Color

from copy import copy

from gaphas.item import Element, NW, NE, SW, SE
from gaphas.connector import Position
from gaphas.painter import CairoBoundingBoxContext

from gaphas.util import text_align, text_set_font, text_extents

from awesome_tool.mvc.views.gap.constraint import KeepRectangleWithinConstraint, PortRectConstraint
from awesome_tool.mvc.views.gap.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, SnappedSide
from awesome_tool.mvc.views.gap.scope import ScopedVariableView

from awesome_tool.mvc.models.state import StateModel

from awesome_tool.statemachine.enums import StateExecutionState

from awesome_tool.utils import constants, log
logger = log.get_logger(__name__)


class StateView(Element):
    """ A State has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    def __init__(self, state_m, size, hierarchy_level):
        super(StateView, self).__init__(size[0], size[1])
        assert isinstance(state_m, StateModel)

        self._state_m = ref(state_m)
        self.hierarchy_level = hierarchy_level

        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

        self._left_center = Position((0, self.height / 2.))
        self._right_center = Position((self.width, self.height / 2.))
        self.constraint(line=(self._left_center, (self._handles[NW].pos, self._handles[SW].pos)), align=0.5)
        self.constraint(line=(self._right_center, (self._handles[NE].pos, self._handles[SE].pos)), align=0.5)

        self._income = None
        self._outcomes = []
        self._inputs = []
        self._outputs = []
        self._scoped_variables = []

        self.keep_rect_constraints = {}
        self.port_constraints = {}

        self.hovered = False
        self.selected = False

        if not isinstance(state_m.meta['name']['gui']['editor']['size'], tuple):
            name_width = self.width * 0.8
            name_height = self.height * 0.2
            name_size = (name_width, name_height)
        else:
            name_size = state_m.meta['name']['gui']['editor']['size']

        self._name_view = NameView(state_m.state.name, name_size)

        if isinstance(state_m.meta['name']['gui']['editor']['rel_pos'], tuple):
            name_pos = state_m.meta['name']['gui']['editor']['rel_pos']
            self._name_view.matrix.translate(*name_pos)

    def setup_canvas(self):
        self._income = self.add_income()

        canvas = self.canvas
        parent = canvas.get_parent(self)
        canvas.add(self._name_view, self)

        self.add_keep_rect_within_constraint(canvas, self, self._name_view)

        if parent is not None:
            assert isinstance(parent, StateView)
            self.add_keep_rect_within_constraint(canvas, parent, self)

        # Registers local constraints
        super(StateView, self).setup_canvas()

    def get_all_ports(self):
        port_list = [self.income]
        port_list += self.outcomes
        port_list += self.inputs
        port_list += self.outputs
        port_list += self.scoped_variables
        return port_list

    def get_logic_ports(self):
        port_list = [self.income]
        port_list += self.outcomes
        return port_list

    def get_data_ports(self):
        port_list = self.inputs
        port_list += self.outputs
        port_list += self.scoped_variables
        return port_list

    @staticmethod
    def add_keep_rect_within_constraint(canvas, parent, child):
        solver = canvas.solver
        port_side_size = min(parent.width, parent.height) / 20.

        child_nw_abs = canvas.project(child, child.handles()[NW].pos)
        child_se_abs = canvas.project(child, child.handles()[SE].pos)
        parent_nw_abs = canvas.project(parent, parent.handles()[NW].pos)
        parent_se_abs = canvas.project(parent, parent.handles()[SE].pos)
        constraint = KeepRectangleWithinConstraint(parent_nw_abs, parent_se_abs, child_nw_abs, child_se_abs, child, port_side_size)
        solver.add_constraint(constraint)
        parent.keep_rect_constraints[child] = constraint

    def remove_keep_rect_within_constraint_from_parent(self):
        canvas = self.canvas
        parent = canvas.get_parent(self)

        if parent is not None and isinstance(parent, StateView):
            constraint = parent.keep_rect_constraints[self]
            solver = canvas.solver
            solver.remove_constraint(constraint)

    @property
    def port_side_size(self):
        return min(self.width, self.height) / 20.

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    @property
    def corner_handles(self):
        return [self.handles()[NW], self.handles()[NE], self.handles()[SW], self.handles()[SE]]

    @property
    def model(self):
        return self._state_m()

    @property
    def income(self):
        return self._income

    @property
    def outcomes(self):
        return self._outcomes

    @property
    def outputs(self):
        return self._outputs

    @property
    def inputs(self):
        return copy(self._inputs)

    @property
    def scoped_variables(self):
        return self._scoped_variables

    @property
    def name_view(self):
        return self._name_view

    @staticmethod
    def get_state_drawing_area(state):
        assert isinstance(state, StateView)
        port_side_size = min(state.width, state.height) / 20.

        state_nw_pos_x = state.handles()[NW].pos.x + port_side_size
        state_nw_pos_y = state.handles()[NW].pos.y + port_side_size
        state_nw_pos = Position((state_nw_pos_x, state_nw_pos_y))
        state_se_pos_x = state.handles()[SE].pos.x - port_side_size
        state_se_pos_y = state.handles()[SE].pos.y - port_side_size
        state_se_pos = Position((state_se_pos_x, state_se_pos_y))

        return state_nw_pos, state_se_pos

    def draw(self, context):
        c = context.cairo

        c.set_line_width(0.1 / self.hierarchy_level)
        nw = self._handles[NW].pos
        c.rectangle(nw.x, nw.y, self.width, self.height)
        # if context.hovered:
        #     c.set_source_rgba(.8, .8, 1, .8)
        # else:
        if self.selected:
            c.set_source_color(Color('#2e9aff'))
        elif self.model.state.active:
            c.set_source_color(Color('#5b8d4f'))
        else:
            c.set_source_color(Color('#50555F'))
        c.fill_preserve()
        c.set_source_color(Color('#050505'))
        c.stroke()

        inner_nw, inner_se = self.get_state_drawing_area(self)
        c.rectangle(inner_nw.x, inner_nw.y, inner_se.x - inner_nw.x, inner_se.y - inner_nw.y)
        c.set_source_color(Color('#383D47'))
        c.fill_preserve()
        c.set_source_color(Color('#050505'))
        c.stroke()

        self._income.draw(context, self)

        for outcome in self._outcomes:
            outcome.draw(context, self)

        for input in self._inputs:
            input.draw(context, self)

        for output in self._outputs:
            output.draw(context, self)

    def connect_to_income(self, item, handle):
        self._income.add_connected_handle(handle, item)
        item.set_port_for_handle(self._income, handle)
        self._connect_to_port(self._income.port, item, handle)

    def connect_to_outcome(self, outcome_id, item, handle):
        outcome_v = self.outcome_port(outcome_id)
        outcome_v.add_connected_handle(handle, item)
        item.set_port_for_handle(outcome_v, handle)
        self._connect_to_port(outcome_v.port, item, handle)

    def connect_to_input_port(self, port_id, item, handle):
        port_v = self.input_port(port_id)
        port_v.add_connected_handle(handle, item)
        item.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, item, handle)

    def connect_to_output_port(self, port_id, item, handle):
        port_v = self.output_port(port_id)
        port_v.add_connected_handle(handle, item)
        item.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, item, handle)

    def connect_to_scoped_variable_input(self, scoped_variable_id, item, handle):
        scoped_variable_v = self.scoped_variable(scoped_variable_id)
        scoped_variable_v.input_port.add_connected_handle(handle, item)
        item.set_port_for_handle(scoped_variable_v.input_port, handle)
        c = scoped_variable_v.input_port_port.constraint(self.canvas, item, handle, scoped_variable_v)
        self.canvas.connect_item(item, handle, scoped_variable_v, scoped_variable_v.input_port_port, c)

    def connect_to_scoped_variable_output(self, scoped_variable_id, item, handle):
        scoped_variable_v = self.scoped_variable(scoped_variable_id)
        scoped_variable_v.output_port.add_connected_handle(handle, item)
        item.set_port_for_handle(scoped_variable_v.output_port, handle)
        c = scoped_variable_v.output_port_port.constraint(self.canvas, item, handle, scoped_variable_v)
        self.canvas.connect_item(item, handle, scoped_variable_v, scoped_variable_v.output_port_port, c)

    def _connect_to_port(self, port, item, handle):
        c = port.constraint(self.canvas, item, handle, self)
        self.canvas.connect_item(item, handle, self, port, c)

    def income_port(self):
        return self._income

    def outcome_port(self, outcome_id):
        for outcome in self._outcomes:
            if outcome.outcome_id == outcome_id:
                return outcome
        raise AttributeError("Outcome with id '{0}' not found in state".format(outcome_id, self.model.state.name))

    def input_port(self, port_id):
        return self._data_port(self._inputs, port_id)

    def output_port(self, port_id):
        return self._data_port(self._outputs, port_id)

    def scoped_variable(self, scoped_variable_id):
        return self._data_port(self._scoped_variables, scoped_variable_id)

    def _data_port(self, port_list, port_id):
        for port in port_list:
            if port.port_id == port_id:
                return port
        raise AttributeError("Port with id '{0}' not found in state".format(port_id, self.model.state.name))

    def add_income(self):
        income_v = IncomeView(self)
        self._ports.append(income_v.port)
        self._handles.append(income_v.handle)

        port_meta = self.model.meta['income']['gui']['editor']
        if isinstance(port_meta['rel_pos'], tuple):
            income_v.handle.pos = port_meta['rel_pos']
        else:
            income_v.handle.pos = 0, self.height * .05
        self.add_rect_constraint_for_port(income_v)
        return income_v

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m, self)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)

        port_meta = self.model.meta['outcome%d' % outcome_v.outcome_id]['gui']['editor']
        if isinstance(port_meta['rel_pos'], tuple):
            outcome_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(outcome_v.port_side_size, SnappedSide.RIGHT,
                                                                     outcome_v)
            if pos_x is None:
                self.model.state.remove_outcome(outcome_v.outcome_id)
                return
            outcome_v.handle.pos = pos_x, pos_y
            outcome_v.side = side
        self.add_rect_constraint_for_port(outcome_v)

    def remove_outcome(self, outcome_v):
        self._outcomes.remove(outcome_v)
        self._ports.remove(outcome_v.port)
        self._handles.remove(outcome_v.handle)

        if outcome_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints[outcome_v])

    def add_input_port(self, port_m):
        input_port_v = InputPortView(self, port_m)
        self._inputs.append(input_port_v)
        self._ports.append(input_port_v.port)
        self._handles.append(input_port_v.handle)

        port_meta = self.model.meta['input%d' % input_port_v.port_id]['gui']['editor']
        if isinstance(port_meta['rel_pos'], tuple):
            input_port_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(input_port_v.port_side_size, SnappedSide.LEFT,
                                                                     input_port_v, logic=False, in_port=True)
            if pos_x is None:
                self.model.state.remove_input_data_port(input_port_v.port_id)
                return
            input_port_v.handle.pos = pos_x, pos_y
            input_port_v.side = side
        self.add_rect_constraint_for_port(input_port_v)

    def remove_input_port(self, input_port_v):
        self._inputs.remove(input_port_v)
        self._ports.remove(input_port_v.port)
        self._handles.remove(input_port_v.handle)

        if input_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints[input_port_v])

    def add_output_port(self, port_m):
        output_port_v = OutputPortView(self, port_m)
        self._outputs.append(output_port_v)
        self._ports.append(output_port_v.port)
        self._handles.append(output_port_v.handle)

        port_meta = self.model.meta['output%d' % output_port_v.port_id]['gui']['editor']
        if isinstance(port_meta['rel_pos'], tuple):
            output_port_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(output_port_v.port_side_size, SnappedSide.RIGHT,
                                                                     output_port_v, logic=False)
            if pos_x is None:
                self.model.state.remove_output_data_port(output_port_v.port_id)
                return
            output_port_v.handle.pos = pos_x, pos_y
            output_port_v.side = side
        self.add_rect_constraint_for_port(output_port_v)

    def remove_output_port(self, output_port_v):
        self._outputs.remove(output_port_v)
        self._ports.remove(output_port_v.port)
        self._handles.remove(output_port_v.handle)

        if output_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints[output_port_v])

    def add_scoped_variable(self, scoped_variable_m, size):
        scoped_variable_v = ScopedVariableView(scoped_variable_m, size, self)
        self._scoped_variables.append(scoped_variable_v)

        canvas = self.canvas
        canvas.add(scoped_variable_v, self)

        self.add_keep_rect_within_constraint(canvas, self, scoped_variable_v)

        return scoped_variable_v

    def add_rect_constraint_for_port(self, port):
        constraint = PortRectConstraint((self.handles()[NW].pos, self.handles()[SE].pos), port.pos, port)
        solver = self.canvas.solver
        solver.add_constraint(constraint)
        self.port_constraints[port] = constraint

    def _calculate_unoccupied_position(self, port_size, side, new_port, logic=True, in_port=False):
        if in_port:
            new_pos_x = 0
        else:
            new_pos_x = self.width
        if logic:
            new_pos_y = self.height * .05
        else:
            new_pos_y = self.height * .95

        position_found = False
        all_ports = self.get_all_ports()
        new_pos = (new_pos_x, new_pos_y)

        port_limitation_counter = 0

        while not position_found and port_limitation_counter < 100:
            position_found, new_pos_x, new_pos_y, side = self._check_pos(all_ports, port_size, side, new_port, new_pos, logic, in_port)
            new_pos = (new_pos_x, new_pos_y)
            port_limitation_counter += 1

        if port_limitation_counter == 100:
            logger.warn("Cannot add new port, as there is no space left in State. Please rearrange or delete ports and "
                        "try again.")
            return None, None, None

        return new_pos_x, new_pos_y, side

    def _check_pos(self, all_ports, port_size, side, new_port, new_pos, logic, in_port):
        new_pos_x = new_pos[0]
        new_pos_y = new_pos[1]
        new_port_pos_top = new_pos_y - port_size / 2.
        new_port_pos_bot = new_pos_y + port_size / 2.
        new_port_pos_left = new_pos_x - port_size / 2.
        new_port_pos_right = new_pos_x + port_size / 2.

        def adjust_pos(width, height):
            if side is SnappedSide.RIGHT and new_port_pos_bot > height:
                return False, new_pos_x - 2 * port_size, height, side.next()
            elif side is SnappedSide.RIGHT and new_port_pos_top < 0:
                return False, new_pos_x - 2 * port_size, 0, side.prev()
            elif side is SnappedSide.LEFT and new_port_pos_bot > height:
                return False, new_pos_x + 2 * port_size, height, side.prev()
            elif side is SnappedSide.LEFT and new_port_pos_top < 0:
                return False, new_pos_x + 2 * port_size, 0, side.next()
            elif side is SnappedSide.TOP and new_port_pos_right > width:
                return False, width, new_pos_y + 2 * port_size, side.next()
            elif side is SnappedSide.TOP and new_port_pos_left < 0:
                return False, 0, new_pos_y + 2 * port_size, side.prev()
            elif side is SnappedSide.BOTTOM and new_port_pos_right > width:
                return False, width, new_pos_y - 2 * port_size, side.prev()
            elif side is SnappedSide.BOTTOM and new_port_pos_left < 0:
                return False, 0, new_pos_y - 2 * port_size, side.next()

        for port in all_ports:

            if port is not new_port and port.side is side:

                if side is SnappedSide.RIGHT or side is SnappedSide.LEFT:
                    port_pos = port.pos
                    port_pos_top = port_pos.y.value - port_size / 2.
                    port_pos_bot = port_pos.y.value + port_size / 2.

                    if port_pos_top == new_port_pos_top and port_pos_bot == port_pos_bot:
                        new_pos_y = self._move_pos_one_step(new_pos_y, port_size, side, logic, in_port)
                        break
                    elif port_pos_top < new_port_pos_bot < port_pos_bot:
                        new_pos_y = self._move_pos_one_step(new_pos_y, port_size, side, logic, in_port)
                        break
                    elif port_pos_top < new_port_pos_top < port_pos_bot:
                        new_pos_y = self._move_pos_one_step(new_pos_y, port_size, side, logic, in_port)
                        break
                elif side is SnappedSide.TOP or side is SnappedSide.BOTTOM:
                    port_pos = port.pos
                    port_pos_left = port_pos.x.value - port_size / 2.
                    port_pos_right = port_pos.x.value + port_size / 2.

                    if port_pos_left == new_port_pos_left and port_pos_right == new_port_pos_right:
                        new_pos_x = self._move_pos_one_step(new_pos_x, port_size, side, logic, in_port)
                        break
                    elif port_pos_left < new_port_pos_right < port_pos_right:
                        new_pos_x = self._move_pos_one_step(new_pos_x, port_size, side, logic, in_port)
                        break
                    elif port_pos_left < new_port_pos_left < port_pos_right:
                        new_pos_x = self._move_pos_one_step(new_pos_x, port_size, side, logic, in_port)
                        break

        if new_pos_x != new_pos[0] or new_pos_y != new_pos[1]:
            res = adjust_pos(self.width, self.height)
            if res:
                return res
            return False, new_pos_x, new_pos_y, side

        res = adjust_pos(self.width, self.height)
        if res:
            return res

        return True, new_pos_x, new_pos_y, side

    @staticmethod
    def _move_pos_one_step(pos, port_size, side, logic, in_port):
        if side is SnappedSide.RIGHT:
            if logic or (not logic and in_port):
                return pos + 1.25 * port_size
            else:
                return pos - 1.25 * port_size
        elif side is SnappedSide.LEFT:
            if logic or (not logic and in_port):
                return pos - 1.25 * port_size
            else:
                return pos + 1.25 * port_size
        elif side is SnappedSide.BOTTOM:
            if not in_port and not logic:
                return pos + 1.25 * port_size
            else:
                return pos - 1.25 * port_size
        elif side is SnappedSide.TOP:
            if not in_port and not logic:
                return pos - 1.25 * port_size
            else:
                return pos + 1.25 * port_size


class NameView(Element):

    def __init__(self, name, size):
        super(NameView, self).__init__(size[0], size[1])

        self._name = None
        self.name = name

        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        assert isinstance(name, str)
        self._name = name

    def draw(self, context):
        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        # c.move_to(*self.)

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
        while layout.get_size()[0] / float(SCALE) > self.width or layout.get_size()[1] / float(SCALE) > self.height:
            font_size *= 0.9
            set_font_description()

        c.move_to(0, 0)
        cc.set_source_color(Color('#ededee'))
        pcc.update_layout(layout)
        pcc.show_layout(layout)