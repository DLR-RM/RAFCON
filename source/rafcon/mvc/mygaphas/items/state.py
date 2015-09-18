from weakref import ref
from pango import SCALE, FontDescription, WRAP_WORD
from math import pow
from copy import copy

from rafcon.utils import constants, log

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.models.state import StateModel
from rafcon.mvc.models.container_state import ContainerStateModel

import cairo
from gtk.gdk import CairoContext, Color
from gaphas.item import Element, NW, NE, SW, SE
from gaphas.connector import Position

from rafcon.mvc.mygaphas.constraint import KeepRectangleWithinConstraint, PortRectConstraint
from rafcon.mvc.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedVariablePortView
from rafcon.mvc.mygaphas.items.connection import TransitionView
from rafcon.mvc.mygaphas.utils.enums import SnappedSide
from rafcon.mvc.mygaphas.utils.gap_draw_helper import get_col_rgba
from rafcon.mvc.mygaphas.utils import gap_draw_helper
from rafcon.mvc.mygaphas.utils.cache.image_cache import ImageCache


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

        self._income = None
        self._outcomes = []
        self._inputs = []
        self._outputs = []
        self._scoped_variables = []
        self._scoped_variables_ports = []

        self.keep_rect_constraints = {}
        self.port_constraints = {}

        self.hovered = False
        self.selected = False
        self._moving = False
        self._transparent = False
        self._show_aborted_preempted = global_gui_config.get_config_value("SHOW_ABORTED_PREEMPTED", False)

        self.__symbol_size_cache = {}
        self._image_cache = ImageCache()

        if not isinstance(state_m.meta['name']['gui']['editor_gaphas']['size'], tuple):
            name_width = self.width * 0.8
            name_height = self.height * 0.4
            name_size = (name_width, name_height)
        else:
            name_size = state_m.meta['name']['gui']['editor_gaphas']['size']

        self._name_view = NameView(state_m.state.name, name_size)

        if isinstance(state_m.meta['name']['gui']['editor_gaphas']['rel_pos'], tuple):
            name_pos = state_m.meta['name']['gui']['editor_gaphas']['rel_pos']
            self.name_view.matrix.translate(*name_pos)

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
        port_side_size = parent.port_side_size

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

    def has_selected_child(self):
        for child in self.canvas.get_children(self):
            if isinstance(child, StateView) and child.selected:
                return True
        return False

    @property
    def show_aborted_preempted(self):
        return global_gui_config.get_config_value("SHOW_ABORTED_PREEMPTED", False)

    @property
    def show_data_port_label(self):
        return global_gui_config.get_config_value("SHOW_DATA_FLOWS")

    @property
    def moving(self):
        return self._moving

    @moving.setter
    def moving(self, moving):
        assert isinstance(moving, bool)
        self._moving = moving
        for child in self.canvas.get_children(self):
            if isinstance(child, (StateView, NameView)):
                child.moving = moving

    @property
    def port_side_size(self):
        # dynamic_width = min(self.width, self.height) / 20.
        return constants.ROOT_WIDTH / pow(constants.WIDTH_HIERARCHY_FACTOR, self.hierarchy_level - 1)

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    @property
    def corner_handles(self):
        return [self.handles()[NW], self.handles()[NE], self.handles()[SW], self.handles()[SE]]

    @property
    def aborted_preempted_handles(self):
        return [self.outcomes[-1].handle, self.outcomes[-2].handle]

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
        return self._scoped_variables_ports

    @property
    def name_view(self):
        return self._name_view

    @property
    def transparent(self):
        return self._transparent

    @property
    def child_state_vs(self):
        child_state_vs = []
        for child in self.canvas.get_children(self):
            if isinstance(child, StateView):
                child_state_vs.append(child)
        return child_state_vs

    @staticmethod
    def get_state_drawing_area(state):
        assert isinstance(state, StateView)
        port_side_size = state.port_side_size

        state_nw_pos_x = state.handles()[NW].pos.x + port_side_size
        state_nw_pos_y = state.handles()[NW].pos.y + port_side_size
        state_nw_pos = Position((state_nw_pos_x, state_nw_pos_y))
        state_se_pos_x = state.handles()[SE].pos.x - port_side_size
        state_se_pos_y = state.handles()[SE].pos.y - port_side_size
        state_se_pos = Position((state_se_pos_x, state_se_pos_y))

        return state_nw_pos, state_se_pos

    def foreground(self):
        self._transparent = False

    def background(self):
        self._transparent = True

    def draw(self, context):
        if self.moving and self.parent and self.parent.moving:
            return
        c = context.cairo

        nw = self._handles[NW].pos

        parameters = {
            'hierarchy': self.hierarchy_level,
            'active':  self.model.state.active,
            'selected': self.selected,
            'moving': self.moving,
            'port_side_size': self.port_side_size
        }

        current_zoom = self.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._image_cache.get_cached_image(self.width, self.height, current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "from cache"
            c.save()
            c.scale(1./zoom, 1./zoom)
            c.set_source_surface(image, int(nw.x.value), int(nw.y.value))
            c.paint()
            c.restore()

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw"
            cairo_context = cairo.Context(image)
            c = CairoContext(cairo_context)
            c.scale(current_zoom, current_zoom)

            c.set_line_width(0.1 / self.hierarchy_level)
            c.rectangle(nw.x, nw.y, self.width, self.height)

            if self.model.state.active:
                c.set_source_color(Color(constants.STATE_ACTIVE_COLOR))
            elif self.selected:
                c.set_source_color(Color(constants.STATE_SELECTED_COLOR))
            else:
                c.set_source_rgba(*get_col_rgba(Color(constants.STATE_BORDER_COLOR), self._transparent))
            c.fill_preserve()
            if self.model.state.active:
                c.set_source_color(Color(constants.STATE_ACTIVE_BORDER_COLOR))
                c.set_line_width(.25 / self.hierarchy_level)
            elif self.selected:
                c.set_source_color(Color(constants.STATE_SELECTED_OUTER_BOUNDARY_COLOR))
                c.set_line_width(.25 / self.hierarchy_level)
            else:
                c.set_source_color(Color(constants.BLACK_COLOR))
            c.stroke()
            c.set_line_width(0.1 / self.hierarchy_level)

            inner_nw, inner_se = self.get_state_drawing_area(self)
            c.rectangle(inner_nw.x, inner_nw.y, inner_se.x - inner_nw.x, inner_se.y - inner_nw.y)
            c.set_source_rgba(*get_col_rgba(Color(constants.STATE_BACKGROUND_COLOR)))
            c.fill_preserve()
            c.set_source_color(Color(constants.BLACK_COLOR))
            c.stroke()

            # Copy image surface to current cairo context
            context.cairo.save()
            context.cairo.scale(1. / current_zoom, 1. / current_zoom)
            context.cairo.set_source_surface(image, int(nw.x.value), int(nw.y.value))
            context.cairo.paint()
            context.cairo.restore()

        self._income.port_side_size = self.port_side_size
        self._income.draw(context, self)

        for outcome in self._outcomes:
            if not self.show_aborted_preempted and (outcome.outcome_id == -1 or outcome.outcome_id == -2):
                continue
            outcome.port_side_size = self.port_side_size
            outcome.draw(context, self)

        for input in self._inputs:
            input.port_side_size = self.port_side_size
            input.draw(context, self)

        for output in self._outputs:
            output.port_side_size = self.port_side_size
            output.draw(context, self)

        for scoped_variable in self._scoped_variables_ports:
            scoped_variable.port_side_size = self.port_side_size
            scoped_variable.draw(context, self)

        if isinstance(self.model.state, LibraryState) and not self.moving:
            max_width = self.width / 2.
            max_height = self.height / 2.
            self._draw_symbol(context, constants.SIGN_LIB, True, (max_width, max_height))

        if self.moving:
            max_width = self.width - 2 * self.port_side_size
            max_height = self.height - 2 * self.port_side_size
            self._draw_symbol(context, constants.SIGN_ARROW, False, (max_width, max_height))


    def _draw_symbol(self, context, symbol, is_library_state, max_size):
        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        pcc = CairoContext(cc)
        pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()

        font_name = constants.FONT_NAMES[1]

        def set_font_description():
            layout.set_markup('<span font_desc="%s %s">&#x%s;</span>' %
                              (font_name,
                               font_size,
                               symbol))

        if symbol in self.__symbol_size_cache and \
                self.__symbol_size_cache[symbol]['width'] == self.width and \
                self.__symbol_size_cache[symbol]['height'] == self.height:
            font_size = self.__symbol_size_cache[symbol]['size']
            set_font_description()

        else:
            font_size = 30
            set_font_description()

            pango_size = (self.width * SCALE, self.height * SCALE)
            while layout.get_size()[0] > pango_size[0] or layout.get_size()[1] > pango_size[1]:
                font_size *= 0.9
                set_font_description()

            self.__symbol_size_cache[symbol] = {'width': self.width, 'height': self.height, 'size': font_size}

        c.move_to(self.width / 2. - layout.get_size()[0] / float(SCALE) / 2.,
                  self.height / 2. - layout.get_size()[1] / float(SCALE) / 2.)

        alpha = 1.
        if is_library_state and self.transparent:
            alpha = 0.0625
        elif is_library_state:
            alpha = 0.25

        cc.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(constants.STATE_NAME_COLOR), is_library_state,
                                                         alpha=alpha))
        pcc.update_layout(layout)
        pcc.show_layout(layout)

    def get_transitions(self):
        transitions = []
        for child in self.canvas.get_children(self):
            if isinstance(child, TransitionView):
                transitions.append(child)
        return transitions

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

    def connect_to_scoped_variable_port(self, scoped_variable_id, item, handle):
        port_v = self.scoped_variable(scoped_variable_id)
        port_v.add_connected_handle(handle, item)
        item.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, item, handle)

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
        return self._data_port(self._scoped_variables_ports, scoped_variable_id)

    def _data_port(self, port_list, port_id):
        for port in port_list:
            if port.port_id == port_id:
                return port
        raise AttributeError("Port with id '{0}' not found in state".format(port_id, self.model.state.name))

    def add_income(self):
        income_v = IncomeView(self, self.port_side_size)
        self._ports.append(income_v.port)
        self._handles.append(income_v.handle)

        port_meta = self.model.meta['income']['gui']['editor_gaphas']
        if isinstance(port_meta['rel_pos'], tuple):
            income_v.handle.pos = port_meta['rel_pos']
        else:
            income_v.handle.pos = 0, self.height * .05
        self.add_rect_constraint_for_port(income_v)
        return income_v

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m, self, self.port_side_size)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)

        port_meta = self.model.meta['outcome%d' % outcome_v.outcome_id]['gui']['editor_gaphas']
        if isinstance(port_meta['rel_pos'], tuple):
            outcome_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(outcome_v.port_side_size, SnappedSide.RIGHT,
                                                                     outcome_v, logic=True)
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
        input_port_v = InputPortView(self, port_m, self.port_side_size)
        self._inputs.append(input_port_v)
        self._ports.append(input_port_v.port)
        self._handles.append(input_port_v.handle)

        port_meta = self.model.meta['input%d' % input_port_v.port_id]['gui']['editor_gaphas']
        if isinstance(port_meta['rel_pos'], tuple):
            input_port_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(input_port_v.port_side_size, SnappedSide.LEFT,
                                                                     input_port_v, in_port=True)
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
        output_port_v = OutputPortView(self, port_m, self.port_side_size)
        self._outputs.append(output_port_v)
        self._ports.append(output_port_v.port)
        self._handles.append(output_port_v.handle)

        port_meta = self.model.meta['output%d' % output_port_v.port_id]['gui']['editor_gaphas']
        if isinstance(port_meta['rel_pos'], tuple):
            output_port_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(output_port_v.port_side_size, SnappedSide.RIGHT,
                                                                     output_port_v)
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

    def add_scoped_variable(self, scoped_variable_m):
        scoped_variable_port_v = ScopedVariablePortView(self, self.port_side_size, scoped_variable_m)
        self._scoped_variables_ports.append(scoped_variable_port_v)
        self._ports.append(scoped_variable_port_v.port)
        self._handles.append(scoped_variable_port_v.handle)

        scoped_variable_port_v.handle.pos = self.width * (0.1 * len(self._scoped_variables_ports)), 0

        port_meta = self.model.meta['scoped%d' % scoped_variable_port_v.port_id]['gui']['editor_gaphas']
        if isinstance(port_meta['rel_pos'], tuple):
            scoped_variable_port_v.handle.pos = port_meta['rel_pos']
        else:
            pos_x, pos_y, side = self._calculate_unoccupied_position(scoped_variable_port_v.port_side_size,
                                                                     SnappedSide.TOP, scoped_variable_port_v,
                                                                     scoped=True)
            if pos_x is None:
                self.model.state.remove_scoped_variable(scoped_variable_port_v.port_id)
                return
            scoped_variable_port_v.handle.pos = pos_x, pos_y
            scoped_variable_port_v.side = side

        self.add_rect_constraint_for_port(scoped_variable_port_v)

    def remove_scoped_variable(self, scoped_variable_port_v):
        self._scoped_variables_ports.remove(scoped_variable_port_v)
        self._ports.remove(scoped_variable_port_v.port)
        self._handles.remove(scoped_variable_port_v.handle)

        if scoped_variable_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints[scoped_variable_port_v])

    def add_rect_constraint_for_port(self, port):
        constraint = PortRectConstraint((self.handles()[NW].pos, self.handles()[SE].pos), port.pos, port)
        solver = self.canvas.solver
        solver.add_constraint(constraint)
        self.port_constraints[port] = constraint

    def _calculate_unoccupied_position(self, port_size, side, new_port, logic=False, in_port=False, scoped=False):
        if in_port:
            new_pos_x = 0
        elif scoped:
            new_pos_x = self.width * .15
        else:
            new_pos_x = self.width
        if logic:
            new_pos_y = self.height * .15
        elif scoped:
            new_pos_y = 0
        else:
            new_pos_y = self.height * .85

        position_found = False
        all_ports = self.get_all_ports()
        new_pos = (new_pos_x, new_pos_y)

        port_limitation_counter = 0

        while not position_found and port_limitation_counter < 100:
            position_found, new_pos_x, new_pos_y, side = self._check_pos(all_ports, port_size, side, new_port, new_pos,
                                                                         logic, in_port)
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
                return pos + constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
            else:
                return pos - constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
        elif side is SnappedSide.LEFT:
            if logic or (not logic and in_port):
                return pos - constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
            else:
                return pos + constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
        elif side is SnappedSide.BOTTOM:
            if not in_port and not logic:
                return pos + constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
            else:
                return pos - constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
        elif side is SnappedSide.TOP:
            if not in_port and not logic:
                return pos - constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size
            else:
                return pos + constants.INITIAL_DISTANCE_BETWEEN_PORTS_MULTIPLIER * port_size

    def resize_all_children(self, old_size, paste=False):
        from rafcon.mvc.mygaphas.utils import gap_helper
        new_size = (self.width, self.height)
        canvas = self.canvas

        def resize_child(state, old_size, new_size, paste):
            def calc_abs_pos(item, pos):
                projection = canvas.project(item, pos)
                return projection[0].value, projection[1].value

            def handle_set_rel_pos(item, handle_pos, new_pos, parent_abs_pos, old_size=None):
                projection = canvas.project(item, handle_pos)
                projection[0].value = parent_abs_pos[0] + new_pos[0]
                projection[1].value = parent_abs_pos[1] + new_pos[1]
                if isinstance(item, StateView) and old_size:
                    item.handles()[SE].pos.x = item.handles()[NW].pos.x + old_size[0]
                    item.handles()[SE].pos.y = item.handles()[NW].pos.y + old_size[1]

            state_abs_pos = calc_abs_pos(state, state.handles()[NW].pos)

            width_factor = new_size[0] / old_size[0]
            height_factor = new_size[1] / old_size[1]

            def calc_new_rel_pos(old_rel_pos, old_parent_size, new_parent_size):
                old_rel_pos_x_rel = old_rel_pos[0] / old_parent_size[0]
                old_rel_pos_y_rel = old_rel_pos[1] / old_parent_size[1]
                new_rel_pos_x = new_parent_size[0] * old_rel_pos_x_rel
                new_rel_pos_y = new_parent_size[1] * old_rel_pos_y_rel
                return new_rel_pos_x, new_rel_pos_y

            for port_v in state.get_all_ports():
                new_rel_pos = calc_new_rel_pos(port_v.handle.pos, old_size, new_size)
                port_v.handle.pos = new_rel_pos

            name_v = state.name_view
            old_rel_pos = gap_helper.calc_rel_pos_to_parent(canvas, name_v, name_v.handles()[NW])
            new_rel_pos = calc_new_rel_pos(old_rel_pos, old_size, new_size)
            handle_set_rel_pos(name_v, name_v.handles()[NW].pos, new_rel_pos, state_abs_pos,
                               (name_v.width, name_v.height))

            name_v.width *= width_factor
            name_v.height *= height_factor

            if isinstance(state.model, ContainerStateModel):
                for transition_v in state.get_transitions():
                    for waypoint in transition_v.waypoints:
                        old_rel_pos = gap_helper.calc_rel_pos_to_parent(canvas, transition_v, waypoint)
                        new_rel_pos = calc_new_rel_pos(old_rel_pos, old_size, new_size)
                        handle_set_rel_pos(transition_v, waypoint.pos, new_rel_pos, state_abs_pos)

                for child_state_v in self.child_state_vs:
                    if not paste:
                        old_rel_pos = gap_helper.calc_rel_pos_to_parent(canvas, child_state_v, child_state_v.handles()[NW])
                        new_rel_pos = calc_new_rel_pos(old_rel_pos, old_size, new_size)
                        handle_set_rel_pos(child_state_v, child_state_v.handles()[NW].pos, new_rel_pos, state_abs_pos,
                                           (child_state_v.width, child_state_v.height))

                        old_size = (child_state_v.width, child_state_v.height)
                    else:
                        meta_rel_pos = child_state_v.model.meta['gui']['editor_gaphas']['rel_pos']
                        new_rel_pos = calc_new_rel_pos(meta_rel_pos, old_size, new_size)

                        child_state_v.matrix.translate(*new_rel_pos)

                        old_size = child_state_v.model.meta['gui']['editor_gaphas']['size']

                    new_size = (old_size[0] * width_factor, old_size[1] * height_factor)
                    child_state_v.width = new_size[0]
                    child_state_v.height = new_size[1]

                    resize_child(child_state_v, old_size, new_size, paste)

        resize_child(self, old_size, new_size, paste)


class NameView(Element):

    def __init__(self, name, size):
        super(NameView, self).__init__(size[0], size[1])

        self._name = None
        self.name = name

        self.min_width = 0.0001
        self.min_height = 0.0001

        self.moving = False

        self.__font_size_cache = {'text': None, 'width': None, 'height': None, 'size': None}

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        assert isinstance(name, str)
        self._name = name

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    def draw(self, context):
        if self.moving:
            return

        c = context.cairo

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

        if context.selected:
            c.rectangle(0, 0, self.width, self.height)
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(constants.LABEL_COLOR), alpha=.1))
            c.fill_preserve()
            c.set_source_rgba(0, 0, 0, 0)
            c.stroke()

        pcc = CairoContext(cc)
        pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = pcc.create_layout()
        layout.set_wrap(WRAP_WORD)
        layout.set_width(int(self.width) * SCALE)
        layout.set_text(self.name)

        def set_font_description():
            font = FontDescription(font_name + " " + str(font_size))
            layout.set_font_description(font)

        font_name = constants.FONT_NAMES[0]

        if self.__font_size_cache['width'] == self.width and self.__font_size_cache['height'] == self.height and \
                self.__font_size_cache['text'] == self.name:
            font_size = self.__font_size_cache['size']
            set_font_description()

        else:
            font_size = self.height * 0.8

            set_font_description()
            pango_size = (self.width * SCALE, self.height * SCALE)
            while layout.get_size()[0] > pango_size[0] or layout.get_size()[1] > pango_size[1]:
                font_size *= 0.9
                set_font_description()

            self.__font_size_cache = {'text': self.name, 'width': self.width, 'height': self.height, 'size': font_size}

        c.move_to(*self.handles()[NW].pos)
        cc.set_source_rgba(*get_col_rgba(Color(constants.STATE_NAME_COLOR), self.parent.transparent))
        pcc.update_layout(layout)
        pcc.show_layout(layout)