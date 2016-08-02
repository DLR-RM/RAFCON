from weakref import ref
from pango import SCALE, FontDescription, WRAP_WORD
from copy import copy
import cairo

from gaphas.item import Element, NW, NE, SW, SE
from gaphas.connector import Position
from gaphas.matrix import Matrix

from rafcon.statemachine.enums import StateExecutionState

from rafcon.mvc.mygaphas.canvas import ItemProjection
from rafcon.mvc.mygaphas.constraint import KeepRectangleWithinConstraint, PortRectConstraint
from rafcon.mvc.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedVariablePortView
from rafcon.mvc.mygaphas.items.connection import TransitionView
from rafcon.mvc.mygaphas.utils.enums import SnappedSide
from rafcon.mvc.mygaphas.utils.gap_draw_helper import get_col_rgba
from rafcon.mvc.mygaphas.utils import gap_draw_helper
from rafcon.mvc.mygaphas.utils.cache.image_cache import ImageCache

from rafcon.mvc.models import AbstractStateModel, LibraryStateModel, ContainerStateModel
from rafcon.mvc.config import global_gui_config as gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.mvc.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateView(Element):
    """ A State has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    _map_handles_port_v = {}

    def __init__(self, state_m, size, hierarchy_level):
        super(StateView, self).__init__(size[0], size[1])
        assert isinstance(state_m, AbstractStateModel)

        self._state_m = ref(state_m)
        self.hierarchy_level = hierarchy_level

        self._income = None
        self._outcomes = []
        self._inputs = []
        self._outputs = []
        self._scoped_variables = []
        self._scoped_variables_ports = []

        self.keep_rect_constraints = {}
        self.port_constraints = {}

        self._moving = False
        self._transparent = False

        self.__symbol_size_cache = {}
        self._image_cache = ImageCache()

        name_meta = state_m.meta['gui']['editor_gaphas']['name']
        if not isinstance(name_meta['size'], tuple):
            name_width = self.width * 0.8
            name_height = self.height * 0.4
            name_meta['size'] = (name_width, name_height)
        name_size = name_meta['size']

        self._name_view = NameView(state_m.state.name, name_size)

        if not isinstance(name_meta['rel_pos'], tuple):
            name_meta['rel_pos'] = (0, 0)
        name_pos = name_meta['rel_pos']
        self.name_view.matrix.translate(*name_pos)

    @property
    def selected(self):
        return self in self.canvas.get_first_view().selected_items

    @property
    def hovered(self):
        return self is self.canvas.get_first_view().hovered_item

    def setup_canvas(self):
        self._income = self.add_income()

        canvas = self.canvas
        parent = canvas.get_parent(self)

        self.update_minimum_size()

        canvas.add(self.name_view, self)
        self.name_view.update_minimum_size()

        self.add_keep_rect_within_constraint(canvas, self, self._name_view)

        if parent is not None:
            assert isinstance(parent, StateView)
            self.add_keep_rect_within_constraint(canvas, parent, self)

        # Registers local constraints
        super(StateView, self).setup_canvas()

    def update_minimum_size(self):
        if not self.parent:
            self.min_width = 1
            self.min_height = 1
        else:
            min_side_length = max(self.parent.width, self.parent.height) / \
                              constants.MAXIMUM_CHILD_TO_PARENT_STATE_SIZE_RATIO
            if min_side_length != self.min_width:
                self.min_width = min_side_length
            if min_side_length != self.min_height:
                self.min_height = min_side_length

    def update_minimum_size_of_children(self):
        if self.canvas:
            for constraint in self.constraints:
                self.canvas.solver.request_resolve_constraint(constraint)
            for item in self.canvas.get_all_children(self):
                if isinstance(item, (StateView, NameView)):
                    item.update_minimum_size()

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

    def remove(self):
        """Remove recursively all children and then the StateView itself
        """
        children = self.canvas.get_children(self)[:]
        for child in children:
            if isinstance(child, StateView):
                child.remove()
            if isinstance(child, NameView):
                self.canvas.remove(child)
        self.remove_keep_rect_within_constraint_from_parent()
        self.canvas.remove(self)

    @staticmethod
    def add_keep_rect_within_constraint(canvas, parent, child):
        solver = canvas.solver

        child_nw = ItemProjection(child.handles()[NW].pos, child, parent)
        child_se = ItemProjection(child.handles()[SE].pos, child, parent)
        constraint = KeepRectangleWithinConstraint(parent.handles()[NW].pos, parent.handles()[SE].pos,
                                                   child_nw, child_se, child, lambda: parent.border_width)
        solver.add_constraint(constraint)
        parent.keep_rect_constraints[child] = constraint

    def remove_keep_rect_within_constraint_from_parent(self):
        canvas = self.canvas
        solver = canvas.solver

        name_constraint = self.keep_rect_constraints[self.name_view]
        solver.remove_constraint(name_constraint)

        parent_state_v = canvas.get_parent(self)
        if parent_state_v is not None and isinstance(parent_state_v, StateView):
            constraint = parent_state_v.keep_rect_constraints[self]
            solver.remove_constraint(constraint)

    def has_selected_child(self):
        for child in self.canvas.get_children(self):
            if isinstance(child, StateView) and child.selected:
                return True
        return False

    @property
    def position(self):
        _, _, _, _, x0, y0 = self.matrix
        return x0, y0

    @position.setter
    def position(self, pos):
        self.matrix = Matrix(x0=pos[0], y0=pos[1])

    @property
    def show_data_port_label(self):
        return global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True)

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
    def border_width(self):
        h = self._handles
        nw_pos = h[NW].pos
        se_pos = h[SE].pos
        width = float(se_pos.x) - float(nw_pos.x)
        height = float(se_pos.y) - float(nw_pos.y)
        return min(width, height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR

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

    @model.setter
    def model(self, state_m):
        self._state_m = ref(state_m)

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

    def child_state_views(self):
        for child in self.canvas.get_children(self):
            if isinstance(child, StateView):
                yield child

    @staticmethod
    def get_state_drawing_area(state):
        assert isinstance(state, StateView)
        border_width = state.border_width

        state_nw_pos_x = state.handles()[NW].pos.x + border_width
        state_nw_pos_y = state.handles()[NW].pos.y + border_width
        state_nw_pos = Position((state_nw_pos_x, state_nw_pos_y))
        state_se_pos_x = state.handles()[SE].pos.x - border_width
        state_se_pos_y = state.handles()[SE].pos.y - border_width
        state_se_pos = Position((state_se_pos_x, state_se_pos_y))

        return state_nw_pos, state_se_pos

    def foreground(self):
        self._transparent = False

    def background(self):
        self._transparent = True

    def apply_meta_data(self, recursive=False):
        state_meta = self.model.meta['gui']['editor_gaphas']

        self.position = state_meta['rel_pos']
        self.width = state_meta['size'][0]
        self.height = state_meta['size'][1]
        self.update_minimum_size_of_children()

        def update_port_position(port_v, meta_data):
            if isinstance(meta_data['rel_pos'], tuple):
                port_v.handle.pos = meta_data['rel_pos']
                self.port_constraints[port_v].update_position(meta_data['rel_pos'])

        if isinstance(state_meta['income']['rel_pos'], tuple):
            update_port_position(self.income, state_meta['income'])
        for outcome_v in self.outcomes:
            update_port_position(outcome_v, outcome_v.model.meta['gui']['editor_gaphas'])
        for data_port_v in self.inputs + self.outputs:
            update_port_position(data_port_v, data_port_v.model.meta['gui']['editor_gaphas'])

        self.name_view.apply_meta_data()

        if isinstance(self.model, ContainerStateModel):
            for scoped_port_v in self.scoped_variables:
                update_port_position(scoped_port_v, scoped_port_v.model.meta['gui']['editor_gaphas'])
            for transition_m in self.model.transitions:
                transition_v = self.canvas.get_view_for_model(transition_m)
                transition_v.apply_meta_data()

            if recursive:
                for state_v in self.canvas.get_children(self):
                    if isinstance(state_v, StateView):
                        state_v.apply_meta_data(recursive=True)

    def draw(self, context):
        if self.moving and self.parent and self.parent.moving:
            return
        c = context.cairo

        nw = self._handles[NW].pos

        parameters = {
            'execution_state':  self.model.state.state_execution_status,
            'selected': self.selected,
            'moving': self.moving,
            'border_width': self.border_width,
            'transparent': self._transparent
        }

        upper_left_corner = (nw.x.value, nw.y.value)
        current_zoom = self.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._image_cache.get_cached_image(self.width, self.height, current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "draw state from cache"
            self._image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw state"
            c = self._image_cache.get_context_for_image(current_zoom)
            multiplicator = self._image_cache.multiplicator
            default_line_width = self.border_width / constants.BORDER_WIDTH_OUTLINE_WIDTH_FACTOR * multiplicator

            c.rectangle(nw.x, nw.y, self.width, self.height)

            state_background_color = gui_config.gtk_colors['STATE_BACKGROUND']
            state_border_color = gui_config.gtk_colors['STATE_BORDER']
            state_border_outline_color = gui_config.gtk_colors['STATE_BORDER_OUTLINE']

            if self.model.state.state_execution_status == StateExecutionState.WAIT_FOR_NEXT_STATE:
                state_border_color = gui_config.gtk_colors['STATE_WAITING_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_WAITING_BORDER_OUTLINE']
            elif self.model.state.active:
                state_border_color = gui_config.gtk_colors['STATE_ACTIVE_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_ACTIVE_BORDER_OUTLINE']
            elif self.selected:
                state_border_color = gui_config.gtk_colors['STATE_SELECTED_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_SELECTED_BORDER_OUTLINE']

            c.set_source_rgba(*get_col_rgba(state_border_color, self._transparent))
            c.fill_preserve()
            c.set_source_rgba(*get_col_rgba(state_border_outline_color, self._transparent))
            # The line gets cropped at the context border, therefore the line width must be doubled
            c.set_line_width(default_line_width * 2)
            c.stroke()

            inner_nw, inner_se = self.get_state_drawing_area(self)
            c.rectangle(inner_nw.x, inner_nw.y, inner_se.x - inner_nw.x, inner_se.y - inner_nw.y)
            c.set_source_rgba(*get_col_rgba(state_background_color))
            c.fill_preserve()
            c.set_source_rgba(*get_col_rgba(state_border_outline_color, self._transparent))
            c.set_line_width(default_line_width)
            c.stroke()

            # Copy image surface to current cairo context
            self._image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

        self._income.draw(context, self)

        for outcome_v in self._outcomes:
            highlight = self.model.state.active and outcome_v.model.outcome is self.model.state.final_outcome
            outcome_v.draw(context, self, highlight)

        for input_v in self._inputs:
            input_v.draw(context, self)

        for output_v in self._outputs:
            output_v.draw(context, self)

        for scoped_variable_v in self._scoped_variables_ports:
            scoped_variable_v.draw(context, self)

        if isinstance(self.model, LibraryStateModel) and not self.moving:
            max_width = self.width / 2.
            max_height = self.height / 2.
            self._draw_symbol(context, constants.SIGN_LIB, True, (max_width, max_height))

        if self.moving:
            max_width = self.width - 2 * self.border_width
            max_height = self.height - 2 * self.border_width
            self._draw_symbol(context, constants.SIGN_ARROW, False, (max_width, max_height))

    def _draw_symbol(self, context, symbol, is_library_state, max_size):
        c = context.cairo
        width = self.width
        height = self.height

        c.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        layout = c.create_layout()

        font_name = constants.ICON_FONT

        def set_font_description():
            layout.set_markup('<span font_desc="%s %s">&#x%s;</span>' %
                              (font_name,
                               font_size,
                               symbol))

        if symbol in self.__symbol_size_cache and \
                self.__symbol_size_cache[symbol]['width'] == width and \
                self.__symbol_size_cache[symbol]['height'] == height:
            font_size = self.__symbol_size_cache[symbol]['size']
            set_font_description()

        else:
            font_size = 30
            set_font_description()

            pango_size = (width * SCALE, height * SCALE)
            while layout.get_size()[0] > pango_size[0] or layout.get_size()[1] > pango_size[1]:
                font_size *= 0.9
                set_font_description()

            self.__symbol_size_cache[symbol] = {'width': width, 'height': height, 'size': font_size}

        c.move_to(width / 2. - layout.get_size()[0] / float(SCALE) / 2.,
                  height / 2. - layout.get_size()[1] / float(SCALE) / 2.)

        alpha = 1.
        if is_library_state and self.transparent:
            alpha = 0.0625
        elif is_library_state:
            alpha = 0.25

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(gui_config.gtk_colors['STATE_NAME'], is_library_state,
                                                         alpha=alpha))
        c.update_layout(layout)
        c.show_layout(layout)

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

    def get_port_for_handle(self, handle):
        if handle in self._map_handles_port_v:
            return self._map_handles_port_v[handle]
        return None

    def _data_port(self, port_list, port_id):
        for port in port_list:
            if port.port_id == port_id:
                return port
        raise AttributeError("Port with id '{0}' not found in state".format(port_id, self.model.state.name))

    def add_income(self):
        income_v = IncomeView(self)
        self._ports.append(income_v.port)
        self._handles.append(income_v.handle)
        self._map_handles_port_v[income_v.handle] = income_v

        port_meta = self.model.meta['gui']['editor_gaphas']['income']
        if not isinstance(port_meta['rel_pos'], tuple):
            # Position income on the top of the left state side
            income_v.side = SnappedSide.LEFT
            pos_x = 0
            pos_y = self._calculate_port_pos_on_line(1, self.height)
            port_meta['rel_pos'] = pos_x, pos_y
        income_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(income_v)
        return income_v

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m, self)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)
        self._map_handles_port_v[outcome_v.handle] = outcome_v

        port_meta = outcome_m.meta['gui']['editor_gaphas']
        if not isinstance(port_meta['rel_pos'], tuple):
            if outcome_m.outcome.outcome_id < 0:
                # Position aborted/preempted in upper right corner
                outcome_v.side = SnappedSide.TOP
                pos_x = self.width - self._calculate_port_pos_on_line(abs(outcome_m.outcome.outcome_id), self.width)
                pos_y = 0
            else:
                # Distribute outcomes on the right side of the state, starting from top
                outcome_v.side = SnappedSide.RIGHT
                pos_x = self.width
                num_outcomes = len([o for o in self.outcomes if o.model.outcome.outcome_id >= 0])
                pos_y = self._calculate_port_pos_on_line(num_outcomes, self.height)
            port_meta['rel_pos'] = pos_x, pos_y
        outcome_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(outcome_v)

    def remove_outcome(self, outcome_v):
        del self._map_handles_port_v[outcome_v.handle]
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
        self._map_handles_port_v[input_port_v.handle] = input_port_v

        port_meta = port_m.meta['gui']['editor_gaphas']
        if not isinstance(port_meta['rel_pos'], tuple):
            # Distribute input ports on the left side of the state, starting from bottom
            input_port_v.side = SnappedSide.LEFT
            num_inputs = len(self._inputs)
            pos_x = 0
            pos_y = self.height - self._calculate_port_pos_on_line(num_inputs, self.height)
            port_meta['rel_pos'] = pos_x, pos_y
        input_port_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(input_port_v)

    def remove_input_port(self, input_port_v):
        del self._map_handles_port_v[input_port_v.handle]
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
        self._map_handles_port_v[output_port_v.handle] = output_port_v

        port_meta = port_m.meta['gui']['editor_gaphas']
        if not isinstance(port_meta['rel_pos'], tuple):
            # Distribute output ports on the right side of the state, starting from bottom
            output_port_v.side = SnappedSide.RIGHT
            num_outputs = len(self._outputs)
            pos_x = self.width
            pos_y = self.height - self._calculate_port_pos_on_line(num_outputs, self.height)
            port_meta['rel_pos'] = pos_x, pos_y
        output_port_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(output_port_v)

    def remove_output_port(self, output_port_v):
        del self._map_handles_port_v[output_port_v.handle]
        self._outputs.remove(output_port_v)
        self._ports.remove(output_port_v.port)
        self._handles.remove(output_port_v.handle)

        if output_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints[output_port_v])

    def add_scoped_variable(self, scoped_variable_m):
        scoped_variable_port_v = ScopedVariablePortView(self, scoped_variable_m)
        self._scoped_variables_ports.append(scoped_variable_port_v)
        self._ports.append(scoped_variable_port_v.port)
        self._handles.append(scoped_variable_port_v.handle)
        self._map_handles_port_v[scoped_variable_port_v.handle] = scoped_variable_port_v

        scoped_variable_port_v.handle.pos = self.width * (0.1 * len(self._scoped_variables_ports)), 0

        port_meta = scoped_variable_m.meta['gui']['editor_gaphas']
        if not isinstance(port_meta['rel_pos'], tuple):
            # Distribute scoped variables on the top side of the state, starting from left
            scoped_variable_port_v.side = SnappedSide.TOP
            num_scoped_vars = len(self._scoped_variables_ports)
            pos_x = self._calculate_port_pos_on_line(num_scoped_vars, self.width,
                                                     port_width=self.border_width * 4)
            pos_y = 0
            port_meta['rel_pos'] = pos_x, pos_y
        scoped_variable_port_v.handle.pos = port_meta['rel_pos']

        self.add_rect_constraint_for_port(scoped_variable_port_v)

    def remove_scoped_variable(self, scoped_variable_port_v):
        del self._map_handles_port_v[scoped_variable_port_v.handle]
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

    def _calculate_port_pos_on_line(self, port_num, side_length, port_width=None):
        """Calculate the position of a port on a line

        The position depends on the number of element. Elements are equally spaced. If the end of the line is
        reached, ports are stacked.
        :param int port_num: The number of the port of that type
        :param float side_length: The length of the side the element is placed on
        :param float port_width: The width of one port
        :return: The position on the line for the given port
        :rtype: float
        """
        if port_width is None:
            port_width = 2 * self.border_width
        border_size = self.border_width
        pos = 0.5 * border_size + port_num * port_width
        outermost_pos = max(side_length / 2., side_length - 0.5 * border_size - port_width)
        pos = min(pos, outermost_pos)
        return pos

    def resize_all_children(self, old_size, paste=False):
        def calc_new_rel_pos(old_rel_pos, old_parent_size, new_parent_size):
            new_rel_pos_x = old_rel_pos[0] * new_parent_size[0] / old_parent_size[0]
            new_rel_pos_y = old_rel_pos[1] * new_parent_size[1] / old_parent_size[1]
            return new_rel_pos_x, new_rel_pos_y

        def set_item_properties(item, item_meta, size, rel_pos):
            item.width = size[0]
            item.height = size[1]
            item_meta['size'] = size
            if item is not self:
                item.position = rel_pos
                item_meta['rel_pos'] = rel_pos
            if isinstance(item, StateView):
                item.update_minimum_size_of_children()

        def resize_state_v(state_v, old_state_size, new_state_size, use_meta_data):
            width_factor = float(new_state_size[0]) / old_state_size[0]
            height_factor = float(new_state_size[1]) / old_state_size[1]

            # Set new state view properties
            old_state_rel_pos = state_v.position
            new_state_rel_pos = calc_new_rel_pos(old_state_rel_pos, old_state_size, new_state_size)
            set_item_properties(state_v, state_v.model.meta['gui']['editor_gaphas'], new_state_size, new_state_rel_pos)

            # Set new name view properties
            name_v = state_v.name_view
            if use_meta_data:
                old_name_size = state_v.model.meta['gui']['editor_gaphas']['name']['size']
            else:
                old_name_size = (name_v.width, name_v.height)
            new_name_size = (old_name_size[0] * width_factor, old_name_size[1] * height_factor)
            old_name_rel_pos = state_v.model.meta['gui']['editor_gaphas']['name']['rel_pos']
            new_name_rel_pos = calc_new_rel_pos(old_name_rel_pos, old_state_size, new_state_size)
            set_item_properties(name_v, state_v.model.meta['gui']['editor_gaphas'], new_name_size, new_name_rel_pos)

            # Set new port view properties
            for port_v in state_v.get_all_ports():
                new_port_rel_pos = calc_new_rel_pos(port_v.handle.pos, old_state_size, new_state_size)
                port_v.handle.pos = new_port_rel_pos

            if isinstance(state_v.model, ContainerStateModel):
                for transition_v in state_v.get_transitions():
                    for waypoint in transition_v.waypoints:
                        old_rel_pos = self.canvas.get_matrix_i2i(transition_v, transition_v.parent).transform_point(
                            *waypoint.pos)
                        new_rel_pos = calc_new_rel_pos(old_rel_pos, old_state_size, new_state_size)
                        waypoint.pos = self.canvas.get_matrix_i2i(transition_v.parent, transition_v).transform_point(
                            *new_rel_pos)

                for child_state_v in state_v.child_state_views():
                    if use_meta_data:
                        old_child_size = child_state_v.model.meta['gui']['editor_gaphas']['size']
                    else:
                        old_child_size = (child_state_v.width, child_state_v.height)

                    new_child_size = (old_child_size[0] * width_factor, old_child_size[1] * height_factor)
                    resize_state_v(child_state_v, old_child_size, new_child_size, use_meta_data)

        new_size = (self.width, self.height)
        resize_state_v(self, old_size, new_size, paste)


class NameView(Element):

    def __init__(self, name, size):
        super(NameView, self).__init__(size[0], size[1])

        self._name = None
        self.name = name

        self.min_width = 1
        self.min_height = 1

        self.moving = False

        self._image_cache = ImageCache(multiplicator=1.5)

    def update_minimum_size(self):
        min_side_length = max(self.parent.width, self.parent.height) / constants.MAXIMUM_NAME_TO_PARENT_STATE_SIZE_RATIO
        if min_side_length != self.min_width:
            self.min_width = min_side_length
        if min_side_length != self.min_height:
            self.min_height = min_side_length

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        assert isinstance(name, basestring)
        self._name = name

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    @property
    def position(self):
        _, _, _, _, x0, y0 = self.matrix
        return x0, y0

    @position.setter
    def position(self, pos):
        self.matrix = Matrix(x0=pos[0], y0=pos[1])

    def apply_meta_data(self):
        name_meta = self.parent.model.meta['gui']['editor_gaphas']['name']
        # logger.info("name rel_pos {}".format(name_meta['rel_pos']))
        # logger.info("name size {}".format(name_meta['size']))
        self.position = name_meta['rel_pos']
        self.width = name_meta['size'][0]
        self.height = name_meta['size'][1]

    def draw(self, context):
        if self.moving:
            return

        c = context.cairo

        parameters = {
            'name': self.name,
            'selected': context.selected
        }

        upper_left_corner = (0, 0)
        current_zoom = self.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._image_cache.get_cached_image(self.width, self.height,
                                                                          current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "from cache"
            self._image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw"
            c = self._image_cache.get_context_for_image(current_zoom)

            if context.selected:
                c.rectangle(0, 0, self.width, self.height)
                c.set_source_rgba(*gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'], alpha=.1))
                c.fill_preserve()
                c.set_source_rgba(0, 0, 0, 0)
                c.stroke()

            c.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

            layout = c.create_layout()
            layout.set_wrap(WRAP_WORD)
            layout.set_width(int(self.width) * SCALE)
            layout.set_text(self.name)

            def set_font_description():
                font = FontDescription(font_name + " " + str(font_size))
                layout.set_font_description(font)

            font_name = constants.INTERFACE_FONT

            font_size = self.height * 0.8

            set_font_description()
            pango_size = (self.width * SCALE, self.height * SCALE)
            while layout.get_size()[0] > pango_size[0] or layout.get_size()[1] > pango_size[1]:
                font_size *= 0.9
                set_font_description()

            c.move_to(*self.handles()[NW].pos)
            c.set_source_rgba(*get_col_rgba(gui_config.gtk_colors['STATE_NAME'], self.parent.transparent))
            c.update_layout(layout)
            c.show_layout(layout)

            # Copy image surface to current cairo context
            self._image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)