# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from future.utils import string_types
from builtins import str
from weakref import ref
from gi.repository.Pango import SCALE, FontDescription, WrapMode
from gi.repository import PangoCairo
# from cairo import Antialias
from copy import copy
import cairo

from gaphas.item import Element, NW, NE, SW, SE
from gaphas.connector import Position
from gaphas.matrix import Matrix
from gaphas.solver import Variable
from gaphas.painter import CairoBoundingBoxContext

from rafcon.core.states.state import StateExecutionStatus

from rafcon.gui.mygaphas.canvas import ItemProjection
from rafcon.gui.mygaphas.constraint import KeepRectangleWithinConstraint, PortRectConstraint, BorderWidthConstraint
from rafcon.gui.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedVariablePortView
from rafcon.gui.mygaphas.items.connection import TransitionView
from rafcon.gui.mygaphas.utils.enums import SnappedSide
from rafcon.gui.mygaphas.utils.gap_draw_helper import get_col_rgba
from rafcon.gui.mygaphas.utils import gap_draw_helper
from rafcon.gui.mygaphas.utils.cache.image_cache import ImageCache

from rafcon.gui.models import AbstractStateModel, LibraryStateModel, ContainerStateModel
from rafcon.gui.helpers.meta_data import contains_geometric_info
from rafcon.gui.helpers.label import set_label_markup
from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)

# Fixed width of the Pango layout. The higher this value, the better is the accuracy, but the more memory is consumed
BASE_WIDTH = 100.


class StateView(Element):
    """ A State has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    _map_handles_port_v = {}

    def __init__(self, state_m, size, hierarchy_level):
        super(StateView, self).__init__(size[0], size[1])
        assert isinstance(state_m, AbstractStateModel)
        # Reapply size, as Gaphas sets default minimum size to 1, which is too large for highly nested states
        self.min_width = self.min_height = 0
        self.width = size[0]
        self.height = size[1]

        self._c_min_w = self._constraints[0]
        self._c_min_h = self._constraints[1]

        self.is_root_state_of_library = state_m.state.is_root_state_of_library

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

        self._view = None

        self.__symbol_size_cache = {}
        self._image_cache = ImageCache()

        self._border_width = Variable(min(self.width, self.height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        border_width_constraint = BorderWidthConstraint(self._handles[NW].pos, self._handles[SE].pos,
                                                        self._border_width, constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        self._constraints.append(border_width_constraint)

        # Initialize NameView
        name_meta = state_m.get_meta_data_editor()['name']
        if not contains_geometric_info(name_meta['size']):
            name_width = self.width - 2 * self._border_width
            name_height = self.height * 0.4
            name_meta = state_m.set_meta_data_editor('name.size', (name_width, name_height))['name']
        name_size = name_meta['size']

        self._name_view = NameView(state_m.state.name, name_size)

        if not contains_geometric_info(name_meta['rel_pos']):
            name_meta['rel_pos'] = (self.border_width, self.border_width)
        name_pos = name_meta['rel_pos']
        self.name_view.matrix.translate(*name_pos)

    @property
    def selected(self):
        return self in self.view.selected_items

    @property
    def hovered(self):
        return self is self.view.hovered_item

    @property
    def view(self):
        if not self._view:
            self._view = self.canvas.get_first_view()
        return self._view

    def setup_canvas(self):
        canvas = self.canvas
        parent = self.parent

        self.update_minimum_size()

        # Draw NameView beneath all other state elements
        canvas.add(self.name_view, self, index=0)
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
            min_side_length = min(self.parent.width, self.parent.height) / \
                              constants.MAXIMUM_CHILD_TO_PARENT_STATE_SIZE_RATIO
            if min_side_length != self.min_width:
                self.min_width = min_side_length
            if min_side_length != self.min_height:
                self.min_height = min_side_length

    def update_minimum_size_of_children(self):
        if self.canvas:
            self.canvas.resolve_item_constraints(self)
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
        view = self.canvas.get_first_view()
        if view:
            view.unselect_item(self)

        for child in self.canvas.get_children(self)[:]:
            child.remove()

        self.remove_income()
        for outcome_v in self.outcomes[:]:
            self.remove_outcome(outcome_v)
        for input_port_v in self.inputs[:]:
            self.remove_input_port(input_port_v)
        for output_port_v in self.outputs[:]:
            self.remove_output_port(output_port_v)
        for scoped_variable_port_v in self.scoped_variables[:]:
            self.remove_scoped_variable(scoped_variable_port_v)

        self.remove_keep_rect_within_constraint_from_parent()
        for constraint in self._constraints[:]:
            self.canvas.solver.remove_constraint(constraint)
            self._constraints.remove(constraint)
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

        name_constraint = self.keep_rect_constraints.pop(self.name_view)
        solver.remove_constraint(name_constraint)

        parent_state_v = self.parent
        if parent_state_v is not None and isinstance(parent_state_v, StateView):
            constraint = parent_state_v.keep_rect_constraints.pop(self)
            solver.remove_constraint(constraint)

    def set_enable_flag_keep_rect_within_constraints(self, enable):
        """ Enable/disables the KeepRectangleWithinConstraint for child states """
        for child_state_v in self.child_state_views():
            self.keep_rect_constraints[child_state_v].enable = enable
            child_state_v.keep_rect_constraints[child_state_v._name_view].enable = enable

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
        return self._border_width.value

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
        if self.model:
            self.canvas.exchange_model(self.model, state_m)
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
    def transparency(self):
        """Calculates the transparency for the state

        :return: State transparency
        :rtype: float
        """
        # TODO: Implement transparency logic here (e.g. for different viewing modes)
        return 0.

    def child_state_views(self):
        for child in self.canvas.get_children(self):
            if isinstance(child, StateView):
                yield child

    def show_content(self, with_content=False):
        """Checks if the state is a library with the `show_content` flag set

        :param with_content: If this parameter is `True`, the method return only True if the library represents a
          ContainerState
        :return: Whether the content of a library state is shown
        """
        if isinstance(self.model, LibraryStateModel) and self.model.show_content():
            return not with_content or isinstance(self.model.state_copy, ContainerStateModel)
        return False

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

    def apply_meta_data(self, recursive=False):
        # Do not check KeepRectWithin constraints when applying meta data, as this causes issues in recursive operations
        self.set_enable_flag_keep_rect_within_constraints(False)
        state_meta = self.model.get_meta_data_editor()

        self.position = state_meta['rel_pos']
        self.width = state_meta['size'][0]
        self.height = state_meta['size'][1]
        self.update_minimum_size_of_children()

        def update_port_position(port_v, meta_data):
            if contains_geometric_info(meta_data['rel_pos']):
                port_v.handle.pos = meta_data['rel_pos']
                self.port_constraints[port_v].update_position(meta_data['rel_pos'])

        update_port_position(self.income, self.income.model.get_meta_data_editor())
        for outcome_v in self.outcomes:
            update_port_position(outcome_v, outcome_v.model.get_meta_data_editor())
        for data_port_v in self.inputs + self.outputs:
            update_port_position(data_port_v, data_port_v.model.get_meta_data_editor())

        self.name_view.apply_meta_data()

        if isinstance(self.model, ContainerStateModel):
            for scoped_port_v in self.scoped_variables:
                update_port_position(scoped_port_v, scoped_port_v.model.get_meta_data_editor())
            for transition_m in self.model.transitions:
                transition_v = self.canvas.get_view_for_model(transition_m)
                transition_v.apply_meta_data()

            if recursive:
                for state_v in self.canvas.get_children(self):
                    if isinstance(state_v, StateView):
                        state_v.apply_meta_data(recursive=True)
        self.set_enable_flag_keep_rect_within_constraints(True)

    def draw(self, context):
        # Do not draw if
        # * state (or its parent) is currently moved
        # * core element is no longer existing (must have just been removed)
        # * is root state of a library (drawing would hide the LibraryState itself)
        if not self.model.state or self.moving and self.parent and self.parent.moving or \
                self.model.state.is_root_state_of_library:
            if not context.draw_all:
                return

        width = self.width
        height = self.height
        border_width = self.border_width
        view_width, view_height = self.view.get_matrix_i2v(self).transform_distance(width, height)
        if min(view_width, view_height) < constants.MINIMUM_STATE_SIZE_FOR_DISPLAY and self.parent and not \
                context.draw_all:
            return

        c = context.cairo
        nw = self._handles[NW].pos
        parameters = {
            'execution_state':  self.model.state.state_execution_status,
            'selected': self.selected,
            'moving': self.moving,
            'border_width': border_width,
            'transparency': self.transparency,
            'draw_all': context.draw_all
        }

        upper_left_corner = (nw.x.value, nw.y.value)
        current_zoom = self.view.get_zoom_factor()
        from_cache, image, zoom = self._image_cache.get_cached_image(width, height, current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            self._image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            c = self._image_cache.get_context_for_image(current_zoom)
            multiplicator = self._image_cache.multiplicator
            default_line_width = border_width / constants.BORDER_WIDTH_OUTLINE_WIDTH_FACTOR * multiplicator

            c.rectangle(nw.x, nw.y, width, height)

            state_background_color = gui_config.gtk_colors['STATE_BACKGROUND']
            state_border_color = gui_config.gtk_colors['STATE_BORDER']
            state_border_outline_color = gui_config.gtk_colors['STATE_BORDER_OUTLINE']

            if self.model.state.state_execution_status == StateExecutionStatus.WAIT_FOR_NEXT_STATE:
                state_border_color = gui_config.gtk_colors['STATE_WAITING_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_WAITING_BORDER_OUTLINE']
            elif self.model.state.active:
                state_border_color = gui_config.gtk_colors['STATE_ACTIVE_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_ACTIVE_BORDER_OUTLINE']
            elif self.selected:
                state_border_color = gui_config.gtk_colors['STATE_SELECTED_BORDER']
                state_border_outline_color = gui_config.gtk_colors['STATE_SELECTED_BORDER_OUTLINE']

            c.set_source_rgba(*get_col_rgba(state_border_color, self.transparency))
            c.fill_preserve()
            c.set_source_rgba(*get_col_rgba(state_border_outline_color, self.transparency))
            # The line gets cropped at the context border, therefore the line width must be doubled
            c.set_line_width(default_line_width * 2)
            c.stroke()

            if not context.draw_all:
                inner_nw, inner_se = self.get_state_drawing_area(self)
                c.rectangle(inner_nw.x, inner_nw.y, inner_se.x - inner_nw.x, inner_se.y - inner_nw.y)
                c.set_source_rgba(*get_col_rgba(state_background_color))
                c.fill_preserve()
                c.set_source_rgba(*get_col_rgba(state_border_outline_color, self.transparency))
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
            symbol_transparency = 0.9 if self.show_content(with_content=True) else 0.75
            self._draw_symbol(context, constants.SIGN_LIB, gui_config.gtk_colors['STATE_NAME'], symbol_transparency)

        if self.moving:
            self._draw_symbol(context, constants.SIGN_ARROW, gui_config.gtk_colors['STATE_NAME'])

    def _draw_symbol(self, context, symbol, color, transparency=0.):
        c = context.cairo
        cairo_context = c
        if isinstance(c, CairoBoundingBoxContext):
            cairo_context = c._cairo
        width = self.width
        height = self.height

        # c.set_antialias(Antialias.GOOD)

        layout = PangoCairo.create_layout(cairo_context)

        def set_font_description():
            set_label_markup(layout, symbol, is_icon=True, size=font_size)

        if symbol in self.__symbol_size_cache and \
                self.__symbol_size_cache[symbol]['width'] == width and \
                self.__symbol_size_cache[symbol]['height'] == height:
            font_size = self.__symbol_size_cache[symbol]['size']
            set_font_description()

        else:
            font_size = 30
            set_font_description()

            pango_size = (width * SCALE, height * SCALE)
            while layout.get_size()[0] > pango_size[0] * constants.ICON_STATE_FILL_FACTOR or \
                    layout.get_size()[1] > pango_size[1] * constants.ICON_STATE_FILL_FACTOR:
                font_size *= 0.9
                set_font_description()

            self.__symbol_size_cache[symbol] = {'width': width, 'height': height, 'size': font_size}

        c.move_to(width / 2. - layout.get_size()[0] / float(SCALE) / 2.,
                  height / 2. - layout.get_size()[1] / float(SCALE) / 2.)

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        PangoCairo.update_layout(cairo_context, layout)
        PangoCairo.show_layout(cairo_context, layout)

    def get_transitions(self):
        transitions = []
        for child in self.canvas.get_children(self):
            if isinstance(child, TransitionView):
                transitions.append(child)
        return transitions

    def connect_connection_to_port(self, connection_v, port, as_target=True):
        handle = connection_v.to_handle() if as_target else connection_v.from_handle()
        if isinstance(port, IncomeView):
            self.connect_to_income(connection_v, handle)
        elif isinstance(port, OutcomeView):
            self.connect_to_outcome(port.outcome_id, connection_v, handle)
        elif isinstance(port, InputPortView):
            self.connect_to_input_port(port.port_id, connection_v, handle)
        elif isinstance(port, OutputPortView):
            self.connect_to_output_port(port.port_id, connection_v, handle)
        elif isinstance(port, ScopedVariablePortView):
            self.connect_to_scoped_variable_port(port.port_id, connection_v, handle)

    def connect_to_income(self, connection_v, handle):
        self._income.add_connected_handle(handle, connection_v)
        connection_v.set_port_for_handle(self._income, handle)
        self._connect_to_port(self._income.port, connection_v, handle)

    def connect_to_outcome(self, outcome_id, connection_v, handle):
        outcome_v = self.outcome_port(outcome_id)
        outcome_v.add_connected_handle(handle, connection_v)
        connection_v.set_port_for_handle(outcome_v, handle)
        self._connect_to_port(outcome_v.port, connection_v, handle)

    def connect_to_input_port(self, port_id, connection_v, handle):
        port_v = self.input_port(port_id)
        port_v.add_connected_handle(handle, connection_v)
        connection_v.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, connection_v, handle)

    def connect_to_output_port(self, port_id, connection_v, handle):
        port_v = self.output_port(port_id)
        port_v.add_connected_handle(handle, connection_v)
        connection_v.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, connection_v, handle)

    def connect_to_scoped_variable_port(self, scoped_variable_id, connection_v, handle):
        port_v = self.scoped_variable(scoped_variable_id)
        port_v.add_connected_handle(handle, connection_v)
        connection_v.set_port_for_handle(port_v, handle)
        self._connect_to_port(port_v.port, connection_v, handle)

    def _connect_to_port(self, port, connection_v, handle):
        c = port.constraint(self.canvas, connection_v, handle, self)
        self.canvas.connect_item(connection_v, handle, self, port, c)

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

    def add_income(self, income_m):
        income_v = IncomeView(income_m, self)
        self.canvas.add_port(income_v)
        self._income = income_v
        self._ports.append(income_v.port)
        self._handles.append(income_v.handle)
        self._map_handles_port_v[income_v.handle] = income_v

        port_meta = income_m.get_meta_data_editor()
        if not contains_geometric_info(port_meta['rel_pos']):
            # print("generate rel_pos")
            # Position income on the top of the left state side
            income_v.side = SnappedSide.LEFT
            pos_x = 0
            pos_y = self._calculate_port_pos_on_line(1, self.height)
            port_meta = income_m.set_meta_data_editor('rel_pos', (pos_x, pos_y))
        # print("add income", self.model, self.model.parent, port_meta['rel_pos'])
        income_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(income_v)
        return income_v

    def remove_income(self):
        income_v = self._income
        del self._map_handles_port_v[income_v.handle]
        self._income = None
        self._ports.remove(income_v.port)
        self._handles.remove(income_v.handle)

        self.canvas.remove_port(income_v)
        if income_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints.pop(income_v))

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m, self)
        self.canvas.add_port(outcome_v)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)
        self._map_handles_port_v[outcome_v.handle] = outcome_v

        port_meta = outcome_m.get_meta_data_editor()
        if not contains_geometric_info(port_meta['rel_pos']):
            # print("generate rel_pos")
            if outcome_m.outcome.outcome_id < 0:
                # Position aborted/preempted in upper right corner
                outcome_v.side = SnappedSide.TOP
                pos_x = self.width - self._calculate_port_pos_on_line(abs(outcome_m.outcome.outcome_id), self.width)
                pos_y = 0
            else:
                # Distribute outcomes on the right side of the state, starting from top
                outcome_v.side = SnappedSide.RIGHT
                pos_x = self.width

                number_of_outcome = [o.model for o in self.outcomes if o.model.outcome.outcome_id >= 0].index(outcome_m) + 1
                pos_y = self._calculate_port_pos_on_line(number_of_outcome, self.height)
            port_meta = outcome_m.set_meta_data_editor('rel_pos', (pos_x, pos_y))
        # print("add outcome", self.model, self.model.parent, port_meta['rel_pos'])
        outcome_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(outcome_v)

    def remove_outcome(self, outcome_v):
        del self._map_handles_port_v[outcome_v.handle]
        self._outcomes.remove(outcome_v)
        self._ports.remove(outcome_v.port)
        self._handles.remove(outcome_v.handle)

        self.canvas.remove_port(outcome_v)
        if outcome_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints.pop(outcome_v))

    def add_input_port(self, port_m):
        input_port_v = InputPortView(self, port_m)
        self.canvas.add_port(input_port_v)
        self._inputs.append(input_port_v)
        self._ports.append(input_port_v.port)
        self._handles.append(input_port_v.handle)
        self._map_handles_port_v[input_port_v.handle] = input_port_v

        port_meta = port_m.get_meta_data_editor()
        if not contains_geometric_info(port_meta['rel_pos']):
            # print("generate rel_pos")
            # Distribute input ports on the left side of the state, starting from bottom
            input_port_v.side = SnappedSide.LEFT
            number_of_input = self.model.input_data_ports.index(port_m) + 1
            pos_x = 0
            pos_y = self.height - self._calculate_port_pos_on_line(number_of_input, self.height)
            port_meta = port_m.set_meta_data_editor('rel_pos', (pos_x, pos_y))
        # print("add input_port", self.model, self.model.parent, port_meta['rel_pos'])
        input_port_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(input_port_v)

    def remove_input_port(self, input_port_v):
        del self._map_handles_port_v[input_port_v.handle]
        self._inputs.remove(input_port_v)
        self._ports.remove(input_port_v.port)
        self._handles.remove(input_port_v.handle)

        self.canvas.remove_port(input_port_v)
        if input_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints.pop(input_port_v))

    def add_output_port(self, port_m):
        output_port_v = OutputPortView(self, port_m)
        self.canvas.add_port(output_port_v)
        self._outputs.append(output_port_v)
        self._ports.append(output_port_v.port)
        self._handles.append(output_port_v.handle)
        self._map_handles_port_v[output_port_v.handle] = output_port_v

        port_meta = port_m.get_meta_data_editor()
        if not contains_geometric_info(port_meta['rel_pos']):
            # Distribute output ports on the right side of the state, starting from bottom
            # print("generate rel_pos")
            output_port_v.side = SnappedSide.RIGHT
            number_of_output = self.model.output_data_ports.index(port_m) + 1
            pos_x = self.width
            pos_y = self.height - self._calculate_port_pos_on_line(number_of_output, self.height)
            port_meta = port_m.set_meta_data_editor('rel_pos', (pos_x, pos_y))
        # print("add output_port", self.model, self.model.parent, port_meta['rel_pos'])
        output_port_v.handle.pos = port_meta['rel_pos']
        self.add_rect_constraint_for_port(output_port_v)

    def remove_output_port(self, output_port_v):
        del self._map_handles_port_v[output_port_v.handle]
        self._outputs.remove(output_port_v)
        self._ports.remove(output_port_v.port)
        self._handles.remove(output_port_v.handle)

        self.canvas.remove_port(output_port_v)
        if output_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints.pop(output_port_v))

    def add_scoped_variable(self, scoped_variable_m):
        scoped_variable_port_v = ScopedVariablePortView(self, scoped_variable_m)
        self.canvas.add_port(scoped_variable_port_v)
        self._scoped_variables_ports.append(scoped_variable_port_v)
        self._ports.append(scoped_variable_port_v.port)
        self._handles.append(scoped_variable_port_v.handle)
        self._map_handles_port_v[scoped_variable_port_v.handle] = scoped_variable_port_v

        scoped_variable_port_v.handle.pos = self.width * (0.1 * len(self._scoped_variables_ports)), 0

        port_meta = scoped_variable_m.get_meta_data_editor()
        if not contains_geometric_info(port_meta['rel_pos']):
            # Distribute scoped variables on the top side of the state, starting from left
            # print("generate rel_pos")
            scoped_variable_port_v.side = SnappedSide.BOTTOM

            number_of_scoped_var = self.model.scoped_variables.index(scoped_variable_m) + 1
            pos_x = self._calculate_port_pos_on_line(number_of_scoped_var, self.width,
                                                     port_width=self.border_width * 4)
            pos_y = self.height
            port_meta = scoped_variable_m.set_meta_data_editor('rel_pos', (pos_x, pos_y))
        # print("add scoped_variable", self.model, self.model.parent, port_meta['rel_pos'])
        scoped_variable_port_v.handle.pos = port_meta['rel_pos']

        self.add_rect_constraint_for_port(scoped_variable_port_v)

    def remove_scoped_variable(self, scoped_variable_port_v):
        del self._map_handles_port_v[scoped_variable_port_v.handle]
        self._scoped_variables_ports.remove(scoped_variable_port_v)
        self._ports.remove(scoped_variable_port_v.port)
        self._handles.remove(scoped_variable_port_v.handle)

        self.canvas.remove_port(scoped_variable_port_v)
        if scoped_variable_port_v in self.port_constraints:
            self.canvas.solver.remove_constraint(self.port_constraints.pop(scoped_variable_port_v))

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

        def set_item_properties(item, size, rel_pos):
            prefix = 'name.' if isinstance(item, NameView) else ''
            item_m = item.model if isinstance(item, StateView) else item.parent.model
            item.width = size[0]
            item.height = size[1]
            item_m.set_meta_data_editor(prefix + 'size', size)
            if item is not self:
                item.position = rel_pos
                item_m.set_meta_data_editor(prefix + 'rel_pos', rel_pos)
            if isinstance(item, StateView):
                item.update_minimum_size_of_children()

        def resize_state_v(state_v, old_state_size, new_state_size, use_meta_data):

            state_v.set_enable_flag_keep_rect_within_constraints(enable=False)

            width_factor = float(new_state_size[0]) / old_state_size[0]
            height_factor = float(new_state_size[1]) / old_state_size[1]

            # Set new state view properties
            old_state_rel_pos = state_v.position
            new_state_rel_pos = calc_new_rel_pos(old_state_rel_pos, old_state_size, new_state_size)
            set_item_properties(state_v, new_state_size, new_state_rel_pos)

            # Set new name view properties
            name_v = state_v.name_view
            if use_meta_data:
                old_name_size = state_v.model.get_meta_data_editor()['name']['size']
                old_name_rel_pos = state_v.model.get_meta_data_editor()['name']['rel_pos']
            else:
                old_name_size = (name_v.width, name_v.height)
                old_name_rel_pos = name_v.position
            new_name_size = (old_name_size[0] * width_factor, old_name_size[1] * height_factor)
            new_name_rel_pos = calc_new_rel_pos(old_name_rel_pos, old_state_size, new_state_size)
            set_item_properties(name_v, new_name_size, new_name_rel_pos)

            def resize_child_state_v(child_state_v):
                if use_meta_data:
                    old_child_size = child_state_v.model.get_meta_data_editor()['size']
                else:
                    old_child_size = (child_state_v.width, child_state_v.height)

                new_child_size = (old_child_size[0] * width_factor, old_child_size[1] * height_factor)
                new_child_size = (max(new_child_size[0], child_state_v.min_width),
                                  max(new_child_size[1], child_state_v.min_height))
                resize_state_v(child_state_v, old_child_size, new_child_size, use_meta_data)

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
                    resize_child_state_v(child_state_v)
            elif state_v.show_content():
                state_copy_v = self.canvas.get_view_for_model(state_v.model.state_copy)
                resize_child_state_v(state_copy_v)

            state_v.set_enable_flag_keep_rect_within_constraints(enable=True)

        # Deactivate KeepRectangleWithin constraints for child states
        self.set_enable_flag_keep_rect_within_constraints(enable=False)
        # Now we can solve the KeepRectangleWithin constraints of this state without being called recursively
        # We also force the solving of the minimal size constraints
        if self.parent:
            self.view.canvas.resolve_constraint((self.parent.keep_rect_constraints[self], self._c_min_w, self._c_min_h))
        else:
            self.view.canvas.resolve_constraint((self._c_min_w, self._c_min_h))
        new_size = (self.width, self.height)
        resize_state_v(self, old_size, new_size, paste)



class NameView(Element):

    def __init__(self, name, size):
        super(NameView, self).__init__(size[0], size[1])
        # Reapply size, as Gaphas sets default minimum size to 1, which is too large for highly nested states
        self.min_width = self.min_height = 0
        self.width = size[0]
        self.height = size[1]

        self._name = None
        self.name = name

        self.moving = False

        self._view = None

        self._image_cache = ImageCache(multiplicator=1.5)

    def remove(self):
        self.canvas.remove(self)

    def update_minimum_size(self):
        min_side_length = min(self.parent.width, self.parent.height) / constants.MAXIMUM_NAME_TO_PARENT_STATE_SIZE_RATIO
        if min_side_length != self.min_width:
            self.min_width = min_side_length
        if min_side_length != self.min_height:
            self.min_height = min_side_length

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        assert isinstance(name, string_types)
        self._name = name

    @property
    def parent(self):
        return self.canvas.get_parent(self)

    @property
    def model(self):
        return self.parent.model

    @property
    def position(self):
        _, _, _, _, x0, y0 = self.matrix
        return x0, y0

    @position.setter
    def position(self, pos):
        self.matrix = Matrix(x0=pos[0], y0=pos[1])

    @property
    def view(self):
        if not self._view:
            self._view = self.canvas.get_first_view()
        return self._view

    @property
    def transparency(self):
        if self.parent.show_content(with_content=True):
            return gui_config.get_config_value('SHOW_CONTENT_LIBRARY_NAME_TRANSPARENCY', 0.5)
        return self.parent.transparency

    def apply_meta_data(self):
        name_meta = self.parent.model.get_meta_data_editor()['name']
        # logger.info("name rel_pos {}".format(name_meta['rel_pos']))
        # logger.info("name size {}".format(name_meta['size']))
        self.position = name_meta['rel_pos']
        # print("name pos from meta", name_meta['rel_pos'])
        self.width = name_meta['size'][0]
        self.height = name_meta['size'][1]

    def draw(self, context):
        # Do not draw if
        # * state (or its parent) is currently moved
        # * core element is no longer existing (must have just been removed)
        # * is root state of a library (drawing would hide the LibraryState itself)
        if not self.model.state or self.moving or self.parent.model.state.is_root_state_of_library:
            return

        width = self.width
        height = self.height
        view_width, view_height = self.view.get_matrix_i2v(self).transform_distance(width, height)
        if min(view_width, view_height) < constants.MINIMUM_NAME_SIZE_FOR_DISPLAY and not context.draw_all:
            return
        font_transparency = self.transparency

        c = context.cairo
        parameters = {
            'name': self.name,
            'selected': context.selected,
            'transparency': font_transparency,
            'draw_all': context.draw_all
        }

        upper_left_corner = (0, 0)
        current_zoom = self.view.get_zoom_factor()
        from_cache, image, zoom = self._image_cache.get_cached_image(width, height, current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            self._image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            c = self._image_cache.get_context_for_image(current_zoom)

            if context.selected or context.draw_all:
                # Draw light background color if selected
                c.rectangle(0, 0, width, height)
                c.set_source_rgba(*gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'], transparency=.9))
                c.fill_preserve()
                c.set_source_rgba(0, 0, 0, 0)
                c.stroke()

            if context.draw_all:
                # Copy image surface to current cairo context
                self._image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)
                return


            # c.set_antialias(Antialias.GOOD)

            cairo_context = c
            if isinstance(c, CairoBoundingBoxContext):
                cairo_context = c._cairo

            layout = PangoCairo.create_layout(cairo_context)
            layout.set_wrap(WrapMode.WORD)
            layout.set_width(int(round(BASE_WIDTH * SCALE)))
            layout.set_text(self.name, -1)

            def set_font_description(font_size):
                font = FontDescription(font_name + " " + str(font_size))
                layout.set_font_description(font)

            font_name = constants.INTERFACE_FONT

            zoom_scale = BASE_WIDTH / width
            scaled_height = height * zoom_scale
            font_size_parameters = {"text": self.name, "height": scaled_height}
            font_size = self.view.value_cache.get_value("font_size", font_size_parameters)

            if font_size:
                set_font_description(font_size)
            else:
                available_size = (BASE_WIDTH * SCALE, scaled_height * SCALE)
                word_count = len(self.name.split(" "))
                # Set max font size to available height
                max_font_size = scaled_height * 0.9
                # Calculate minimum size that is still to be drawn
                min_name_height = max_font_size / 10.
                # Calculate line height if all words are wrapped
                line_height = max_font_size / word_count
                # Use minimum if previous values and add safety margin
                min_font_size = min(line_height * 0.5, min_name_height)

                # Iteratively calculate font size by always choosing the average of the maximum and minimum size
                working_font_size = None
                current_font_size = (max_font_size + min_font_size) / 2.
                set_font_description(current_font_size)

                while True:
                    logical_extents = layout.get_size()
                    width_factor = logical_extents[0] / available_size[0]
                    height_factor = logical_extents[1] / available_size[1]
                    max_factor = max(width_factor, height_factor)

                    if max_factor > 1:  # font size too large
                        max_font_size = current_font_size
                    elif max_factor > 0.9:  # font size fits!
                        break
                    else:  # font size too small
                        # Nevertheless store the font size in case we do not find anything better
                        if not working_font_size or current_font_size > working_font_size:
                            working_font_size = current_font_size
                        min_font_size = current_font_size
                    if 0.99 < min_font_size / max_font_size < 1.01:  # Stop criterion: changes too small
                        if working_font_size:
                            current_font_size = working_font_size
                            set_font_description(current_font_size)
                        break
                    current_font_size = (max_font_size + min_font_size) / 2.
                    set_font_description(current_font_size)
                self.view.value_cache.store_value("font_size", current_font_size, font_size_parameters)

            c.move_to(*self.handles()[NW].pos)
            cairo_context.set_source_rgba(*get_col_rgba(gui_config.gtk_colors['STATE_NAME'], font_transparency))
            c.save()
            # The pango layout has a fixed width and needs to be fitted to the context size
            cairo_context.scale(1. / zoom_scale, 1. / zoom_scale)

            PangoCairo.update_layout(cairo_context, layout)
            PangoCairo.show_layout(cairo_context, layout)
            c.restore()

            # Copy image surface to current cairo context
            self._image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)
