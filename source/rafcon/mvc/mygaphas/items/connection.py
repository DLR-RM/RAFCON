from weakref import ref
from pango import FontDescription, SCALE

from gtkmvc import Observer
import cairo
from gtk.gdk import Color, CairoContext

from rafcon.utils import constants

from rafcon.statemachine.scope import ScopedVariable
from rafcon.statemachine.states.hierarchy_state import HierarchyState

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.models.transition import TransitionModel
from rafcon.mvc.models.data_flow import DataFlowModel

from rafcon.mvc.mygaphas.constraint import KeepRelativePositionConstraint, KeepPortDistanceConstraint
from rafcon.mvc.mygaphas.items.line import PerpLine
from rafcon.mvc.mygaphas.items.ports import PortView, ScopedVariablePortView
from rafcon.mvc.mygaphas.utils.enums import SnappedSide
from rafcon.mvc.mygaphas.utils import gap_draw_helper


class ConnectionView(PerpLine):

    def set_port_for_handle(self, port, handle):
        if handle is self.from_handle():
            self.from_port = port
        elif handle is self.to_handle():
            self.to_port = port

    def reset_port_for_handle(self, handle):
        if handle is self.from_handle():
            self.reset_from_port()
        elif handle is self.to_handle():
            self.reset_to_port()

    def remove_connection_from_port(self, port):
        if self._from_port and port is self._from_port:
            self._from_port.remove_connected_handle(self._from_handle)
        elif self._to_port and port is self._to_port:
            self._to_port.remove_connected_handle(self._to_handle)

    def remove_connection_from_ports(self):
        if self._from_port:
            self._from_port.remove_connected_handle(self._from_handle)
            self._from_port.tmp_disconnect()
        if self._to_port:
            self._to_port.remove_connected_handle(self._to_handle)
            self.to_port.tmp_disconnect()


class ConnectionPlaceholderView(ConnectionView):

    def __init__(self, hierarchy_level, transition_placeholder):
        super(ConnectionPlaceholderView, self).__init__(hierarchy_level)
        self.line_width = .5 / hierarchy_level

        self.transition_placeholder = transition_placeholder

        if transition_placeholder:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.TRANSITION_LINE_COLOR))
            self._arrow_color = gap_draw_helper.get_col_rgba(Color(constants.LABEL_COLOR))
        else:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_LINE_COLOR))
            self._arrow_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_PORT_COLOR))


class TransitionView(ConnectionView):

    def __init__(self, transition_m, hierarchy_level):
        super(TransitionView, self).__init__(hierarchy_level)
        self._transition_m = None
        self.model = transition_m
        self.line_width = .5 / hierarchy_level

    @property
    def model(self):
        return self._transition_m()

    @model.setter
    def model(self, transition_model):
        assert isinstance(transition_model, TransitionModel)
        self._transition_m = ref(transition_model)

    def draw(self, context):
        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.TRANSITION_LINE_COLOR_SELECTED),
                                                            self.parent.transparent)
        else:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.TRANSITION_LINE_COLOR),
                                                            self.parent.transparent)
        self._arrow_color = gap_draw_helper.get_col_rgba(Color(constants.LABEL_COLOR), self.parent.transparent)
        super(TransitionView, self).draw(context)


class DataFlowView(ConnectionView):

    def __init__(self, data_flow_m, hierarchy_level):
        super(DataFlowView, self).__init__(hierarchy_level)
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = None
        self.model = data_flow_m
        self.line_width = .5 / hierarchy_level

        self._show = global_gui_config.get_config_value("SHOW_DATA_FLOWS", False)

        self._line_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_LINE_COLOR))
        self._arrow_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_PORT_COLOR))

    @property
    def model(self):
        return self._data_flow_m()

    @model.setter
    def model(self, data_flow_m):
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = ref(data_flow_m)

    @property
    def show_connection(self):
        return global_gui_config.get_config_value("SHOW_DATA_FLOWS", False) or self._show

    def show(self):
        self._show = True

    def hide(self):
        self._show = False

    def draw(self, context):
        if not self.show_connection:
            return
        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_LINE_COLOR_SELECTED))
        else:
            self._line_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_LINE_COLOR))
        super(DataFlowView, self).draw(context)


class ScopedVariableDataFlowView(DataFlowView, Observer):

    def __init__(self, data_flow_m, hierarchy_level, scoped_variable):
        Observer.__init__(self)
        super(ScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level)

        self._name_width = 10.
        self._name_width_updated = False

        self._print_side = SnappedSide.LEFT
        self._label_selection_waypoint = None

        assert isinstance(scoped_variable, ScopedVariable)
        self._scoped_variable = scoped_variable

    @property
    def from_port(self):
        return self._from_port

    @property
    def to_port(self):
        return self._to_port

    @from_port.setter
    def from_port(self, port):
        assert isinstance(port, PortView)
        self._from_port = port
        self.observe_model(port)
        self._head_length = port.port_side_size
        if not self._from_waypoint:
            self._from_waypoint = self.add_perp_waypoint()
            self._from_port_constraint = KeepPortDistanceConstraint(self.from_handle().pos, self._from_waypoint.pos,
                                                                    port, self._head_length, self.is_out_port(port))
            self.canvas.solver.add_constraint(self._from_port_constraint)
        if self.to_port:
            self.line_width = min(self.to_port.port_side_size, port.port_side_size) * .2
        else:
            self.line_width = port.port_side_size * .2

    @to_port.setter
    def to_port(self, port):
        assert isinstance(port, PortView)
        self._to_port = port
        self.observe_model(port)
        self._to_head_length = port.port_side_size
        if not self._to_waypoint:
            self._to_waypoint = self.add_perp_waypoint(begin=False)
            self._to_port_constraint = KeepPortDistanceConstraint(self.to_handle().pos, self._to_waypoint.pos,
                                                                  port, 2 * self._to_head_length, self.is_in_port(port))
            self.canvas.solver.add_constraint(self._to_port_constraint)
        if self.from_port:
            self.line_width = min(self.from_port.port_side_size, port.port_side_size) * .2

    def reset_from_port(self):
        self.relieve_model(self.from_port)
        super(ScopedVariableDataFlowView, self).reset_from_port()

    def reset_to_port(self):
        self.relieve_model(self.to_port)
        super(ScopedVariableDataFlowView, self).reset_to_port()

    @property
    def name(self):
        return self._scoped_variable.name
        
    @property
    def connected(self):
        raise NotImplementedError

    @property
    def desired_name_height(self):
        if self.from_port:
            return self._head_length * 2.5
        else:
            return self._head_length * 1.5

    def draw(self, context):
        raise NotImplementedError

    def _update_label_selection_waypoint(self):
        raise NotImplementedError

    def _draw_label(self, context, from_scoped=True):
        if self.parent and self.parent.moving:
            return

        c = context.cairo
        c.set_line_width(self._head_length * .03)

        if from_scoped:
            handle_pos = self.to_handle().pos
            port_side_size = self._to_head_length
        else:
            handle_pos = self.from_handle().pos
            port_side_size = self._head_length

        c.set_source_color(Color(constants.DATA_PORT_COLOR))
        c.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        scoped_layout = c.create_layout()
        port_layout = None

        has_connected_port = False
        if from_scoped and self.to_port:
            self._print_side = self.to_port.side
            has_connected_port = True
            port_layout = c.create_layout()
            port_layout.set_text(" " + self.to_port.name + " ")
        elif not from_scoped and self.from_port:
            self._print_side = self.from_port.side.opposite()
            has_connected_port = True
            port_layout = c.create_layout()
            port_layout.set_text(" " + self.from_port.name + " ")
        scoped_layout.set_text(" " + self.name + " ")

        font_name = constants.FONT_NAMES[0]
        font_size = 20

        def set_font_description(layout):
            font = FontDescription(font_name + " " + str(font_size))
            layout.set_font_description(font)

        if port_layout:
            set_font_description(scoped_layout)
            while scoped_layout.get_size()[1] / float(SCALE) > self.desired_name_height / 2.:
                font_size *= 0.9
                set_font_description(scoped_layout)
            scoped_name_size = scoped_layout.get_size()[0] / float(SCALE), scoped_layout.get_size()[1] / float(SCALE)

            set_font_description(port_layout)
            while port_layout.get_size()[1] / float(SCALE) > self.desired_name_height / 2.:
                font_size *= 0.9
                set_font_description(port_layout)
            port_name_size = port_layout.get_size()[0] / float(SCALE), port_layout.get_size()[1] / float(SCALE)
            name_size = max(scoped_name_size[0], port_name_size[0]), scoped_name_size[1] + port_name_size[1]
        else:
            set_font_description(scoped_layout)
            while scoped_layout.get_size()[1] / float(SCALE) > self.desired_name_height:
                font_size *= 0.9
                set_font_description(scoped_layout)
            scoped_name_size = scoped_layout.get_size()[0] / float(SCALE), scoped_layout.get_size()[1] / float(SCALE)
            name_size = scoped_name_size

        self._name_width = name_size[0]
        self._update_label_selection_waypoint()

        if not has_connected_port:
            col = gap_draw_helper.get_col_rgba(Color(constants.DATA_PORT_COLOR))
            rot_angle, move_x, move_y = gap_draw_helper.draw_name_label(context, col, name_size,
                                                                        handle_pos, self._print_side, port_side_size)
        else:
            rot_angle, move_x, move_y = gap_draw_helper.draw_connected_scoped_label(context, constants.DATA_PORT_COLOR,
                                                                                    name_size, handle_pos,
                                                                                    self._print_side, port_side_size)

        c.move_to(move_x, move_y)
        if has_connected_port:
            c.set_source_color(Color(constants.SCOPED_VARIABLE_TEXT_COLOR))
        else:
            c.set_source_color(Color(constants.DATA_PORT_COLOR))

        c.update_layout(scoped_layout)
        c.rotate(rot_angle)
        c.show_layout(scoped_layout)
        c.rotate(-rot_angle)

        if port_layout:
            if self._print_side is SnappedSide.RIGHT or self._print_side is SnappedSide.LEFT:
                c.move_to(move_x, move_y + scoped_name_size[1])
            elif self._print_side is SnappedSide.BOTTOM:
                c.move_to(move_x - scoped_name_size[1], move_y)
            elif self._print_side is SnappedSide.TOP:
                c.move_to(move_x + scoped_name_size[1], move_y)
            c.set_source_color(Color(constants.DATA_PORT_COLOR))

            c.update_layout(port_layout)
            c.rotate(rot_angle)
            c.show_layout(port_layout)
            c.rotate(-rot_angle)

        parent_state = self.parent.model.state
        if (global_gui_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False) and port_layout and
                isinstance(parent_state, HierarchyState)):
            scoped_data_id = str(self._scoped_variable.data_port_id) + parent_state.state_id
            if scoped_data_id in parent_state.scoped_data.iterkeys():
                value = parent_state.scoped_data[scoped_data_id].value

                value_layout = c.create_layout()
                value_layout.set_text(gap_draw_helper.limit_value_string_length(value))
                set_font_description(value_layout)
                font = FontDescription(font_name + " " + str(name_size[1] * .9))
                value_layout.set_font_description(font)

                value_text_size = value_layout.get_size()[0] / float(SCALE), name_size[1]

                fill_color = gap_draw_helper.get_col_rgba(Color(constants.DATA_VALUE_BACKGROUND_COLOR))
                rot_angle, move_x, move_y = gap_draw_helper.draw_data_value_rect(context, fill_color, value_text_size,
                                                                                 name_size, (move_x, move_y),
                                                                                 self._print_side)

                c.move_to(move_x, move_y)

                c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(constants.SCOPED_VARIABLE_TEXT_COLOR)))

                c.update_layout(value_layout)
                c.rotate(rot_angle)
                c.show_layout(value_layout)
                c.rotate(-rot_angle)


class FromScopedVariableDataFlowView(ScopedVariableDataFlowView):

    def __init__(self, data_flow_m, hierarchy_level, scoped_variable):
        super(FromScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level, scoped_variable)

    @property
    def connected(self):
        return self._from_port is not None

    @property
    def from_port(self):
        return self._from_port

    @property
    def desired_name_height(self):
        if self.to_port:
            return self._to_head_length * 2.5
        else:
            return self._to_head_length * 1.5

    @from_port.setter
    def from_port(self, port):
        if isinstance(port, ScopedVariablePortView):
            self._from_port = port
            self._head_length = port.port_side_size
            if not self._from_waypoint:
                self._from_waypoint = self.add_perp_waypoint()
                self._from_port_constraint = KeepPortDistanceConstraint(self.from_handle().pos, self._from_waypoint.pos,
                                                                        port, self._head_length, self.is_out_port(port))
                self.canvas.solver.add_constraint(self._from_port_constraint)
            if self.to_port:
                self.line_width = min(self.to_port.port_side_size, port.port_side_size) * .2
            else:
                self.line_width = port.port_side_size * .2

            if len(self.handles()) == 4:
                self._update_label_selection_waypoint(True)
                # self.add_waypoint((self.to_handle().x + 2 * self._head_length + self._name_width, self.to_handle().y))

    @Observer.observe('side', assign=True)
    def _to_port_changed_side(self, model, prop_name, info):
        self._update_label_selection_waypoint(True)

    def _update_label_selection_waypoint(self, side_changed=False):
        if not self._name_width_updated or side_changed:
            if not side_changed:
                self._name_width_updated = True
            if len(self._handles) == 5:
                self._handles.remove(self._handles[2])
                self._update_ports()
            pos_x = 0.
            pos_y = 0.
            if self.to_port.side is SnappedSide.LEFT:
                pos_x = self.to_handle().x - 2 * self._to_head_length - self._name_width
                pos_y = self.to_handle().y
            elif self.to_port.side is SnappedSide.RIGHT:
                pos_x = self.to_handle().x + 2 * self._to_head_length + self._name_width
                pos_y = self.to_handle().y
            elif self.to_port.side is SnappedSide.TOP:
                pos_x = self.to_handle().x
                pos_y = self.to_handle().y - 2 * self._to_head_length - self._name_width
            elif self.to_port.side is SnappedSide.BOTTOM:
                pos_x = self.to_handle().x
                pos_y = self.to_handle().y + 2 * self._to_head_length + self._name_width
            self.add_waypoint((pos_x, pos_y))

    def add_waypoint(self, pos):
        handle = self._create_handle(pos)
        self._handles.insert(2, handle)
        self._keep_distance_to_port(handle)
        self._update_ports()
        self._label_selection_waypoint = handle

    def _keep_distance_to_port(self, handle):
        canvas = self.canvas
        solver = canvas.solver
        constraint = KeepRelativePositionConstraint(self.to_handle().pos, handle.pos)
        solver.add_constraint(constraint)

    def reset_from_port(self):
        super(FromScopedVariableDataFlowView, self).reset_from_port()
        if len(self._handles) == 4:
            self._handles.remove(self._label_selection_waypoint)
            self._label_selection_waypoint = None
            self._update_ports()

    def draw(self, context):
        if not self.show_connection:
            return
        if not self.connected:
            super(ScopedVariableDataFlowView, self).draw(context)
        else:
            self._draw_label(context)


class ToScopedVariableDataFlowView(ScopedVariableDataFlowView):

    def __init__(self, data_flow_m, hierarchy_level, scoped_variable):
        super(ToScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level, scoped_variable)

    @property
    def connected(self):
        return self._to_port is not None

    @property
    def to_port(self):
        return self._to_port

    @to_port.setter
    def to_port(self, port):
        if isinstance(port, ScopedVariablePortView):
            self._to_port = port
            self._to_head_length = port.port_side_size
            if not self._to_waypoint:
                self._to_waypoint = self.add_perp_waypoint(begin=False)
                self._to_port_constraint = KeepPortDistanceConstraint(self.to_handle().pos, self._to_waypoint.pos,
                                                                      port, 2 * self._to_head_length, self.is_in_port(port))
                self.canvas.solver.add_constraint(self._to_port_constraint)
            if self.from_port:
                self.line_width = min(self.from_port.port_side_size, port.port_side_size) * .2

            if len(self.handles()) == 4:
                self._update_label_selection_waypoint(True)
                # self.add_waypoint((self.from_handle().x + 2 * self._head_length + self._name_width, self.from_handle().y))

    @Observer.observe('side', assign=True)
    def _from_port_changed_side(self, model, prop_name, info):
        self._update_label_selection_waypoint(True)

    def _update_label_selection_waypoint(self, side_changed=False):
        if not self._name_width_updated or side_changed:
            if not side_changed:
                self._name_width_updated = True
            if len(self._handles) == 5:
                self._handles.remove(self._handles[2])
                self._update_ports()
            pos_x = 0.
            pos_y = 0.
            if self.from_port.side is SnappedSide.LEFT:
                pos_x = self.from_handle().x + 2 * self._head_length + self._name_width
                pos_y = self.from_handle().y
            elif self.from_port.side is SnappedSide.RIGHT:
                pos_x = self.from_handle().x - 2 * self._head_length - self._name_width
                pos_y = self.from_handle().y
            elif self.from_port.side is SnappedSide.TOP:
                pos_x = self.from_handle().x
                pos_y = self.from_handle().y + 2 * self._head_length + self._name_width
            elif self.from_port.side is SnappedSide.BOTTOM:
                pos_x = self.from_handle().x
                pos_y = self.from_handle().y - 2 * self._head_length - self._name_width
            self.add_waypoint((pos_x, pos_y))

    def add_waypoint(self, pos):
        handle = self._create_handle(pos)
        self._handles.insert(2, handle)
        self._keep_distance_to_port(handle)
        self._update_ports()
        self._label_selection_waypoint = handle

    def _keep_distance_to_port(self, handle):
        canvas = self.canvas
        solver = canvas.solver
        constraint = KeepRelativePositionConstraint(self.from_handle().pos, handle.pos)
        solver.add_constraint(constraint)

    def reset_to_port(self):
        super(ToScopedVariableDataFlowView, self).reset_to_port()
        if len(self._handles) == 4:
            self._handles.remove(self._label_selection_waypoint)
            self._label_selection_waypoint = None
            self._update_ports()

    def draw(self, context):
        if not self.show_connection:
            return
        if not self.connected:
            super(ScopedVariableDataFlowView, self).draw(context)
        else:
            self._draw_label(context, False)