from weakref import ref

from gaphas.segment import Segment
from gaphas.item import Line, NW, SE

from awesome_tool.mvc.views.gap.constraint import KeepPointWithinConstraint, KeepRelativePositionConstraint,\
    KeepPortDistanceConstraint
from awesome_tool.mvc.views.gap.line import PerpLine

from awesome_tool.mvc.models.transition import TransitionModel
from awesome_tool.mvc.models.data_flow import DataFlowModel

from awesome_tool.mvc.views.gap.ports import PortView, IncomeView, InputPortView, ScopedDataInputPortView,\
    OutcomeView, OutputPortView, ScopedDataOutputPortView, ScopedVariablePortView

from awesome_tool.mvc.controllers.gap.enums import SnappedSide, Direction

from awesome_tool.utils import constants
from awesome_tool.mvc.config import global_gui_config

from awesome_tool.mvc.controllers.gap import gap_draw_helper

from gtkmvc import Observer

import cairo
from pango import FontDescription, SCALE
from gtk.gdk import Color, CairoContext

from math import pi, atan2


# class PConnectionView(Line):
#
#     def __init__(self, hierarchy_level, perpendicular_ends=False):
#         self._from_port = None
#         self._to_port = None
#
#         super(PConnectionView, self).__init__()
#         self._from_handle = self.handles()[0]
#         self._to_handle = self.handles()[1]
#         self._segment = Segment(self, view=self.canvas)
#         self.hierarchy_level = hierarchy_level
#
#         self._line_color = ""
#         self._arrow_color = ""
#
#         self._head_length = 0.
#         self._to_head_length = 0.
#
#         self._perpendicular_ends = perpendicular_ends
#
#     @property
#     def name(self):
#         if self.from_port:
#             return self.from_port.name
#         return None
#
#     @property
#     def parent(self):
#         return self.canvas.get_parent(self)
#
#     @property
#     def from_port(self):
#         return self._from_port
#
#     @property
#     def to_port(self):
#         return self._to_port
#
#     @from_port.setter
#     def from_port(self, port):
#         assert isinstance(port, PortView)
#         self._from_port = port
#         if self.to_port:
#             self.line_width = min(self.to_port.port_side_size, port.port_side_size) * .2
#         else:
#             self.line_width = port.port_side_size * .2
#         self._head_length = port.port_side_size
#
#     @to_port.setter
#     def to_port(self, port):
#         assert isinstance(port, PortView)
#         self._to_port = port
#         if self.from_port:
#             self.line_width = min(self.from_port.port_side_size, port.port_side_size) * .2
#
#     @property
#     def waypoints(self):
#         waypoints = []
#         for handle in self.handles():
#             if handle not in self.end_handles():
#                 waypoints.append(handle)
#         return waypoints
#
#     def end_handles(self):
#         return [self.from_handle(), self.to_handle()]
#
#     def reset_from_port(self):
#         self._from_port = None
#
#     def reset_to_port(self):
#         self._to_port = None
#
#     def set_port_for_handle(self, port, handle):
#         if handle is self.from_handle():
#             self.from_port = port
#         elif handle is self.to_handle():
#             self.to_port = port
#
#     def reset_port_for_handle(self, handle):
#         if handle is self.from_handle():
#             self.reset_from_port()
#         elif handle is self.to_handle():
#             self.reset_to_port()
#
#     def remove_connection_from_port(self, port):
#         if self._from_port and port is self._from_port:
#             self._from_port.remove_connected_handle(self._from_handle)
#         elif self._to_port and port is self._to_port:
#             self._to_port.remove_connected_handle(self._to_handle)
#
#     def remove_connection_from_ports(self):
#         if self._from_port:
#             self._from_port.remove_connected_handle(self._from_handle)
#             self._from_port.tmp_disconnect()
#         if self._to_port:
#             self._to_port.remove_connected_handle(self._to_handle)
#             self.to_port.tmp_disconnect()
#
#     def _keep_handle_in_parent_state(self, handle):
#         canvas = self.canvas
#         parent = canvas.get_parent(self)
#         solver = canvas.solver
#         if parent is None:
#             return
#         handle_pos_abs = canvas.project(self, handle.pos)
#         parent_nw_abs = canvas.project(parent, parent.handles()[NW].pos)
#         parent_se_abs = canvas.project(parent, parent.handles()[SE].pos)
#         constraint = KeepPointWithinConstraint(parent_nw_abs, parent_se_abs, handle_pos_abs)
#         solver.add_constraint(constraint)
#
#     def from_handle(self):
#         return self._from_handle
#
#     def to_handle(self):
#         return self._to_handle
#
#     def add_waypoint(self, pos):
#         handle = self._create_handle(pos)
#         self._handles.insert(-1, handle)
#         self._keep_handle_in_parent_state(handle)
#         self._update_ports()
#
#     def _reversible_insert_handle(self, index, handle):
#         super(ConnectionView, self)._reversible_insert_handle(index, handle)
#         self._keep_handle_in_parent_state(handle)
#
#     def draw_head(self, context):
#         cr = context.cairo
#         cr.set_source_color(Color(self._arrow_color))
#         cr.move_to(0, 0)
#         cr.line_to(self._head_length, 0)
#         cr.stroke()
#         cr.move_to(self._head_length, 0)
#         return
#
#     def draw_tail(self, context):
#         cr = context.cairo
#         cr.set_source_color(Color(self._line_color))
#         cr.line_to(2.5 / self.hierarchy_level, 0)
#         cr.stroke()
#         cr.set_source_color(Color(self._arrow_color))
#         cr.move_to(2.5 / self.hierarchy_level, 0)
#         cr.line_to(0, 0)
#         cr.line_to(1.0 / self.hierarchy_level, 1.0 / self.hierarchy_level)
#         cr.move_to(0, 0)
#         cr.line_to(1.0 / self.hierarchy_level, -1.0 / self.hierarchy_level)
#         cr.stroke()
#
#     @staticmethod
#     def is_in_port(port):
#         return isinstance(port, (IncomeView, InputPortView, ScopedDataInputPortView))
#
#     @staticmethod
#     def is_out_port(port):
#         return isinstance(port, (OutcomeView, OutputPortView, ScopedDataOutputPortView))
#
#     def post_update(self, context):
#         super(Line, self).post_update(context)
#         if not self._perpendicular_ends:
#             h0, h1 = self._handles[:2]
#             p0, p1 = h0.pos, h1.pos
#             self._head_angle = atan2(p1.y - p0.y, p1.x - p0.x)
#             h1, h0 = self._handles[-2:]
#             p1, p0 = h1.pos, h0.pos
#             self._tail_angle = atan2(p1.y - p0.y, p1.x - p0.x)
#         else:
#             if self.from_port is None:
#                 h0, h1 = self._handles[:2]
#                 p0, p1 = h0.pos, h1.pos
#                 self._head_angle = atan2(p1.y - p0.y, p1.x - p0.x)
#             elif self.from_port.side is SnappedSide.RIGHT:
#                 self._head_angle = 0 if self.is_out_port(self.from_port) else pi
#             elif self.from_port.side is SnappedSide.TOP:
#                 self._head_angle = pi * 3. / 2. if self.is_out_port(self.from_port) else pi / 2.
#             elif self.from_port.side is SnappedSide.LEFT:
#                 self._head_angle = pi if self.is_out_port(self.from_port) else 0
#             elif self.from_port.side is SnappedSide.BOTTOM:
#                 self._head_angle = pi / 2. if self.is_out_port(self.from_port) else pi * 3. / 2.
#
#             if self.to_port is None:
#                 h1, h0 = self._handles[-2:]
#                 p1, p0 = h1.pos, h0.pos
#                 self._tail_angle = atan2(p1.y - p0.y, p1.x - p0.x)
#             elif self.to_port.side is SnappedSide.RIGHT:
#                 self._tail_angle = pi if self.is_out_port(self.to_port) else 0
#             elif self.to_port.side is SnappedSide.TOP:
#                 self._tail_angle = pi / 2. if self.is_out_port(self.to_port) else pi * 3. / 2.
#             elif self.to_port.side is SnappedSide.LEFT:
#                 self._tail_angle = 0 if self.is_out_port(self.to_port) else pi
#             elif self.to_port.side is SnappedSide.BOTTOM:
#                 self._tail_angle = pi * 3. / 2. if self.is_out_port(self.to_port) else pi / 2.
#
#     def draw(self, context):
#         if self.parent and self.parent.moving:
#             return
#
#         def draw_line_end(pos, angle, draw):
#             cr = context.cairo
#             cr.save()
#             try:
#                 cr.translate(*pos)
#                 cr.rotate(angle)
#                 draw(context)
#             finally:
#                 cr.restore()
#
#         cr = context.cairo
#         cr.set_line_width(self.line_width)
#         draw_line_end(self._handles[0].pos, self._head_angle, self.draw_head)
#         for h in self._handles[1:-1]:
#             cr.line_to(*h.pos)
#         draw_line_end(self._handles[-1].pos, self._tail_angle, self.draw_tail)
#         cr.stroke()
#
#         if self.name:
#             self._draw_name(context)
#
#     def _draw_name(self, context):
#         c = context.cairo
#
#         # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
#         if isinstance(c, CairoContext):
#             cc = c
#         else:
#             cc = c._cairo
#
#         if len(self._handles) % 2:
#             index = len(self._handles) / 2
#             cx, cy = self._handles[index].pos
#             angle = 0
#         else:
#             index1 = len(self._handles) / 2 - 1
#             index2 = index1 + 1
#
#             p1, p2 = self._handles[index1].pos, self._handles[index2].pos
#
#             cx = (p1.x + p2.x) / 2
#             cy = (p1.y + p2.y) / 2
#
#             angle = atan2(p2.y - p1.y, p2.x - p1.x)
#             if angle < -pi / 2.:
#                 angle += pi
#             elif angle > pi / 2.:
#                 angle -= pi
#
#         outcome_side = self.from_port.port_side_size
#
#         pcc = CairoContext(cc)
#         pcc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)
#
#         layout = pcc.create_layout()
#         layout.set_text(self.name)
#
#         font_name = constants.FONT_NAMES[0]
#         font_size = outcome_side
#
#         font = FontDescription(font_name + " " + str(font_size))
#         layout.set_font_description(font)
#
#         cc.set_source_color(Color('#ededee'))
#         pcc.update_layout(layout)
#         c.save()
#
#         c.move_to(cx, cy)
#         if global_gui_config.get_config_value("ROTATE_NAMES_ON_CONNECTIONS", default=False):
#             c.rotate(angle)
#
#         pcc.show_layout(layout)
#
#         c.restore()
#
#     def _get_tail_pos(self):
#         if self.to_port is None:
#             return self._handles[-1].pos
#
#         tail_pos = None
#
#         to_port_left_side = (self._handles[-1].pos.x.value - self.to_port.port_side_size / 2., self._handles[-1].pos.y.value)
#         to_port_right_side = (self._handles[-1].pos.x.value + self.to_port.port_side_size / 2., self._handles[-1].pos.y.value)
#         to_port_top_side = (self._handles[-1].pos.x.value, self._handles[-1].pos.y.value - self.to_port.port_side_size / 2.)
#         to_port_bottom_side = (self._handles[-1].pos.x.value, self._handles[-1].pos.y.value + self.to_port.port_side_size / 2.)
#
#         if self.to_port.side is SnappedSide.RIGHT:
#             tail_pos = to_port_left_side if self.is_out_port(self.to_port) else to_port_right_side
#         elif self.to_port.side is SnappedSide.TOP:
#             tail_pos = to_port_bottom_side if self.is_out_port(self.to_port) else to_port_top_side
#         elif self.to_port.side is SnappedSide.LEFT:
#             tail_pos = to_port_right_side if self.is_out_port(self.to_port) else to_port_left_side
#         elif self.to_port.side is SnappedSide.BOTTOM:
#             tail_pos = to_port_top_side if self.is_out_port(self.to_port) else to_port_bottom_side
#
#         return tail_pos


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
            self._line_color = '#81848b'
            self._arrow_color = '#ffffff'
        else:
            self._line_color = '#6c5e3c'
            self._arrow_color = '#ffC926'


class TransitionView(ConnectionView):

    def __init__(self, transition_m, hierarchy_level):
        super(TransitionView, self).__init__(hierarchy_level)
        self._transition_m = None
        self.model = transition_m
        self.line_width = .5 / hierarchy_level

        self._line_color = '#81848b'
        self._arrow_color = '#ffffff'

    @property
    def model(self):
        return self._transition_m()

    @model.setter
    def model(self, transition_model):
        assert isinstance(transition_model, TransitionModel)
        self._transition_m = ref(transition_model)


class DataFlowView(ConnectionView):

    def __init__(self, data_flow_m, hierarchy_level):
        super(DataFlowView, self).__init__(hierarchy_level)
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = None
        self.model = data_flow_m
        self.line_width = .5 / hierarchy_level

        self._line_color = '#6c5e3c'
        self._arrow_color = '#ffC926'

    @property
    def model(self):
        return self._data_flow_m()

    @model.setter
    def model(self, data_flow_m):
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = ref(data_flow_m)


class ScopedVariableDataFlowView(DataFlowView, Observer):

    def __init__(self, data_flow_m, hierarchy_level, name):
        Observer.__init__(self)
        super(ScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level)

        self._name_width = 10.
        self._name_width_updated = False

        self._print_side = SnappedSide.LEFT
        self._label_selection_waypoint = None

        self._name = None
        self.name = name

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
        return self._name

    @name.setter
    def name(self, name):
        assert isinstance(name, str)
        self._name = name
        
    @property
    def connected(self):
        raise NotImplementedError

    @property
    def desired_name_height(self):
        return self._head_length * 1.5

    def draw(self, context):
        if not self.connected:
            super(ScopedVariableDataFlowView, self).draw(context)
        else:
            self._draw_label(context)

    def _draw_label(self, context):
        raise NotImplementedError


class FromScopedVariableDataFlowView(ScopedVariableDataFlowView):

    def __init__(self, data_flow_m, hierarchy_level, name):
        super(FromScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level, name)

    @property
    def connected(self):
        return self._from_port is not None

    @property
    def from_port(self):
        return self._from_port

    @property
    def desired_name_height(self):
        return self._to_head_length * 1.5

    @from_port.setter
    def from_port(self, port):
        # TODO: change ScopedDataInputPortView to port type of scoped variable in state border
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

            self._name = port.model.scoped_variable.name
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

    def _draw_label(self, context):
        if self.parent and self.parent.moving:
            return

        c = context.cairo
        c.set_line_width(self._head_length * .03)

        handle_pos = self.to_handle().pos
        port_side_size = self._to_head_length

        c.set_source_color(Color('#f00'))

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

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
        while layout.get_size()[1] / float(SCALE) > self.desired_name_height:
            font_size *= 0.9
            set_font_description()

        name_size = layout.get_size()[0] / float(SCALE), layout.get_size()[1] / float(SCALE)
        self._name_width = layout.get_size()[0] / float(SCALE)
        self._update_label_selection_waypoint()

        if self.to_port:
            self._print_side = self.to_port.side

        rot_angle, move_x, move_y = gap_draw_helper.draw_name_label(context, '#00f', name_size, handle_pos,
                                                                    self._print_side, port_side_size)

        c.move_to(move_x, move_y)

        pcc.update_layout(layout)
        pcc.rotate(rot_angle)
        pcc.show_layout(layout)
        pcc.rotate(-rot_angle)


class ToScopedVariableDataFlowView(ScopedVariableDataFlowView):

    def __init__(self, data_flow_m, hierarchy_level, name):
        super(ToScopedVariableDataFlowView, self).__init__(data_flow_m, hierarchy_level, name)

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
            self._name = port.model.scoped_variable.name
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
                pos_x = self.from_handle().x - 2 * self._head_length - self._name_width
                pos_y = self.from_handle().y
            elif self.from_port.side is SnappedSide.RIGHT:
                pos_x = self.from_handle().x + 2 * self._head_length + self._name_width
                pos_y = self.from_handle().y
            elif self.from_port.side is SnappedSide.TOP:
                pos_x = self.from_handle().x
                pos_y = self.from_handle().y - 2 * self._head_length - self._name_width
            elif self.from_port.side is SnappedSide.BOTTOM:
                pos_x = self.from_handle().x
                pos_y = self.from_handle().y + 2 * self._head_length + self._name_width
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

    def _draw_label(self, context):
        if self.parent and self.parent.moving:
            return

        c = context.cairo
        c.set_line_width(self._head_length * .03)

        handle_pos = self.from_handle().pos
        port_side_size = self._head_length

        c.set_source_color(Color('#f00'))

        # Ensure that we have CairoContext anf not CairoBoundingBoxContext (needed for pango)
        if isinstance(c, CairoContext):
            cc = c
        else:
            cc = c._cairo

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
        while layout.get_size()[1] / float(SCALE) > self.desired_name_height:
            font_size *= 0.9
            set_font_description()

        name_size = layout.get_size()[0] / float(SCALE), layout.get_size()[1] / float(SCALE)
        self._name_width = layout.get_size()[0] / float(SCALE)
        self._update_label_selection_waypoint()

        if self.from_port:
            self._print_side = self.from_port.side

        rot_angle, move_x, move_y = gap_draw_helper.draw_name_label(context, '#f00', name_size, handle_pos,
                                                                    self._print_side, port_side_size)

        c.move_to(move_x, move_y)

        pcc.update_layout(layout)
        pcc.rotate(rot_angle)
        pcc.show_layout(layout)
        pcc.rotate(-rot_angle)