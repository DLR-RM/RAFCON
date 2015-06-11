from weakref import ref

from gaphas.segment import Segment, HandleSelection, SegmentHandleSelection
from gaphas.item import Line, NW, SE

from awesome_tool.mvc.views.gap.constraint import KeepPointWithinConstraint

from awesome_tool.mvc.models.transition import TransitionModel
from awesome_tool.mvc.models.data_flow import DataFlowModel

from awesome_tool.mvc.views.gap.ports import PortView

from gtk.gdk import Color


class ConnectionView(Line):

    def __init__(self, hierarchy_level):
        super(ConnectionView, self).__init__()
        self._from_handle = self.handles()[0]
        self._to_handle = self.handles()[1]
        self._segment = Segment(self, view=self.canvas)
        self.hierarchy_level = hierarchy_level

        self._from_port = None
        self._to_port = None

        self._line_color = ""
        self._arrow_color = ""

        # self.orthogonal = True

    # def setup_canvas(self):
    #     super(ConnectionView, self).setup_canvas()

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

    @to_port.setter
    def to_port(self, port):
        assert isinstance(port, PortView)
        self._to_port = port

    def end_handles(self):
        return [self.from_handle(), self.to_handle()]

    def reset_from_port(self):
        self._from_port = None

    def reset_to_port(self):
        self._to_port = None

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
        if self._to_port:
            self._to_port.remove_connected_handle(self._to_handle)

    def _keep_handle_in_parent_state(self, handle):
        canvas = self.canvas
        parent = canvas.get_parent(self)
        solver = canvas.solver
        if parent is None:
            return
        handle_pos_abs = canvas.project(self, handle.pos)
        parent_nw_abs = canvas.project(parent, parent.handles()[NW].pos)
        parent_se_abs = canvas.project(parent, parent.handles()[SE].pos)
        constraint = KeepPointWithinConstraint(parent_nw_abs, parent_se_abs, handle_pos_abs)
        solver.add_constraint(constraint)

    def from_handle(self):
        return self._from_handle

    def to_handle(self):
        return self._to_handle

    def add_waypoint(self, pos):
        handle = self._create_handle(pos)
        self._handles.insert(-1, handle)
        self._keep_handle_in_parent_state(handle)
        self._update_ports()

    def _reversible_insert_handle(self, index, handle):
        super(ConnectionView, self)._reversible_insert_handle(index, handle)
        self._keep_handle_in_parent_state(handle)

    def draw_head(self, context):
        cr = context.cairo
        cr.set_source_color(Color(self._arrow_color))
        cr.rectangle(0, -.5 / 4 / self.hierarchy_level, .5 / 2 / self.hierarchy_level, .5/ 2 / self.hierarchy_level)
        cr.fill_preserve()
        cr.move_to(0, 0)
        cr.line_to(2.5 / self.hierarchy_level, 0)
        cr.stroke()
        cr.move_to(2.5 / self.hierarchy_level, 0)
        return

    def draw_tail(self, context):
        cr = context.cairo
        cr.set_source_color(Color(self._line_color))
        cr.line_to(2.5 / self.hierarchy_level, 0)
        cr.stroke()
        cr.set_source_color(Color(self._arrow_color))
        cr.move_to(2.5 / self.hierarchy_level, 0)
        cr.line_to(0, 0)
        cr.line_to(1.0 / self.hierarchy_level, 1.0 / self.hierarchy_level)
        cr.move_to(0, 0)
        cr.line_to(1.0 / self.hierarchy_level, -1.0 / self.hierarchy_level)
        cr.stroke()


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
        self.transition_m = transition_m
        self.line_width = .5 / hierarchy_level

        self._line_color = '#81848b'
        self._arrow_color = '#ffffff'

    @property
    def transition_m(self):
        return self._transition_m()

    @transition_m.setter
    def transition_m(self, transition_model):
        assert isinstance(transition_model, TransitionModel)
        self._transition_m = ref(transition_model)


class DataFlowView(ConnectionView):

    def __init__(self, data_flow_m, hierarchy_level):
        super(DataFlowView, self).__init__(hierarchy_level)
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = ref(data_flow_m)
        self.line_width = 0.5

        self._line_color = '#6c5e3c'
        self._arrow_color = '#ffC926'

    @property
    def data_flow_m(self):
        return self._data_flow_m()

    @data_flow_m.setter
    def data_flow_m(self, data_flow_m):
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = data_flow_m


@HandleSelection.when_type(ConnectionView)
class ConnectionSegmentHandleSelection(SegmentHandleSelection):

    def unselect(self):
        self.view.canvas.solver.solve()
        super(ConnectionSegmentHandleSelection, self).unselect()