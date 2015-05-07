from awesome_tool.utils import log
logger = log.get_logger(__name__)

import gtk

from gtkmvc import View

from weakref import ref

from gaphas import GtkView
from gaphas.item import Element, Line, NW, NE,SW, SE
from gaphas.constraint import Constraint
from gaphas.connector import PointPort, Handle, Position
from gaphas.geometry import distance_point_point
from gaphas.util import path_ellipse
from gaphas.aspect import Connector
from gaphas.segment import Segment, HandleSelection, SegmentHandleSelection

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel
from awesome_tool.mvc.models.transition import TransitionModel
from awesome_tool.mvc.models.data_flow import DataFlowModel
from awesome_tool.mvc.models.state import StateModel


class GraphicalEditorView(View):
    top = 'main_frame'

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        View.__init__(self)

        self.v_box = gtk.VBox()
        self.scroller = gtk.ScrolledWindow()
        self.editor = GtkView()
        self.editor.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#272c36'))
        self.scroller.add(self.editor)
        self.v_box.pack_end(self.scroller)

        self['main_frame'] = self.v_box

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(500, 500)


class KeepRectangleWithinConstraint(Constraint):
    """Ensure that the children is within its parent

    Attributes:
     - parent_nw: NW coordinates of parent
     - parent_se: SE coordinates of parent
     - child_nw: NW coordinates of child
     - child_se: SE coordinates of child
    """

    def __init__(self, parent_nw, parent_se, child_nw, child_se, margin=None):
        super(KeepRectangleWithinConstraint, self).__init__(parent_nw[0], parent_nw[1], parent_se[0], parent_se[1],
                                                            child_nw[0], child_nw[1], child_se[0], child_se[1])
        self.parent_nw = parent_nw
        self.parent_se = parent_se
        self.child_nw = child_nw
        self.child_se = child_se

        self.margin = margin
        min_margin = 0  # (parent_se[0].value - parent_nw[0].value) / 1000.
        if margin is None or margin < min_margin:
            self.margin = min_margin

    def solve_for(self, var=None):
        """
        Ensure that the children is within its parent
        """
        child_width = self.child_se[0].value - self.child_nw[0].value
        child_height = self.child_se[1].value - self.child_nw[1].value
        # Left edge (west)
        if self.parent_nw[0].value > self.child_nw[0].value - self.margin:
            self.child_nw[0].value = self.parent_nw[0].value + self.margin
            self.child_se[0].value = self.child_nw[0].value + child_width
        # Right edge (east)
        if self.parent_se[0].value < self.child_se[0].value + self.margin:
            self.child_se[0].value = self.parent_se[0].value - self.margin
            self.child_nw[0].value = self.child_se[0].value - child_width
        # Upper edge (north)
        if self.parent_nw[1].value > self.child_nw[1].value - self.margin:
            self.child_nw[1].value = self.parent_nw[1].value + self.margin
            self.child_se[1].value = self.child_nw[1].value + child_height
        # Lower edge (south)
        if self.parent_se[1].value < self.child_se[1].value + self.margin:
            self.child_se[1].value = self.parent_se[1].value - self.margin
            self.child_nw[1].value = self.child_se[1].value - child_height


class KeepPointWithinConstraint(KeepRectangleWithinConstraint):
    """Ensure that the children is within its parent

    Attributes:
     - parent_nw: NW coordinates of parent
     - parent_se: SE coordinates of parent
     - child_pos: coordinates of child
    """

    def __init__(self, parent_nw, parent_se, child_pos, margin=None):
        super(KeepPointWithinConstraint, self).__init__(parent_nw, parent_se, child_pos, child_pos)


class EqualDistributionConstraint(Constraint):

    def __init__(self, line):
        super(EqualDistributionConstraint, self).__init__(line[0][0], line[0][1], line[1][0], line[1][1])
        self._line = line
        self._points = []

    def add_point(self, p, sort=None):
        if sort is None:
            sort = len(self._points)
        self._points.append((p, sort))
        self._variables.append(p[0])
        self._variables.append(p[1])
        self.create_weakest_list()
        self._sort_points()

    def _sort_points(self):
        self._points.sort(lambda p1, p2: cmp(p1[1], p2[1]))

    def remove_point(self, p):
        try:
            self._variables.remove(p[0])
            self._variables.remove(p[1])
            self.create_weakest_list()
            for point_entry in self._points:
                if point_entry[0] is p:
                    break
            self._points.remove(point_entry)
        except ValueError:
            logger.error("Cannot remove point '{0}' of constraint".format(p))

    def solve_for(self, var=None):
        from math import atan2, sin, cos
        alpha = atan2(self._line[1][1] - self._line[0][1], self._line[1][0] - self._line[0][0])
        length = distance_point_point(self._line[0], self._line[1])
        ds = length / (len(self._points) + 1)
        dx = ds * cos(alpha)
        dy = ds * sin(alpha)

        for index, p in enumerate(self._points):
            pos = p[0]
            pos.x = self._line[0][0] + (index + 1) * dx
            pos.y = self._line[0][1] + (index + 1) * dy


class StateView(Element):
    """ A State has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    def __init__(self, state_m, size):
        super(StateView, self).__init__(size[0], size[1])
        assert isinstance(state_m, StateModel)

        self._state_m = ref(state_m)

        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

        self._left_center = Position((0, self.height / 2.))
        self._right_center = Position((self.width, self.height / 2.))
        self.constraint(line=(self._left_center, (self._handles[NW].pos, self._handles[SW].pos)), align=0.5)
        self.constraint(line=(self._right_center, (self._handles[NE].pos, self._handles[SE].pos)), align=0.5)

        self._income = IncomeView()
        self.constraint(line=(self._income.pos, (self._handles[NW].pos, self._left_center)), align=0.5)

        self._outcomes = []
        self._outcomes_distribution = EqualDistributionConstraint((self._handles[NE].pos, self._right_center))

        self._inputs = []
        self._inputs_distribution = EqualDistributionConstraint((self._handles[SW].pos, self._left_center))

        self._outputs = []
        self._outputs_distribution = EqualDistributionConstraint((self._handles[SE].pos, self._right_center))

        self._scoped_variables = []

    def setup_canvas(self):
        canvas = self.canvas
        parent = canvas.get_parent(self)

        solver = canvas.solver
        solver.add_constraint(self._outcomes_distribution)
        solver.add_constraint(self._inputs_distribution)
        solver.add_constraint(self._outputs_distribution)

        if parent is not None:
            assert isinstance(parent, StateView)
            self_nw_abs = canvas.project(self, self.handles()[NW].pos)
            self_se_abs = canvas.project(self, self.handles()[SE].pos)
            parent_nw_abs = canvas.project(parent, parent.handles()[NW].pos)
            parent_se_abs = canvas.project(parent, parent.handles()[SE].pos)
            constraint = KeepRectangleWithinConstraint(parent_nw_abs, parent_se_abs, self_nw_abs, self_se_abs)
            solver.add_constraint(constraint)

        # Registers local constraints
        super(StateView, self).setup_canvas()

    @property
    def state_m(self):
        return self._state_m()

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

        self._income.draw(context, self)

        for outcome in self._outcomes:
            outcome.draw(context, self)

        for input in self._inputs:
            input.draw(context, self)

        for output in self._outputs:
            output.draw(context, self)

    def connect_to_income(self, item, handle):
        self._connect_to_port(self._income.port, item, handle)

    def connect_to_outcome(self, outcome_id, item, handle):
        outcome_v = self.outcome_port(outcome_id)
        self._connect_to_port(outcome_v.port, item, handle)

    def connect_to_input_port(self, port_id, item, handle):
        port_v = self.input_port(port_id)
        self._connect_to_port(port_v.port, item, handle)

    def connect_to_output_port(self, port_id, item, handle):
        port_v = self.output_port(port_id)
        self._connect_to_port(port_v.port, item, handle)

    def connect_to_scoped_variable_input(self, scoped_variable_id, item, handle):
        scoped_variable_v = self.scoped_variable(scoped_variable_id)
        c = scoped_variable_v.input_port.constraint(self.canvas, item, handle, scoped_variable_v)
        self.canvas.connect_item(item, handle, scoped_variable_v, scoped_variable_v.input_port, c)

    def connect_to_scoped_variable_output(self, scoped_variable_id, item, handle):
        scoped_variable_v = self.scoped_variable(scoped_variable_id)
        c = scoped_variable_v.output_port.constraint(self.canvas, item, handle, scoped_variable_v)
        self.canvas.connect_item(item, handle, scoped_variable_v, scoped_variable_v.output_port, c)


    def _connect_to_port(self, port, item, handle):
        c = port.constraint(self.canvas, item, handle, self)
        self.canvas.connect_item(item, handle, self, port, c)

    def income_port(self):
        return self._income

    def outcome_port(self, outcome_id):
        for outcome in self._outcomes:
            if outcome.outcome_id == outcome_id:
                return outcome
        raise AttributeError("Outcome with id '{0}' not found in state".format(outcome_id, self.state_m.state.name))

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
        raise AttributeError("Port with id '{0}' not found in state".format(port_id, self.state_m.state.name))

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)
        self._outcomes_distribution.add_point(outcome_v.pos, outcome_v.sort)

    def add_input_port(self, port_m):
        input_port_v = InputPortView(port_m)
        self._inputs.append(input_port_v)
        self._ports.append(input_port_v.port)
        self._handles.append(input_port_v.handle)
        self._inputs_distribution.add_point(input_port_v.pos, input_port_v.sort)

    def add_output_port(self, port_m):
        output_port_v = OutputPortView(port_m)
        self._outputs.append(output_port_v)
        self._ports.append(output_port_v.port)
        self._handles.append(output_port_v.handle)
        self._outputs_distribution.add_point(output_port_v.pos, output_port_v.sort)

    def add_scoped_variable(self, scoped_variable_m, size):
        scoped_variable_v = ScopedVariableView(scoped_variable_m, size)
        self._scoped_variables.append(scoped_variable_v)

        canvas = self.canvas
        canvas.add(scoped_variable_v, self)

        self_nw_abs = canvas.project(self, self.handles()[NW].pos)
        self_se_abs = canvas.project(self, self.handles()[SE].pos)
        scoped_nw_abs = canvas.project(scoped_variable_v, scoped_variable_v.handles()[NW].pos)
        scoped_se_abs = canvas.project(scoped_variable_v, scoped_variable_v.handles()[SE].pos)
        constraint = KeepRectangleWithinConstraint(self_nw_abs, self_se_abs, scoped_nw_abs, scoped_se_abs)
        solver = canvas.solver
        solver.add_constraint(constraint)

        return scoped_variable_v


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


class PortView(object):

    def __init__(self):
        self.handle = Handle(connectable=True, movable=False)
        self.port = PointPort(self.handle.pos)

    @property
    def pos(self):
        return self.handle.pos

    def draw(self, context, state):
        raise NotImplementedError


class IncomeView(PortView):

    def __init__(self):
        super(IncomeView, self).__init__()

    def draw(self, context, state):
        c = context.cairo
        min_state_side = min(state.width, state.height)
        outcome_side = min_state_side / 20.
        path_ellipse(c, self.pos.x, self.pos.y, outcome_side, outcome_side)


class OutcomeView(PortView):

    def __init__(self, outcome_m):
        super(OutcomeView, self).__init__()

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
        c = context.cairo
        min_state_side = min(state.width, state.height)
        outcome_side = min_state_side / 20.
        c.set_line_width(0.25)
        # c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        path_ellipse(c, self.pos.x, self.pos.y, outcome_side, outcome_side)


class DataPortView(PortView):

    def __init__(self, port_m):
        super(DataPortView, self).__init__()

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
        c = context.cairo
        min_state_side = min(state.width, state.height)
        port_side = min_state_side / 25.
        c.set_line_width(0.15)
        c.rectangle(self.pos.x - port_side / 2, self.pos.y - port_side / 2, port_side, port_side)


class InputPortView(DataPortView):
    pass


class OutputPortView(DataPortView):
    pass


class ConnectionView(Line):

    def __init__(self):
        super(ConnectionView, self).__init__()
        self._from_handle = self.handles()[0]
        self._to_handle = self.handles()[1]
        self._segment = Segment(self, view=self.canvas)
        # self.orthogonal = True

    # def setup_canvas(self):
    #     super(ConnectionView, self).setup_canvas()

    def _keep_handle_in_parent_state(self, handle):
        canvas = self.canvas
        parent = canvas.get_parent(self)
        solver = canvas.solver
        if parent is None:
            return
        assert isinstance(parent, StateView)
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
        cr.move_to(0, 0)
        return

    def draw_tail(self, context):
        cr = context.cairo
        cr.line_to(0, 0)
        cr.line_to(2, 2)
        cr.line_to(2, -2)
        cr.line_to(0, 0)
        cr.stroke()


class TransitionView(ConnectionView):

    def __init__(self, transition_m):
        super(TransitionView, self).__init__()
        assert isinstance(transition_m, TransitionModel)
        self._transition_m = ref(transition_m)
        self.line_width = 0.5


class DataFlowView(ConnectionView):

    def __init__(self, data_flow_m):
        super(DataFlowView, self).__init__()
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = ref(data_flow_m)
        self.line_width = 0.5


@HandleSelection.when_type(ConnectionView)
class ConnectionSegmentHandleSelection(SegmentHandleSelection):

    def unselect(self):
        self.view.canvas.solver.solve()
        super(ConnectionSegmentHandleSelection, self).unselect()