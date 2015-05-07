from awesome_tool.utils import log
logger = log.get_logger(__name__)

import gtk

from gtkmvc import View

from weakref import ref

from gaphas import GtkView
from gaphas.item import Element, Line, NW, NE,SW, SE
from gaphas.constraint import Constraint
from gaphas.connector import PointPort, Handle
from gaphas.geometry import distance_point_point
from gaphas.util import path_ellipse
from gaphas.aspect import Connector
from gaphas.segment import Segment, HandleSelection, SegmentHandleSelection

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.transition import TransitionModel
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
        length = distance_point_point(self._line[0], self._line[1])
        dx = length / (len(self._points) + 1)

        for index, p in enumerate(self._points):
            pos = p[0]
            pos.x = self._line[0][0]
            pos.y = self._line[0][1] + (index + 1) * dx


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

        self._income = IncomeView()
        self.constraint(line=(self._income.pos, (self._handles[NW].pos, self._handles[SW].pos)), align=0.5)

        self._outcomes = []
        self._outcomes_distribution = EqualDistributionConstraint((self._handles[NE].pos, self._handles[SE].pos))

    def setup_canvas(self):
        canvas = self.canvas
        parent = canvas.get_parent(self)

        solver = canvas.solver
        solver.add_constraint(self._outcomes_distribution)

        if parent is not None:
            assert isinstance(parent, StateView)
            self_nw_abs = canvas.project(self, self.nw_pos())
            self_se_abs = canvas.project(self, self.se_pos())
            parent_nw_abs = canvas.project(parent, parent.nw_pos())
            parent_se_abs = canvas.project(parent, parent.se_pos())
            less_than = KeepRectangleWithinConstraint(parent_nw_abs, parent_se_abs, self_nw_abs, self_se_abs)
            solver.add_constraint(less_than)

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

    def connect_to_income(self, item, handle):
        c = self._income.port.constraint(self.canvas, item, handle, self)
        self.canvas.connect_item(item, handle, self, self._income.port, c)

    def connect_to_outcome(self, outcome_id, item, handle):
        outcome_v = self.outcome_port(outcome_id)
        outcome_port = outcome_v.port
        c = outcome_port.constraint(self.canvas, item, handle, self)
        self.canvas.connect_item(item, handle, self, outcome_port, c)

    def income_port(self):
        return self._income

    def outcome_port(self, outcome_id):
        for outcome in self._outcomes:
            if outcome.outcome_id == outcome_id:
                return outcome
        raise AttributeError("Outcome with id '{0}' not found in state".format(outcome_id, self.state_m.state.name))

    def nw_pos(self):
        return self._handles[NW].pos

    def se_pos(self):
        return self._handles[SE].pos

    def add_outcome(self, outcome_m):
        outcome_v = OutcomeView(outcome_m)
        self._outcomes.append(outcome_v)
        self._ports.append(outcome_v.port)
        self._handles.append(outcome_v.handle)
        self._outcomes_distribution.add_point(outcome_v.pos, outcome_v.sort)


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
        parent_nw_abs = canvas.project(parent, parent.nw_pos())
        parent_se_abs = canvas.project(parent, parent.se_pos())
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

    def __init__(self):
        super(TransitionView, self).__init__()
        self.line_width = 0.5


@HandleSelection.when_type(ConnectionView)
class ConnectionSegmentHandleSelection(SegmentHandleSelection):

    def unselect(self):
        self.view.canvas.solver.solve()
        super(ConnectionSegmentHandleSelection, self).unselect()