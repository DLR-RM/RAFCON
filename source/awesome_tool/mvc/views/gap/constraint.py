from awesome_tool.utils import log
logger = log.get_logger(__name__)

from gaphas.geometry import distance_point_point
from gaphas.constraint import Constraint
from awesome_tool.mvc.views.gap.ports import SnappedSide

from copy import deepcopy

EPSILON = 1e-6


def _update(variable, value):
    if abs(variable.value - value) > EPSILON:
        variable.value = value


class KeepRectangleWithinConstraint(Constraint):
    """Ensure that the children is within its parent

    Attributes:
     - parent_nw: NW coordinates of parent
     - parent_se: SE coordinates of parent
     - child_nw: NW coordinates of child
     - child_se: SE coordinates of child
    """

    def __init__(self, parent_nw, parent_se, child_nw, child_se, child=None, margin=None):
        super(KeepRectangleWithinConstraint, self).__init__(parent_nw[0], parent_nw[1], parent_se[0], parent_se[1],
                                                            child_nw[0], child_nw[1], child_se[0], child_se[1])
        self.parent_nw = parent_nw
        self.parent_se = parent_se
        self.child_nw = child_nw
        self.child_se = child_se
        self.child = child

        self.margin = margin
        min_margin = 0  # (parent_se[0].value - parent_nw[0].value) / 1000.
        if margin is None or margin < min_margin:
            self.margin = min_margin

    def solve_for(self, var=None):
        """
        Ensure that the children is within its parent
        """

        from gaphas.solver import Projection
        while isinstance(var, Projection):
            var = var.variable()

        print var.__dict__
        print var

        self.move()
        # from awesome_tool.mvc.views.gap.state import StateView, NameView
        # if isinstance(self.child, StateView) and self.child.hovered:
        #     self.move()
        # elif isinstance(self.child, NameView):
        #     self.move()

    def move(self):
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

    def __init__(self, line, parent_width):
        super(EqualDistributionConstraint, self).__init__(line[0][0], line[0][1], line[1][0], line[1][1])
        self._line = line
        self._points = []
        self._parent_width = parent_width

    def add_point(self, p, sort=None):
        if sort is None:
            sort = len(self._points)
        self._points.append((p, sort))
        self._variables.append(p[0])
        self._variables.append(p[1])
        self.create_weakest_list()
        self._sort_points()

    def add_outcome_points(self, handle_p, port_p, sort=None):
        if sort is None:
            sort = len(self._points)
        self._points.append((handle_p, port_p, sort))
        self._variables.append(handle_p[0])
        self._variables.append(handle_p[1])
        self._variables.append(port_p[0])
        self._variables.append(port_p[1])
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

    def remove_outcome_points(self, handle_p):
        try:
            self._variables.remove(handle_p[0])
            self._variables.remove(handle_p[1])
            self.create_weakest_list()
            for point_entry in self._points:
                if point_entry[0] is handle_p:
                    self._variables.remove(point_entry[1][0])
                    self._variables.remove(point_entry[1][1])
                    self.create_weakest_list()
                    break
            self._points.remove(point_entry)
        except ValueError:
            logger.error("Cannot remove point '{0} of constraint".format(handle_p))

    def solve_for(self, var=None):
        from math import atan2, sin, cos
        alpha = atan2(self._line[1][1] - self._line[0][1], self._line[1][0] - self._line[0][0])
        length = distance_point_point(self._line[0], self._line[1])
        ds = length / (len(self._points) + 1)
        dx = ds * cos(alpha)
        dy = ds * sin(alpha)

        for index, p in enumerate(self._points):
            if len(p) == 2:
                pos = p[0]
                pos.x = self._line[0][0] + (index + 1) * dx - min(5, self._parent_width * 0.05)
                pos.y = self._line[0][1] + (index + 1) * dy
            elif len(p) == 3:
                h_pos = p[0]
                p_pos = p[1]
                h_pos.x = self._line[0][0] + (index + 1) * dx - min(5, self._parent_width * 0.05)
                h_pos.y = self._line[0][1] + (index + 1) * dy
                p_pos.x = self._line[0][0] + (index + 1) * dx - min(5, self._parent_width * 0.05) - length / 20.
                p_pos.y = self._line[0][1] + (index + 1) * dy


class PortRectConstraint(Constraint):

    def __init__(self, rect, point, port):
        super(PortRectConstraint, self).__init__(rect[0][0], rect[0][1], rect[1][0], rect[1][1], point[0], point[1])

        self._rect = rect
        self._point = point
        self._initial_pos = deepcopy(point)
        self._port = port

        self._distance_to_border = self._port.port_side_size / 2.
        self.update_initial_pos()

    def update_initial_pos(self):
        if self._port.side == SnappedSide.LEFT:
            _update(self._initial_pos.x, self._initial_pos.x + self._distance_to_border)
        elif self._port.side == SnappedSide.TOP:
            _update(self._initial_pos.y, self._initial_pos.y + self._distance_to_border)
        elif self._port.side == SnappedSide.RIGHT:
            _update(self._initial_pos.x, self._initial_pos.x - self._distance_to_border)
        elif self._port.side == SnappedSide.BOTTOM:
            _update(self._initial_pos.y, self._initial_pos.y - self._distance_to_border)

    def solve_for(self, var=None):
        self._solve()

    def _solve(self):
        self.update_distance_to_border()
        px, py = self._point
        nw_x, nw_y, se_x, se_y = self.get_adjusted_border_positions()

        if ((self._initial_pos.x == nw_x and self._initial_pos.y == nw_y) or
                (self._initial_pos.x == se_x and self._initial_pos.y == nw_y) or
                (self._initial_pos.x == se_x and self._initial_pos.y == se_y) or
                (self._initial_pos.x == nw_x and self._initial_pos.y == se_y)):
            self.set_pos(px, se_x, nw_x)
            self.set_pos(py, se_y, nw_y)
        elif self._initial_pos.x == nw_x:
            _update(px, nw_x)
            self.set_pos(py, se_y, nw_y)
            self._port.side = SnappedSide.LEFT
        elif self._initial_pos.y == nw_y:
            _update(py, nw_y)
            self.set_pos(px, se_x, nw_x)
            self._port.side = SnappedSide.TOP
        elif self._initial_pos.x == se_x:
            _update(px, se_x)
            self.set_pos(py, se_y, nw_y)
            self._port.side = SnappedSide.RIGHT
        elif self._initial_pos.y == se_y:
            _update(py, se_y)
            self.set_pos(px, se_x, nw_x)
            self._port.side = SnappedSide.BOTTOM
        else:
            self.set_nearest_border(px, py)

        _update(self._initial_pos.x, deepcopy(px.value))
        _update(self._initial_pos.y, deepcopy(py.value))

    def update_distance_to_border(self):
        self._distance_to_border = self._port.port_side_size / 2.

    @staticmethod
    def set_pos(p, se_pos, nw_pos):
        if p > se_pos:
            _update(p, se_pos)
        elif p < nw_pos:
            _update(p, nw_pos)

    def get_adjusted_border_positions(self):
        nw_x, nw_y = self._rect[0]
        se_x, se_y = self._rect[1]

        nw_x += self._distance_to_border
        nw_y += self._distance_to_border
        se_x -= self._distance_to_border
        se_y -= self._distance_to_border

        return nw_x, nw_y, se_x, se_y

    def set_nearest_border(self, px, py):
        nw_x, nw_y, se_x, se_y = self.get_adjusted_border_positions()

        if self._port.side == SnappedSide.RIGHT:
            _update(px, se_x)
        elif self._port.side == SnappedSide.BOTTOM:
            _update(py, se_y)
        elif self._port.side == SnappedSide.LEFT:
            _update(px, nw_x)
        elif self._port.side == SnappedSide.TOP:
            _update(py, nw_y)