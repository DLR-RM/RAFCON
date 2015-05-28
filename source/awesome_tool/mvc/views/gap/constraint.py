from awesome_tool.utils import log
logger = log.get_logger(__name__)

from gaphas.geometry import distance_point_point
from gaphas.constraint import Constraint

from copy import deepcopy, copy

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


class EqualDistributionDoublePortConstraint(Constraint):

    def __init__(self, line, parent_state):
        super(EqualDistributionDoublePortConstraint, self).__init__(line[0][0], line[0][1], line[1][0], line[1][1])
        self._line = line
        self._points = []
        self._parent_width = parent_state.width
        self._parent_state = parent_state

    def add_double_port_points(self, rh_pos, rp_pos, lh_pos, lp_pos, sort=None):
        if sort is None:
            sort = len(self._points)
        self._points.append((rh_pos, rp_pos, lh_pos, lp_pos, sort))
        self._variables.append(rh_pos[0])
        self._variables.append(rh_pos[1])
        self._variables.append(rp_pos[0])
        self._variables.append(rp_pos[1])
        self._variables.append(lh_pos[0])
        self._variables.append(lh_pos[1])
        self._variables.append(lp_pos[0])
        self._variables.append(lp_pos[1])
        self.create_weakest_list()
        self._sort_points()

    def _sort_points(self):
        self._points.sort(lambda p1, p2: cmp(p1[4], p2[4]))

    def remove_outcome_points(self, rh_pos):
        try:
            self._variables.remove(rh_pos[0])
            self._variables.remove(rh_pos[1])
            self.create_weakest_list()
            for point_entry in self._points:
                if point_entry[0] is rh_pos:
                    self._variables.remove(point_entry[1][0])
                    self._variables.remove(point_entry[1][1])
                    self._variables.remove(point_entry[2][0])
                    self._variables.remove(point_entry[2][1])
                    self._variables.remove(point_entry[3][0])
                    self._variables.remove(point_entry[3][1])
                    self.create_weakest_list()
                    break
            self._points.remove(point_entry)
        except ValueError:
            logger.error("Cannot remove point '{0} of constraint".format(rh_pos))

    def solve_for(self, var=None):
        from math import atan2, sin, cos
        alpha = atan2(self._line[1][1] - self._line[0][1], self._line[1][0] - self._line[0][0])
        length = distance_point_point(self._line[0], self._line[1])
        ds = length / (len(self._points) + 1)
        dx = ds * cos(alpha)
        dy = ds * sin(alpha)

        min_state_side = min(self._parent_state.width, self._parent_state.height)
        port_side_size = min_state_side / 20.

        for index, p in enumerate(self._points):
            rh_pos = p[0]
            rp_pos = p[1]
            lh_pos = p[2]
            lp_pos = p[3]

            rh_pos.x = self._line[0][0] - port_side_size / 2
            rh_pos.y = self._line[0][1] + port_side_size / 2 + index * port_side_size
            rp_pos.x = self._line[0][0]
            rp_pos.y = self._line[0][1] + port_side_size / 2 + index * port_side_size

            lh_pos.x = self._line[0][0] - port_side_size / 2 - self._parent_width * 0.1
            lh_pos.y = self._line[0][1] + port_side_size / 2 + index * port_side_size
            lp_pos.x = self._line[0][0] - port_side_size - self._parent_width * 0.1
            lp_pos.y = self._line[0][1] + port_side_size / 2 + index * port_side_size


class TopBottomConstraint(Constraint):

    def __init__(self, line, parent_state, top=True):
        super(TopBottomConstraint, self).__init__(line[0][0], line[0][1], line[1][0], line[1][1])
        self._line = line
        self._points = []
        self._parent_state = parent_state
        self._top = top

    def add_port_points(self, handle_pos, right_port_pos, left_port_pos, sort=None):
        if sort is None:
            sort = len(self._points)
        self._points.append((handle_pos, right_port_pos, left_port_pos, sort))
        self._variables.append(handle_pos[0])
        self._variables.append(handle_pos[1])
        self._variables.append(right_port_pos[0])
        self._variables.append(right_port_pos[1])
        self._variables.append(left_port_pos[0])
        self._variables.append(left_port_pos[1])
        self.create_weakest_list()
        self._sort_points()

    def _sort_points(self):
        self._points.sort(lambda p1, p2: cmp(p1[3], p2[3]))

    def remove_outcome_points(self, handle_pos):
        try:
            self._variables.remove(handle_pos[0])
            self._variables.remove(handle_pos[1])
            self.create_weakest_list()
            for point_entry in self._points:
                if point_entry[0] is handle_pos:
                    self._variables.remove(point_entry[1][0])
                    self._variables.remove(point_entry[1][1])
                    self._variables.remove(point_entry[2][0])
                    self._variables.remove(point_entry[2][1])
                    self.create_weakest_list()
                    break
            self._points.remove(point_entry)
        except ValueError:
            logger.error("Cannot remove point '{0} of constraint".format(handle_pos))

    def solve_for(self, var=None):
        port_side_size = self._parent_state.height / 20.

        for index, p in enumerate(self._points):
            handle_pos = p[0]
            right_port_pos = p[1]
            left_port_pos = p[2]

            if self._top:
                anchor = self._line[0]
                common_y_pos = anchor[1] + port_side_size / 2. + index * port_side_size
            else:
                anchor = self._line[1]
                common_y_pos = anchor[1] - port_side_size / 2. - (len(self._points) - index) * port_side_size

            left_port_pos.x = anchor[0] - port_side_size
            left_port_pos.y = common_y_pos
            handle_pos.x = anchor[0] - port_side_size / 2.
            handle_pos.y = common_y_pos
            right_port_pos.x = anchor[0]
            right_port_pos.y = common_y_pos


class RectConstraint(Constraint):

    def __init__(self, rect, point):
        super(RectConstraint, self).__init__(rect[0][0], rect[0][1], rect[1][0], rect[1][1], point[0], point[1])

        self._rect = rect
        self._point = point
        self._initial_pos = deepcopy(point)

    def solve_for(self, var=None):
        self._solve()

    def _solve(self):
        px, py = self._point
        nw_x, nw_y = self._rect[0]
        se_x, se_y = self._rect[1]

        if ((self._initial_pos.x == nw_x and self._initial_pos.y == nw_y) or
                (self._initial_pos.x == se_x and self._initial_pos.y == nw_y) or
                (self._initial_pos.x == se_x and self._initial_pos.y == se_y) or
                (self._initial_pos.x == nw_x and self._initial_pos.y == se_y)):
            self.set_pos(px, se_x, nw_x)
            self.set_pos(py, se_y, nw_y)
        elif self._initial_pos.x == nw_x:
            _update(px, nw_x)
            self.set_pos(py, se_y, nw_y)
        elif self._initial_pos.y == nw_y:
            _update(py, nw_y)
            self.set_pos(px, se_x, nw_x)
        elif self._initial_pos.x == se_x:
            _update(px, se_x)
            self.set_pos(py, se_y, nw_y)
        elif self._initial_pos.y == se_y:
            _update(py, se_y)
            self.set_pos(px, se_x, nw_x)

        _update(self._initial_pos.x, deepcopy(px.value))
        _update(self._initial_pos.y, deepcopy(py.value))

    @staticmethod
    def set_pos(p, se_pos, nw_pos):
        if p > se_pos:
            _update(p, se_pos)
        elif p < nw_pos:
            _update(p, nw_pos)