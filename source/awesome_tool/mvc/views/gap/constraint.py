from awesome_tool.utils import log
logger = log.get_logger(__name__)

from gaphas.geometry import distance_point_point
from gaphas.constraint import Constraint


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

