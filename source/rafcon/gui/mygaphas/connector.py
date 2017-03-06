# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from math import sqrt
from gaphas.connector import PointPort


class RectanglePointPort(PointPort):

    def __init__(self, point, port_v=None):
        """Rectangular port class

        The class inherits from PointPort. Handles connected to the port also always connected to the center of the
        port. However, the size of the port is bigger and describes a rectangle. Distances to the port are measured
        from the edges of this rectangle.

        :param point: The center point of the rectangle
        :param port_v: The port view belonging to the port
        """
        super(RectanglePointPort, self).__init__(point)
        self.port_v = port_v

    @property
    def width(self):
        if not self.port_v:
            return 1
        return self.port_v.port_size[0]

    @property
    def height(self):
        if not self.port_v:
            return 1
        return self.port_v.port_size[1]

    def glue(self, pos):
        """Calculates the distance between the given position and the port

        :param (float, float) pos: Distance to this position is calculated
        :return: Distance to port
        :rtype: float
        """
        # Distance between border of rectangle and point
        # Equation from http://stackoverflow.com/a/18157551/3568069
        dx = max(self.point.x - self.width / 2. - pos[0], 0, pos[0] - (self.point.x + self.width / 2.))
        dy = max(self.point.y - self.height / 2. - pos[1], 0, pos[1] - (self.point.y + self.height / 2.))
        dist = sqrt(dx*dx + dy*dy)
        return self.point, dist
