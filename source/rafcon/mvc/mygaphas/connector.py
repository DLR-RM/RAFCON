from math import sqrt
from gaphas.connector import PointPort


class RectanglePointPort(PointPort):

    _width = 0
    _height = 0

    def __init__(self, point, width, height):
        """Rectangular port class

        The class inherits from PointPort. Handles connected to the port also always connected to the center of the
        port. However, the size of the port is bigger and describes a rectangle. Distances to the port are measured
        from the edges of this rectangle.

        :param point: The center point of the rectangle
        :param width: The width of the rectangle
        :param height: The height of the rectangle
        """
        super(RectanglePointPort, self).__init__(point)
        self.width = width
        self.height = height

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, width):
        self._width = abs(width)

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, height):
        self._height = abs(height)

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
