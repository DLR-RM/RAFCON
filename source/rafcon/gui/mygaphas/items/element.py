# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html

"""Element base class for RAFCON canvas items on top of gaphas 5

gaphas 5 turned items into plain model objects without a canvas backref and
without an internal constraint list (constraints are registered at the
Connections object of the model). RAFCON's items rely on both, so this module
provides an Element implementation that mirrors gaphas.item.Element but

* takes the canvas at construction time and exposes it as ``item.canvas``
* keeps track of all item constraints in ``item.constraints``
* provides the gaphas 2.x ``setup_canvas()`` hook, invoked by MyCanvas.add()
"""

from gaphas.constraint import constraint
from gaphas.handle import Handle
from gaphas.item import Matrices, NW, NE, SE, SW
from gaphas.geometry import distance_rectangle_border_point
from gaphas.port import LinePort
from gaphas.solver import REQUIRED, VERY_STRONG, variable


class ElementItem(Matrices):
    """A rectangular item with four handles (NW, NE, SE, SW)"""

    min_width = variable(strength=REQUIRED, varname="_min_width")
    min_height = variable(strength=REQUIRED, varname="_min_height")

    def __init__(self, canvas, width=10, height=10):
        super(ElementItem, self).__init__()
        self._canvas = canvas
        self._constraints = []

        self._handles = [Handle(strength=VERY_STRONG) for _ in range(4)]
        handles = self._handles
        h_nw = handles[NW]
        h_ne = handles[NE]
        h_sw = handles[SW]
        h_se = handles[SE]

        # edges of the element define the default ports
        self._ports = [
            LinePort(h_nw.pos, h_ne.pos),
            LinePort(h_ne.pos, h_se.pos),
            LinePort(h_se.pos, h_sw.pos),
            LinePort(h_sw.pos, h_nw.pos),
        ]

        self.min_width, self.min_height = 10, 10

        add = self.add_constraint
        add(constraint(horizontal=(h_nw.pos, h_ne.pos)))
        add(constraint(horizontal=(h_sw.pos, h_se.pos)))
        add(constraint(vertical=(h_nw.pos, h_sw.pos)))
        add(constraint(vertical=(h_ne.pos, h_se.pos)))
        self._c_min_w = add(constraint(left_of=(h_nw.pos, h_se.pos), delta=self.min_width))
        self._c_min_h = add(constraint(above=(h_nw.pos, h_se.pos), delta=self.min_height))

        self.width = width
        self.height = height

        # Trigger solver to honour width/height by SE handle pos
        self._handles[SE].pos.x.dirty()
        self._handles[SE].pos.y.dirty()

    @property
    def canvas(self):
        return self._canvas

    @property
    def constraints(self):
        return self._constraints

    def add_constraint(self, c):
        """Register a constraint for this item at the canvas and track it"""
        self._canvas.connections.add_constraint(self, c)
        self._constraints.append(c)
        return c

    def remove_constraint(self, c):
        if c in self._constraints:
            self._constraints.remove(c)
        self._canvas.connections.remove_constraint(self, c)

    def remove_all_constraints(self):
        for c in self._constraints[:]:
            self.remove_constraint(c)

    def setup_canvas(self):
        """Called by MyCanvas.add() when the item was added to the canvas"""
        pass

    @property
    def width(self):
        h = self._handles
        return float(h[SE].pos.x) - float(h[NW].pos.x)

    @width.setter
    def width(self, width):
        h = self._handles
        h[SE].pos.x = h[NE].pos.x = h[NW].pos.x + width

    @property
    def height(self):
        h = self._handles
        return float(h[SE].pos.y) - float(h[NW].pos.y)

    @height.setter
    def height(self, height):
        h = self._handles
        h[SE].pos.y = h[SW].pos.y = h[NW].pos.y + height

    def handles(self):
        return self._handles

    def ports(self):
        return self._ports

    def point(self, x, y):
        """Distance from the point (x, y) to the item, in item coordinates"""
        h = self._handles
        x0, y0 = h[NW].pos
        x1, y1 = h[SE].pos
        return distance_rectangle_border_point((x0, y0, x1 - x0, y1 - y0), (x, y))

    def draw(self, context):
        pass
