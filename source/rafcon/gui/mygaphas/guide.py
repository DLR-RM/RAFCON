# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""Guided item/handle movement (alignment guides) on top of gaphas 5

gaphas removed the InMotion aspects; movement is now modelled by small "Move" objects.
This module provides

* ``InMotion(item, view)`` — factory returning a move object with ``start_move(pos)``,
  ``move(pos)`` and ``stop_move()``; states and name views snap to sibling/parent edges
* ``GuidedStateHandleInMotion`` — handle movement with guide support, registered for
  StateView/NameView at the aspect registry
"""

from dataclasses import dataclass

from rafcon.gui.mygaphas.aspect import ItemHandleInMotion, register_handle_in_motion
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.utils import log

logger = log.get_logger(__name__)


@dataclass(frozen=True)
class Guides:
    vertical: tuple
    horizontal: tuple


def _find_closest(item_edges, edges, margin):
    delta = 0
    min_d = 1000
    closest = []
    for e in edges:
        for ie in item_edges:
            d = abs(e - ie)
            if d < min_d:
                min_d = d
                delta = e - ie
                closest = [e]
            elif d == min_d:
                closest.append(e)

    return (delta, closest) if min_d <= margin else (0, ())


def _element_vertical_edges(element):
    """Vertical edges (x values) of an element in item coordinates"""
    x = element.handles()[0].pos.x
    w = element.width
    return [x, x + w / 2, x + w]


def _element_horizontal_edges(element):
    """Horizontal edges (y values) of an element in item coordinates"""
    y = element.handles()[0].pos.y
    h = element.height
    return [y, y + h / 2, y + h]


def reset_guides(view):
    try:
        del view.guides
    except AttributeError:
        pass  # No problem if guides do not exist.
    else:
        view.update_back_buffer()


class ItemInMotion(object):
    """Aspect for dealing with motion on an item; the item is moved"""

    def __init__(self, item, view):
        self.item = item
        self.view = view
        self.last_x = None
        self.last_y = None

    def start_move(self, pos):
        self.last_x, self.last_y = pos

    def move(self, pos):
        """Move the item. x and y are in view coordinates"""
        item = self.item
        view = self.view

        v2i = view.get_matrix_v2i(item)
        x, y = pos
        dx, dy = x - self.last_x, y - self.last_y
        dx, dy = v2i.transform_distance(dx, dy)
        self.last_x, self.last_y = x, y

        item.matrix.translate(dx, dy)
        view.canvas.request_matrix_update(item)

    def stop_move(self):
        pass


class GuidedStateMixin(object):
    """Snap moved states/names to the edges of their siblings and their parent state"""

    MARGIN = 5

    def find_vertical_guides(self, item_vedges):
        # The root state cannot be aligned
        if not self.item.parent:
            return 0, ()

        vedges = set()
        for state_v in self._get_siblings_and_parent():
            i2v = self.view.get_matrix_i2v(state_v)
            for x in _element_vertical_edges(state_v):
                vedges.add(i2v.transform_point(x, 0)[0])
        return _find_closest(item_vedges, vedges, self.MARGIN)

    def find_horizontal_guides(self, item_hedges):
        # The root state cannot be aligned
        if not self.item.parent:
            return 0, ()

        hedges = set()
        for state_v in self._get_siblings_and_parent():
            i2v = self.view.get_matrix_i2v(state_v)
            for y in _element_horizontal_edges(state_v):
                hedges.add(i2v.transform_point(0, y)[1])
        return _find_closest(item_hedges, hedges, self.MARGIN)

    def _get_siblings_and_parent(self):
        states_v = []
        parent_state_v = self.item.parent
        states_v.append(parent_state_v)
        for sibling in self.view.canvas.get_children(parent_state_v):
            if isinstance(sibling, StateView) and sibling is not self.item:
                states_v.append(sibling)
        return states_v

    def update_guides(self, item_vedges, item_hedges):
        dx, edges_x = self.find_vertical_guides(item_vedges)
        dy, edges_y = self.find_horizontal_guides(item_hedges)
        self.view.guides = Guides(tuple(edges_x), tuple(edges_y))
        self.view.update_back_buffer()
        return dx, dy


class GuidedStateInMotion(GuidedStateMixin, ItemInMotion):

    def start_move(self, pos):
        if self.item and self.item.model and self.item.model.state.is_root_state:
            return
        super(GuidedStateInMotion, self).start_move(pos)
        self.item.moving = True

    def move(self, pos):
        if not self.item.moving:
            return

        px, py = pos
        pdx, pdy = px - self.last_x, py - self.last_y
        transform = self.view.get_matrix_i2v(self.item).transform_point
        item_vedges = [transform(x, 0)[0] + pdx for x in _element_vertical_edges(self.item)]
        item_hedges = [transform(0, y)[1] + pdy for y in _element_horizontal_edges(self.item)]
        dx, dy = self.update_guides(item_vedges, item_hedges)

        super(GuidedStateInMotion, self).move((pos[0] + dx, pos[1] + dy))

        parent_item = self.item.parent
        if parent_item:  # e.g. parent_item=root state if a state is moved inside the root state
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)

    def stop_move(self):
        super(GuidedStateInMotion, self).stop_move()
        self.item.moving = False
        reset_guides(self.view)


class GuidedNameInMotion(ItemInMotion):
    def move(self, pos):
        super(GuidedNameInMotion, self).move(pos)
        parent_item = self.item.parent
        if parent_item:
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)

    def stop_move(self):
        super(GuidedNameInMotion, self).stop_move()
        reset_guides(self.view)


def InMotion(item, view):
    """Create the motion object suitable for the type of ``item``"""
    if isinstance(item, StateView):
        return GuidedStateInMotion(item, view)
    if isinstance(item, NameView):
        return GuidedNameInMotion(item, view)
    return ItemInMotion(item, view)


@register_handle_in_motion(StateView, NameView)
class GuidedStateHandleInMotion(GuidedStateMixin, ItemHandleInMotion):
    """Handle motion (resizing, port movement) for states and name views"""

    def move(self, pos):
        item = self.item
        if isinstance(item, StateView):
            for port in item.get_all_ports():
                if port.handle is self.handle:
                    # Ports are not guided and glue directly
                    self.GLUE_DISTANCE = 0
                    return super(GuidedStateHandleInMotion, self).move(pos)

        sink = super(GuidedStateHandleInMotion, self).move(pos)

        x, y = pos
        dx, dy = self.update_guides((x,), (y,))
        if dx or dy:
            v2i = self.view.get_matrix_v2i(item)
            self.handle.pos = v2i.transform_point(x + dx, y + dy)
            self.view.canvas.request_update(item, matrix=False)
        return sink

    def stop_move(self):
        super(GuidedStateHandleInMotion, self).stop_move()
        reset_guides(self.view)
