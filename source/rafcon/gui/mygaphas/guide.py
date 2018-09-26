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

from past.builtins import map
from gaphas.aspect import InMotion, HandleInMotion
from gaphas.guide import GuidedItemInMotion, GuidedItemHandleInMotion, Guide, GuideMixin

from rafcon.gui.mygaphas.items.state import StateView, NameView


class GuidedStateMixin(GuideMixin):

    MARGIN = 5

    def get_excluded_items(self):
        return set()

    def find_vertical_guides(self, item_vedges, pdx, height, excluded_items):
        # The root state cannot be aligned
        if not self.item.parent:
            return 0, ()

        states_v = self._get_siblings_and_parent()

        try:
            guides = list(map(Guide, states_v))
        except TypeError:
            guides = []

        vedges = set()
        for g in guides:
            for x in g.vertical():
                vedges.add(self.view.get_matrix_i2v(g.item).transform_point(x, 0)[0])
        dx, edges_x = self.find_closest(item_vedges, vedges)

        return dx, edges_x

    def find_horizontal_guides(self, item_hedges, pdy, width, excluded_items):
        # The root state cannot be aligned
        if not self.item.parent:
            return 0, ()

        states_v = self._get_siblings_and_parent()

        try:
            guides = list(map(Guide, states_v))
        except TypeError:
            guides = []

        hedges = set()
        for g in guides:
            for y in g.horizontal():
                hedges.add(self.view.get_matrix_i2v(g.item).transform_point(0, y)[1])

        dy, edges_y = self.find_closest(item_hedges, hedges)
        return dy, edges_y

    def _get_siblings_and_parent(self):
        states_v = []
        parent_state_v = self.item.parent
        states_v.append(parent_state_v)
        for sibling in self.view.canvas.get_children(parent_state_v):
            if isinstance(sibling, StateView) and sibling is not self.item:
                states_v.append(sibling)
        return states_v


@InMotion.when_type(StateView)
class GuidedStateInMotion(GuidedStateMixin, GuidedItemInMotion):

    def start_move(self, pos):
        if self.item and self.item.model and self.item.model.state.is_root_state:
            return
        super(GuidedStateInMotion, self).start_move(pos)
        self.item.moving = True

    def move(self, pos):
        if not self.item.moving:
            return
        super(GuidedStateInMotion, self).move(pos)
        parent_item = self.item.parent
        if parent_item:
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)

    def stop_move(self):
        super(GuidedStateInMotion, self).stop_move()
        self.item.moving = False


@InMotion.when_type(NameView)
class GuidedNameInMotion(GuidedItemInMotion):
    def move(self, pos):
        super(GuidedNameInMotion, self).move(pos)
        parent_item = self.item.parent
        if parent_item:
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)


@HandleInMotion.when_type(StateView)
class GuidedStateHandleInMotion(GuidedStateMixin, GuidedItemHandleInMotion):
    
    def glue(self, pos, distance=None):
        distance = distance if distance else self.GLUE_DISTANCE
        super(GuidedStateHandleInMotion, self).glue(pos, distance)

    def move(self, pos):
        ports = self.item.get_all_ports()
        for port in ports:
            if port.handle is self.handle:
                self.GLUE_DISTANCE = 0
                return super(GuidedItemHandleInMotion, self).move(pos)
        super(GuidedStateHandleInMotion, self).move(pos)
