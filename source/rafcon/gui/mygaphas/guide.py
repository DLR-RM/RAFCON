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

from gaphas.aspect import InMotion, HandleInMotion
from gaphas.guide import GuidedItemInMotion, GuidedItemHandleInMotion, GuideMixin

from rafcon.gui.mygaphas.items.state import StateView, NameView


class GuidedStateMixin(GuideMixin):
    pass


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
