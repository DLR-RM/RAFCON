from gaphas.aspect import InMotion
from gaphas.guide import GuidedItemInMotion

from rafcon.mvc.mygaphas.items.state import StateView, NameView


@InMotion.when_type(StateView)
class GuidedStateInMotion(GuidedItemInMotion):

    def get_excluded_items(self):

        parent_state = self.view.canvas.get_parent(self.item)
        siblings = self.view.canvas.get_children(parent_state)
        siblings = set([sibling for sibling in siblings if isinstance(sibling, StateView) and sibling is not self.item])
        all_items = set(self.view.canvas.get_all_items())
        exclude_items = all_items - siblings

        return exclude_items

    def start_move(self, pos):
        super(GuidedStateInMotion, self).start_move(pos)
        self.item.moving = True

    def move(self, pos):
        super(GuidedStateInMotion, self).move(pos)
        parent_item = self.item.parent
        if parent_item:
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)

    def stop_move(self):
        super(GuidedStateInMotion, self).stop_move()
        self.item.moving = False


@InMotion.when_type(NameView)
class GuidedStateInMotion(GuidedItemInMotion):
    def move(self, pos):
        super(GuidedStateInMotion, self).move(pos)
        parent_item = self.item.parent
        if parent_item:
            constraint = parent_item.keep_rect_constraints[self.item]
            self.view.canvas.solver.request_resolve_constraint(constraint)
