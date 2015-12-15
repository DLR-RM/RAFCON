from gaphas.aspect import InMotion
from gaphas.guide import GuidedItemInMotion

from rafcon.mvc.mygaphas.items.state import StateView


@InMotion.when_type(StateView)
class GuidedStateInMotion(GuidedItemInMotion):

    def get_excluded_items(self):

        parent_state = self.view.canvas.get_parent(self.item)
        siblings = self.view.canvas.get_children(parent_state)
        siblings = set([sibling for sibling in siblings if isinstance(sibling, StateView) and sibling is not self.item])
        all_items = set(self.view.canvas.get_all_items())
        exclude_items = all_items - siblings

        return exclude_items
