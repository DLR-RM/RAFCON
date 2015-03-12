import gtk
import copy

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.state_machine import StateMachine, Clipboard, ClipboardType
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.mvc.models.container_state import ContainerStateModel
from awesome_tool.statemachine.states.state_helper import StateHelper


class MenuBarController(ExtendedController):
    """
    The class to trigger all the action, available in the menu bar.
    """
    def __init__(self, state_machine_manager_model, view, state_machines_editor_ctrl):
        ExtendedController.__init__(self, state_machine_manager_model, view)
        self.clipboard = Clipboard()
        self.state_machines_editor_ctrl = state_machines_editor_ctrl

    def register_view(self, view):
        """Called when the View was registered
        """
        pass

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("copy", self._copy_selection)
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)
        shortcut_manager.add_callback_for_action("cut", self._cut_selection)

    def _copy_selection(self, *args):
        logger.debug("copy selection")
        self.clipboard.state_machine_id = copy.copy(self.model.selected_state_machine_id)
        self.clipboard.selection.set(self.model.get_selected_state_machine_model().selection)
        self.clipboard.clipboard_type = ClipboardType.COPY

    def _paste_clipboard(self, *args):
        logger.debug("paste selection")
        currently_selected_sm_id = self.model.selected_state_machine_id
        current_selection = self.model.get_selected_state_machine_model().selection
        # check if the current selection is valid
        if current_selection.get_number_of_selected_items() > 1 or len(current_selection.get_states()) < 1:
            logger.error("Cannot paste clipboard into selection as the selection does not consist of a single"
                         "container state!")
            return
        if len(current_selection.get_states()) == 1 and\
                not isinstance(current_selection.get_states()[0], ContainerStateModel):
                logger.error("Cannot paste clipboard into selection as the selected state model is not "
                             "a container state model")
                return

        # check if the clipboard is valid
        if self.clipboard.selection.get_number_of_selected_items() > 1:
            logger.error("Only one single item is allowed to be copied yet!")
            return
        if not len(self.clipboard.selection.get_states()) == 1:
            logger.error("Only states are allowed to be copied yet!")
            return

        source_state_model = self.clipboard.selection.get_states()[0]
        source_state = source_state_model.state
        target_state_model = current_selection.get_states()[0]
        target_state = target_state_model.state
        state_copy = StateHelper.get_state_copy(source_state)
        target_state.add_state(state_copy)
        state_copy_model = target_state_model.states[state_copy.state_id]

        state_copy_model.copy_meta_data_from_state_model(source_state_model)
        new_x_pos = target_state_model.meta["gui"]["editor"]["pos_x"] + \
                    target_state_model.meta["gui"]["editor"]["width"] * 3 / 100
        new_y_pos = target_state_model.meta["gui"]["editor"]["pos_y"] + \
                    target_state_model.meta["gui"]["editor"]["height"] * 97 / 100 - \
                    state_copy_model.meta["gui"]["editor"]["height"]

        self.state_machines_editor_ctrl.tabs[self.model.selected_state_machine_id]["ctrl"]._move_state(
            state_copy_model, new_x_pos, new_y_pos )
        self.state_machines_editor_ctrl.tabs[self.model.selected_state_machine_id]["ctrl"]._redraw(True)

        if self.clipboard.clipboard_type is ClipboardType.COPY:
            # logger.debug("Copy the following clipboard into the selected state %s: \n%s"
            #              % (str(current_selection.get_states()[0]),
            #                 str(self.clipboard)))
            pass
        elif self.clipboard.clipboard_type is ClipboardType.CUT:
            # logger.debug("Cut the following clipboard into the selected state %s: \n%s"
            #              % (str(current_selection.get_states()[0]),
            #                 str(self.clipboard)))
            parent_of_source_state = source_state.parent
            parent_of_source_state.remove_state(source_state.state_id)


    def _cut_selection(self, *args):
        logger.debug("cut selection")
        self.clipboard.state_machine_id = copy.copy(self.model.selected_state_machine_id)
        self.clipboard.selection.set(self.model.get_selected_state_machine_model().selection)
        self.clipboard.clipboard_type = ClipboardType.CUT