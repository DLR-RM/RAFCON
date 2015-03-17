import gtk
import copy

from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
import awesome_tool.statemachine.singleton
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.state_machine import Clipboard, ClipboardType
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
        self.shortcut_manager = None

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
        self.shortcut_manager = shortcut_manager
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)


    ######################################################
    # menu bar functionality - File
    ######################################################
    def on_new_activate(self, widget, data=None):
        logger.debug("Creating new statemachine ...")
        root_state = HierarchyState("new_root_state")
        sm = StateMachine(root_state)
        awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(sm)
        awesome_tool.statemachine.singleton.global_storage.mark_dirty(sm.state_machine_id)

    def on_open_activate(self, widget, data=None):
        dialog = gtk.FileChooserDialog("Please choose a folder",
                               None,
                               gtk.FILE_CHOOSER_ACTION_SELECT_FOLDER,
                               (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                gtk.STOCK_OPEN, gtk.RESPONSE_OK))

        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            logger.debug("Folder selected: " + dialog.get_filename())
        elif response == gtk.RESPONSE_CANCEL:
            logger.debug("No folder selected")
            dialog.destroy()
            return
        load_path = dialog.get_filename()
        dialog.destroy()

        [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
            global_storage.load_statemachine_from_yaml(load_path)
        awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    def on_save_activate(self, widget, data=None):
        save_path = self.model.get_selected_state_machine_model().state_machine.base_path
        logger.debug("Saving state machine in %s" % save_path)
        if save_path is None:
            self.on_save_as_activate(widget, data=None)
        else:
            awesome_tool.statemachine.singleton.global_storage.save_statemachine_as_yaml(
                self.model.get_selected_state_machine_model().state_machine,
                self.model.get_selected_state_machine_model().state_machine.base_path,
                delete_old_state_machine=False)

        self.model.get_selected_state_machine_model().root_state.store_meta_data_for_state()
        logger.debug("Successfully saved graphics meta data.")

    def on_save_as_activate(self, widget, data=None):

        dialog = gtk.FileChooserDialog("Please choose a file",
                                       None,
                                       gtk.FILE_CHOOSER_ACTION_CREATE_FOLDER,
                                       (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                        gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            logger.debug("File selected: " + dialog.get_filename())
        elif response == gtk.RESPONSE_CANCEL:
            logger.debug("No file selected")
            dialog.destroy()
            return
        self.model.get_selected_state_machine_model().state_machine.base_path = dialog.get_filename()
        dialog.destroy()
        self.on_save_activate(widget, data)

    def on_menu_properties_activate(self, widget, data=None):
        # TODO: implement
        pass

    def on_quit_activate(self, widget, data=None):
        self.on_main_window_destroy(None, None)

    ######################################################
    # menu bar functionality - Edit
    ######################################################

    def on_copy_selection_activate(self, widget, data=None):

        self.shortcut_manager.trigger_action("copy", None, None, force=True)

    def on_paste_clipboard_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("paste", None, None, force=True)

    def on_cut_selection_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("cut", None, None, force=True)

    def on_delete_state_activate(self, widget, data=None):
        logger.debug("Delete selected state now ...")
        selection = self.child_controllers['state_machines_editor_ctrl'].model.state_machines.values()[0].selection
        selected_state_model = selection.get_selected_state()

        if selected_state_model and selected_state_model.parent is not None:
            selected_state_model.parent.state.remove_state(selected_state_model.state.state_id)
            selection.remove(selected_state_model)

    def on_add_state_activate(self, widget, method=None, *arg):
        # TODO: use method from helper class
        pass

    def on_delete_activate(self, widget, data=None):
        logger.debug("Delete something that is selected now ...")
        #logger.debug("focus is here: %s" % self.view['main_window'].get_focus())

        selection = self.child_controllers['state_machines_editor_ctrl'].model.state_machines.values()[0].selection
        if selection.get_selected_state() and selection.get_selected_state().parent is not None:
            selected_state_model = selection.get_selected_state()
            selected_state_model.parent.state.remove_state(selected_state_model.state.state_id)
            selection.remove(selected_state_model)
            logger.debug("Delete State: %s, %s" % (selected_state_model.state.state_id,
                                                   selected_state_model.state.name))
        elif len(selection) == 1 and selection.get_num_data_flows() == 1:
            data_flow = selection.get_data_flows()[0].data_flow
            selection.get_data_flows()[0].parent.state.remove_data_flow(data_flow.data_flow_id)
            selection.remove(selection.get_data_flows()[0])
            logger.debug("Delete DataFlow: from %s to %s" % (data_flow.from_state,
                                                             data_flow.to_state))
        elif len(selection) == 1 and selection.get_num_transitions() == 1:
            transition = selection.get_transitions()[0].transition
            selection.get_transitions()[0].parent.state.remove_transition(transition.transition_id)
            selection.remove(selection.get_transitions()[0])
            logger.debug("Delete Transition: from %s, %s to %s, %s" % (transition.from_state, transition.from_outcome,
                                                                       transition.to_state, transition.to_outcome))
        else:
            logger.debug("in selection is nothing deletable: %s" % selection)

    def on_redo_activate(self, widget, data=None):
        pass

    def on_undo_activate(self, widget, data=None):
        pass

    def on_ungroup_states_activate(self, widget, data=None):
        pass

    def on_group_states_activate(self, widget, data=None):
        pass

    def on_grid_toggled(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - View
    ######################################################
    def on_expert_view_activate(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - Execution
    ######################################################
    def on_start_activate(self, widget, data=None):
        logger.debug("Start execution engine ...")
        awesome_tool.statemachine.singleton.state_machine_execution_engine.start(self.model.selected_state_machine_id)

    def on_pause_activate(self, widget, data=None):
        logger.debug("Pause execution engine ...")
        awesome_tool.statemachine.singleton.state_machine_execution_engine.pause()

    def on_stop_activate(self, widget, data=None):
        logger.debug("Stop execution engine ...")
        awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    def on_step_mode_activate(self, widget, data=None):
        logger.debug("Activate execution engine step mode ...")
        awesome_tool.statemachine.singleton.state_machine_execution_engine.step_mode()

    def on_step_activate(self, widget, data=None):
        logger.debug("Execution step ...")
        awesome_tool.statemachine.singleton.state_machine_execution_engine.step()

    def on_backward_step_mode_activate(self, widget, data=None):
        logger.debug("Backward execution step not implemented yet!")
        pass

    ######################################################
    # menu bar functionality - Help
    ######################################################
    def on_about_activate(self, widget, data=None):
        pass
