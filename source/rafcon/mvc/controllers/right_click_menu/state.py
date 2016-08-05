"""
.. module:: state_machine_right_click_menu
   :platform: Unix, Windows
   :synopsis: The module provides a menu by right click to edit states or its elements.

.. moduleauthor:: Rico Belder


"""

import gtk

from rafcon.mvc.utils import constants
from rafcon.mvc.gui_helper import create_image_menu_item
from rafcon.mvc.clipboard import global_clipboard
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
import rafcon.mvc.singleton as mvc_singleton

from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineRightClickMenu:

    def __init__(self, state_machine_manager_model=None):
        if state_machine_manager_model is None:
            from rafcon.mvc.singleton import state_machine_manager_model
        self.state_machine_manager_model = state_machine_manager_model
        from rafcon.mvc.singleton import main_window_controller
        shortcut_manager = main_window_controller.shortcut_manager
        self.shortcut_manager = shortcut_manager

    def generate_right_click_menu(self):
        menu = gtk.Menu()

        menu.append(create_image_menu_item("Copy selection", constants.BUTTON_COPY, self.on_copy_activate))
        menu.append(create_image_menu_item("Paste selection", constants.BUTTON_PASTE, self.on_paste_activate))
        menu.append(create_image_menu_item("Cut selection", constants.BUTTON_CUT, self.on_cut_activate))
        menu.append(create_image_menu_item("Group states", constants.BUTTON_GROUP, self.on_group_states_activate))
        menu.append(create_image_menu_item("Ungroup states", constants.BUTTON_UNGR, self.on_ungroup_state_activate))
        menu.append(create_image_menu_item("Run from here", constants.BUTTON_START_FROM_SELECTED_STATE,
                                           self.on_run_from_selected_state_activate))
        menu.append(create_image_menu_item("Stop here", constants.BUTTON_RUN_TO_SELECTED_STATE,
                                           self.on_run_to_selected_state_activate))
        menu.append(create_image_menu_item("Save state as state machine", constants.BUTTON_SAVE,
                                           self.on_save_state_as_state_machine_activate))
        menu.append(create_image_menu_item("Substitute state", constants.BUTTON_REFR,
                                           self.on_substitute_state_activate))

        return menu

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger default copy")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.copy(active_sm_m.selection)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger default paste")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.paste(active_sm_m.selection.get_states()[0])

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger default cut")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.cut(active_sm_m.selection)

    def on_group_states_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("group", None, None)

    def on_ungroup_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("ungroup", None, None)

    def on_save_as_state_machine_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("save_as_sm", None, None)

    def on_run_from_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('start_from_selected', None, None)

    def on_run_to_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('run_to_selected', None, None)

    def on_save_state_as_state_machine_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('save_state_as', None, None)

    def on_substitute_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('substitute_state', None, None)

    def mouse_click(self, widget, event=None):

        # Single right click
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            menu = self.generate_right_click_menu()
            menu.show_all()
            return self.activate_menu(event, menu)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        menu.popup(None, None, None, event.button, event.time)
        return True


class StateMachineRightClickMenuController(ExtendedController, StateMachineRightClickMenu):

    def __init__(self, model=None, view=None):
        ExtendedController.__init__(self, model, view)
        StateMachineRightClickMenu.__init__(self)

    def register_view(self, view):
        raise NotImplementedError


class StateMachineTreeRightClickMenuController(StateMachineRightClickMenuController):

    def register_view(self, view):
        view.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        pthinfo = self.view.get_path_at_pos(int(event.x), int(event.y))

        if pthinfo is not None:
            path, col, cellx, celly = pthinfo
            self.view.grab_focus()
            self.view.set_cursor(path, col, 0)

            menu.popup(None, None, None, event.button, event.time)
        return True


class StateRightClickMenuControllerOpenGLEditor(StateMachineRightClickMenuController):

    def register_view(self, view):
        from rafcon.mvc.views.graphical_editor import GraphicalEditorView
        assert isinstance(view, GraphicalEditorView)
        view.editor.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = mvc_singleton.state_machine_manager_model.get_selected_state_machine_model().selection
        if selection.get_num_states() > 0 or selection.get_num_scoped_variables() > 0:
            menu.popup(None, None, None, event.button, event.time)
            return True
        else:
            return False

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger opengl copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger opengl paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger opengl cut")
        self.shortcut_manager.trigger_action("cut", None, None)


class StateRightClickMenuGaphas(StateMachineRightClickMenu):

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = mvc_singleton.state_machine_manager_model.get_selected_state_machine_model().selection
        if selection.get_num_states() > 0 or selection.get_num_scoped_variables() > 0:
            menu.popup(None, None, None, event.button, event.time)
            return True
        else:
            return False

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger gaphas copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger gaphas paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger gaphas cut")
        self.shortcut_manager.trigger_action("cut", None, None)
