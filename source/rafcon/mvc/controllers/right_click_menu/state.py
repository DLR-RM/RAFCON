"""
.. module:: state_machine_right_click_menu
   :platform: Unix, Windows
   :synopsis: The module provides a menu by right click to edit states or its elements.

.. moduleauthor:: Rico Belder


"""

import gtk

from rafcon.mvc.utils import constants
from rafcon.mvc.gui_helper import set_label_markup
from rafcon.mvc.clipboard import global_clipboard
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController

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

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_COPY)
        # label = gtk.Label("Copy selection")
        # set_label_markup(label, '&#x' + constants.BUTTON_COPY + ';',
        #                  font=constants.ICON_FONT, font_size=constants.FONT_SIZE_BIG)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_COPY)
        menu_item.set_label("Copy selection")
        menu_item.connect("activate", self.on_copy_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_PASTE)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_PASTE)
        menu_item.set_label("Paste Selection")
        menu_item.connect("activate", self.on_paste_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_CUT)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_CUT)
        menu_item.set_label("Cut selection")
        menu_item.connect("activate", self.on_cut_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_GROUP)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_LEAVE_FULLSCREEN)
        menu_item.set_label("Group states")
        menu_item.connect("activate", self.on_group_states_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_UNGR)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_FULLSCREEN)
        menu_item.set_label("Ungroup state")
        menu_item.connect("activate", self.on_ungroup_state_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_GROUP)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_GO_DOWN)
        menu_item.set_label("Run from here")
        menu_item.connect("activate", self.on_run_from_selected_state_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_UNGR)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_GO_UP)
        menu_item.set_label("Stop here")
        menu_item.connect("activate", self.on_run_to_selected_state_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

        # menu_item = gtk.ImageMenuItem(constants.BUTTON_UNGR)
        menu_item = gtk.ImageMenuItem(gtk.STOCK_GO_FORWARD)
        menu_item.set_label("Save state as state machine")
        menu_item.connect("activate", self.on_save_state_as_state_machine_activate)
        menu_item.set_always_show_image(True)
        menu.append(menu_item)

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
        logger.info("do save state as state machine")
        self.shortcut_manager.trigger_action('save_state_as', None, None)

    def mouse_click(self, widget, event=None):

        # Single right click
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            menu = self.generate_right_click_menu()
            menu.show_all()
            return self.activate_menu(event, menu)

    def activate_menu(self, event, menu):
        logger.info("activate_menu by " + self.__class__.__name__)
        menu.popup(None, None, None, event.button, event.time)
        return True


class StateRightClickMenuGapahs(StateMachineRightClickMenu):

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger gaphas copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger gaphas paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger gaphas cut")
        self.shortcut_manager.trigger_action("cut", None, None)


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
        logger.info("activate_menu by " + self.__class__.__name__)
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
        menu.popup(None, None, None, event.button, event.time)
        return True

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger opengl copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger opengl paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger opengl cut")
        self.shortcut_manager.trigger_action("cut", None, None)
