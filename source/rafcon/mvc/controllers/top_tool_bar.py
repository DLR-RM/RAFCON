from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.utils import log
import gtk
logger = log.get_logger(__name__)


class TopToolBarController(ExtendedController):
    """
    The class to trigger all the action, available in the tool bar.
    """
    def __init__(self, state_machine_manager_model, view, top_level_window, menu_bar_controller):
        ExtendedController.__init__(self, state_machine_manager_model, view)
        self.shortcut_manager = None

        self.top_level_window = top_level_window
        self.fullscreen = False
        self.menu_bar_controller = menu_bar_controller

        view.get_top_widget().connect("motion_notify_event", self.motion_detected)
        view.get_top_widget().connect("button_press_event", self.button_pressed_event)


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

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """

    def on_minimize_button_clicked(self, widget, data=None):
        self.top_level_window.iconify()

    def on_maximize_button_clicked(self, widget, data=None):
        if self.fullscreen:
            self.top_level_window.unmaximize()
            self.top_level_window.unfullscreen()
            self.fullscreen = False
        else:
            self.top_level_window.maximize()
            self.fullscreen = True

    def on_close_button_clicked(self, widget, data=None):
        self.menu_bar_controller.on_quit_activate(None)

    def motion_detected(self, widget, event=None):
        if event.is_hint:
            x, y, state = event.window.get_pointer()
        else:
            state = event.state

        if state & gtk.gdk.BUTTON1_MASK:
            self.top_level_window.begin_move_drag(gtk.gdk.BUTTON1_MASK, int(event.x_root), int(event.y_root), 0)

    def button_pressed_event(self, widget, event=None):
        if event.type == gtk.gdk._2BUTTON_PRESS:
            self.on_maximize_button_clicked(None)