from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.utils import log

logger = log.get_logger(__name__)


class ToolBarController(ExtendedController):
    """
    The class to trigger all the action, available in the tool bar.
    """

    def __init__(self, state_machine_manager_model, view, menu_bar_ctrl):
        ExtendedController.__init__(self, state_machine_manager_model, view)
        self.menu_bar_ctrl = menu_bar_ctrl
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

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """

    def on_button_refresh_libs_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_refresh_libraries_activate(widget, data)

    def on_button_save_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_save_activate(widget, data)

    def on_button_open_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_open_activate(widget, data)

    def on_button_refresh_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_refresh_all_activate(widget, data)

    def on_button_new_clicked(self, widget, data=None):
        self.menu_bar_ctrl.on_new_activate(widget, data)
