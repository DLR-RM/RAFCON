"""
.. module:: single_widget_window
   :platform: Unix, Windows
   :synopsis: The module provides a template controller for single widget tests.

.. moduleauthor:: Rico Belder


"""

import gtk
from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.shortcut_manager import ShortcutManager


class SingleWidgetWindowController(ExtendedController):
    """Controller handling the view of properties/attributes of ...
    """

    def __init__(self, model, view, ctrl_class):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.shortcut_manager = None
        self.add_controller('widget_ctrl', ctrl_class(model, view.get_top_widget()))

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        self.shortcut_manager = ShortcutManager(self.view['main_window'])
        self.register_actions(self.shortcut_manager)

        view['main_window'].connect('destroy', gtk.main_quit)
        # view.get_top_widget().connect('destroy', gtk.main_quit)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        # self.adapt(self.__state_property_adapter("name", "input_name"))

    pass  # class end
