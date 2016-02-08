from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.top_tool_bar import TopToolBarUndockedWindowController


class UndockedWindowController(ExtendedController):
    """

    """

    def __init__(self, state_machine_manager_model, view):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        self.top_tool_bar_undocked_window_controller = TopToolBarUndockedWindowController(state_machine_manager_model,
                                                                                          view.top_tool_bar,
                                                                                          view['undock_window'])

        self.add_controller('top_tool_bar_undocked_window_controller', self.top_tool_bar_undocked_window_controller)

    def hide_window(self):
        self.view['undock_window'].hide()

    def show_window(self):
        self.view['undock_window'].show()
