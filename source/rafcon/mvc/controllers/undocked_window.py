from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.top_tool_bar import TopToolBarUndockedWindowController


class UndockedWindowController(ExtendedController):
    """Controller handling the un-docked windows

    :param rafcon.mvc.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.mvc.views.undocked_window.UndockedWindowView view: The GTK View showing the separate window
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
