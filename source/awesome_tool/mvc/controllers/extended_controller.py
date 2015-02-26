
from gtkmvc import Controller


class ExtendedController(Controller):

    def __init__(self, model, view):
        Controller.__init__(self, model, view)
        self.child_controllers = dict()

    def register_actions(self, shortcut_manager):
        self.__register_actions_of_child_controllers(shortcut_manager)
        pass

    def __register_actions_of_child_controllers(self, shortcut_manager):
        assert isinstance(self.child_controllers, dict)
        for controller in self.child_controllers.itervalues():
            register_function = getattr(controller, "register_actions", None)
            if hasattr(register_function, '__call__'):
                register_function(shortcut_manager)