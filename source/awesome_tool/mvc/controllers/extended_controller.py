
from gtkmvc import Controller
from mvc.shortcut_manager import ShortcutManager


class ExtendedController(Controller):

    def __init__(self, model, view):
        Controller.__init__(self, model, view)
        self.__child_controllers = dict()
        self.__shortcut_manager = None
        self.__action_registered_controllers = []

    def add_controller(self, key, controller):
        assert isinstance(controller, ExtendedController)
        self.__child_controllers[key] = controller
        if self.__shortcut_manager is not None and controller not in self.__action_registered_controllers:
            controller.register_actions(self.__shortcut_manager)
            self.__action_registered_controllers.append(controller)

    def remove_controller(self, controller):
        if isinstance(controller, ExtendedController):
            if controller in self.__child_controllers.keys():
                self.__action_registered_controllers.remove(self.__child_controllers[controller])
            for key, child_controller in self.__child_controllers.iteritems():
                if controller is child_controller:
                    del self.__child_controllers[key]
                    return True
        key = controller
        if key in self.__child_controllers:
            self.__action_registered_controllers.remove(self.__child_controllers[controller])
            del self.__child_controllers[key]
            return True
        return False

    def get_controller(self, key):
        if key in self.__child_controllers:
            return self.__child_controllers[key]
        return None

    def get_child_controllers(self):
        return self.__child_controllers.values()

    def register_actions(self, shortcut_manager):
        assert isinstance(shortcut_manager, ShortcutManager)
        self.__shortcut_manager = shortcut_manager

        for controller in self.__child_controllers.values():
            if controller not in self.__action_registered_controllers:
                try:
                    controller.register_actions(shortcut_manager)
                except Exception:
                    pass
                self.__action_registered_controllers.append(controller)

    def __register_actions_of_child_controllers(self):
        assert isinstance(self.child_controllers, dict)
        for controller in self.child_controllers.itervalues():
            register_function = getattr(controller, "register_actions", None)
            if hasattr(register_function, '__call__'):
                register_function(self.__shortcut_manager)

    def register_view(self, view):
        pass