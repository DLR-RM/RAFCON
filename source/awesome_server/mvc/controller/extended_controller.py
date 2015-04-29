from gtkmvc import Controller


class ExtendedController(Controller):

    def __init__(self, model, view, spurious=False):
        Controller.__init__(self, model, view, spurious=spurious)
        self.__child_controllers = dict()

    def add_controller(self, key, controller):
        assert isinstance(controller, ExtendedController)
        self.__child_controllers[key] = controller

    def remove_controller(self, controller):
        # remove controller if controller is ExtendedController
        if isinstance(controller, ExtendedController):
            for key, child_controller in self.__child_controllers.iteritems():
                if controller is child_controller:
                    del self.__child_controllers[key]
                    return True
        # remove controller if controller is a string
        key = controller
        if key in self.__child_controllers:
            del self.__child_controllers[key]
            return True
        return False

    def get_controller(self, key):
        if key in self.__child_controllers:
            return self.__child_controllers[key]
        return None

    def get_child_controllers(self):
        return self.__child_controllers.values()

    def register_view(self, view):
        pass