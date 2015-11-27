from rafcon.utils import log

logger = log.get_logger(__name__)

from gtkmvc import Controller
from rafcon.mvc.shortcut_manager import ShortcutManager


class ExtendedController(Controller):
    def __init__(self, model, view, spurious=False):
        Controller.__init__(self, model, view, spurious=spurious)
        self.__child_controllers = dict()
        self.__shortcut_manager = None
        self.__action_registered_controllers = []

    def add_controller(self, key, controller):
        """Add child controller

        The passed controller is registered as child of self. The register_actions method of the child controller is
        called, allowing the child controller to register shortcut callbacks.

        :param key: Name of the controller (unique within self), to latter access it again
        :param ExtendedController controller: Controller to be added as child
        """
        assert isinstance(controller, ExtendedController)
        self.__child_controllers[key] = controller
        if self.__shortcut_manager is not None and controller not in self.__action_registered_controllers:
            controller.register_actions(self.__shortcut_manager)
            self.__action_registered_controllers.append(controller)

    def remove_controller(self, controller):
        """Remove child controller and destroy it

        Removes all references to the child controller and calls destroy() on the controller.

        :param str | ExtendedController controller: Either the child controller object itself or its registered name
        :return: Whether the controller was existing
        :rtype: bool
        """
        # Get name of controller
        if isinstance(controller, ExtendedController):
            for key, child_controller in self.__child_controllers.iteritems():
                if controller is child_controller:
                    break
            else:
                return False
        else:
            key = controller
        if key in self.__child_controllers:
            self.__action_registered_controllers.remove(self.__child_controllers[key])
            self.__child_controllers[key].destroy()
            del self.__child_controllers[key]
            return True
        return False

    def get_controller_by_path(self, ctrl_path, with_print=False):
        actual_ctrl = self
        for child_ctrl_identifier in ctrl_path:
            tmp_ctrl = actual_ctrl.get_controller(child_ctrl_identifier)
            if tmp_ctrl:
                actual_ctrl = tmp_ctrl
            else:
                if with_print:
                    logger.warning(
                        "{0} could not find cild_ctrl_identifier '{1}' ".format(actual_ctrl, child_ctrl_identifier))
                return None

        return actual_ctrl

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
                except Exception as e:
                    logger.error("Error while registering action for {0}: {1}".format(controller.__name__, e))
                    pass
                self.__action_registered_controllers.append(controller)

    def __register_actions_of_child_controllers(self):
        assert isinstance(self.__child_controllers, dict)
        for controller in self.__child_controllers.itervalues():
            register_function = getattr(controller, "register_actions", None)
            if hasattr(register_function, '__call__'):
                register_function(self.__shortcut_manager)

    def register_view(self, view):
        pass

    def destroy(self):
        """Recursively destroy all Controllers

        The method remove all controllers, which calls the destroy method of the child controllers. Then,
        all registered models are relieved and the widget is destroyed.
        """
        controller_names = [key for key in self.__child_controllers]
        for controller_name in controller_names:
            self.remove_controller(controller_name)
        self.relieve_all_models()
        self.view.get_top_widget().destroy()

    def relieve_all_models(self):
        """Relieve all registered models

        By the default, only self.model is relieved. However, inheriting controllers can overwrite this method,
        if they have more registered models.
        """
        self.relieve_model(self.model)
