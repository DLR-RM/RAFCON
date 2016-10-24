"""
.. module:: extended_controller
   :platform: Unix, Windows
   :synopsis: A module that holds all extensions in respect to the gtkmvc.Controller that are used in rafcon.mvc.

.. moduleauthor:: Franz Steinmetz


"""

from gtkmvc import Controller

from rafcon.mvc.shortcut_manager import ShortcutManager
from rafcon.utils import log

logger = log.get_logger(__name__)


class ExtendedController(Controller):
    def __init__(self, model, view, spurious=False):
        self.__registered_models = set()
        Controller.__init__(self, model, view, spurious=spurious)
        self.__action_registered_controllers = []
        self.__signal_handler_ids = {}
        self.__child_controllers = dict()
        self.__shortcut_manager = None
        self.__parent = None

    def add_controller(self, key, controller):
        """Add child controller

        The passed controller is registered as child of self. The register_actions method of the child controller is
        called, allowing the child controller to register shortcut callbacks.

        :param key: Name of the controller (unique within self), to later access it again
        :param ExtendedController controller: Controller to be added as child
        """
        assert isinstance(controller, ExtendedController)
        controller.parent = self
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
            self.__child_controllers[key].unregister_actions(self.__shortcut_manager)
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
        """Return the child controller registered with the name key

        :param key: The name of the controller
        :return: The controller
        :rtype: ExtendedController
        """
        if key in self.__child_controllers:
            return self.__child_controllers[key]
        return None

    def get_child_controllers(self):
        """Returns a list with all registered child controllers

        :return: List of child controllers
        :rtype: list
        """
        return self.__child_controllers.values()

    @property
    def parent(self):
        """Return the parent controller for which this controller is registered as a child.

        :return: The parent controller
        :rtype: ExtendedController
        """
        return self.__parent

    @parent.setter
    def parent(self, controller):
        """Set the parent controller for which this controller is registered as a child.
        """
        assert isinstance(controller, ExtendedController)
        self.__parent = controller

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions in all child controllers.

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        assert isinstance(shortcut_manager, ShortcutManager)
        self.__shortcut_manager = shortcut_manager

        for controller in self.__child_controllers.values():
            if controller not in self.__action_registered_controllers:
                try:
                    controller.register_actions(shortcut_manager)
                except Exception as e:
                    logger.error("Error while registering action for {0}: {1}".format(controller.__name__, e))
                self.__action_registered_controllers.append(controller)

    def unregister_actions(self, shortcut_manager):
        for controller in self.__child_controllers.values():
            controller.unregister_actions(shortcut_manager)
        shortcut_manager.remove_callbacks_for_controller(self)

    def __register_actions_of_child_controllers(self):
        assert isinstance(self.__child_controllers, dict)
        for controller in self.__child_controllers.itervalues():
            register_function = getattr(controller, "register_actions", None)
            if hasattr(register_function, '__call__'):
                register_function(self.__shortcut_manager)

    def destroy(self):
        """Recursively destroy all Controllers

        The method remove all controllers, which calls the destroy method of the child controllers. Then,
        all registered models are relieved, by the controller connected signal handlers are disconnected
        and the widget is destroyed.
        """
        controller_names = [key for key in self.__child_controllers]
        for controller_name in controller_names:
            self.remove_controller(controller_name)
        self.relieve_all_models()
        self.disconnect_all_signal_handlers()

    def connect_signal_handler(self, widget, signal_name, callback, *callback_args):
        """ Connect signal handler function with handed widget

        The method is connecting signal handler functions with widget/view and stores respective handler ids and widgets
        relation to have the option to fully disconnecting controller methods from widget/view signals.
        If the view is continued to be used in future connected signal handler from old controllers could cause
        cross effects.

        :param widget: The widget for which signal a handler callback should be connected.
        :param str signal_name: Signal name.
        :param callback: Callback for respective emit signal.
        :param callback_args: Callback arguments.
        :rtype: int
        :return: handler_id
        """
        if hasattr(widget, 'connect'):
            # callback -> may be we should restrict it to own functions to prevent cross effects and bad coding
            try:
                handler_id = widget.connect(signal_name, callback, callback_args)
            except Exception as e:
                logger.error("Error while connecting signal handler for {0} and signal name: {1}: -> {2}"
                             "".format(widget, signal_name, e))
                return

            if widget in self.__signal_handler_ids:
                self.__signal_handler_ids[widget].append(handler_id)
            else:
                self.__signal_handler_ids[widget] = [handler_id]
            return handler_id
        else:
            logger.warning("The widget has to provide the connect method as a interface -> widget {0}".format(widget))

    def disconnect_signal_handler(self, widget, handler_id):
        """Disconnect signal handler.

        The method only disconnects signal handler that has been connected by the widget.

        :param widget: The widget the handler id belongs to.
        :param int handler_id: Signal handler id of connected callback.
        :return:
        """
        assert isinstance(handler_id, int)
        if widget in self.__signal_handler_ids:
            assert handler_id in self.__signal_handler_ids[widget]
            try:
                widget.disconnect(handler_id)
                self.__signal_handler_ids[widget].remove(handler_id)
            except Exception as e:
                logger.error("Error while disconnecting signal handler for {0}: -> {2}".format(widget, e))
                return
        else:
            logger.warning("This controller has no registered handler id for widget {0}".format(widget))

    def disconnect_all_signal_handlers(self):
        """Disconnects all connected signal handler functions

        The method uses the dictionary of connected signal handler ids for respective widgets.
        """
        for widget in self.__signal_handler_ids.keys():
            for handler_id in self.__signal_handler_ids[widget]:
                widget.disconnect(handler_id)
            del self.__signal_handler_ids[widget]

    def observe_model(self, model):
        """Make this model observable within the controller

        The method also keeps track of all observed models, in order to be able to relieve them later on.

        :param gtkmvc.Model model: The model to be observed
        """
        self.__registered_models.add(model)
        return super(ExtendedController, self).observe_model(model)

    def relieve_model(self, model):
        """Do no longer observe the model

        The model is also removed from the internal list of tracked models.

        :param gtkmvc.Model model: The model to be relieved
        """
        self.__registered_models.remove(model)
        return super(ExtendedController, self).relieve_model(model)

    def relieve_all_models(self):
        """Relieve all registered models

        The method uses the list of registered models to relieve them.
        """
        models = [model for model in self.__registered_models]
        map(self.relieve_model, models)
        self.__registered_models.clear()

    def get_root_window(self):
        if self.__parent:
            return self.__parent.get_root_window()
        return self.view.get_top_widget()
