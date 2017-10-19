# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: extended_controller
   :synopsis: A module that holds all extensions in respect to the gtkmvc.Controller that are used in rafcon.gui.

"""

from gtkmvc import Controller

from rafcon.gui.shortcut_manager import ShortcutManager
from rafcon.utils import log

logger = log.get_logger(__name__)


class ExtendedController(Controller):
    def __init__(self, model, view, spurious=False):
        self.__registered_models = set()
        super(ExtendedController, self).__init__(model, view, spurious=spurious)
        self.__action_registered_controllers = []
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

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
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
        all registered models are relieved and and the widget hand by the initial view argument is destroyed.
        """
        controller_names = [key for key in self.__child_controllers]
        for controller_name in controller_names:
            self.remove_controller(controller_name)
        self.relieve_all_models()
        if self.parent:
            self.__parent = None
        # TODO remove this check if possible -> insert to tolerate bad habit new action_signal and related functions
        if self.view is not None:
            self.view.get_top_widget().destroy()

    def observe_model(self, model):
        """Make this model observable within the controller

        The method also keeps track of all observed models, in order to be able to relieve them later on.

        :param gtkmvc.Model model: The model to be observed
        """
        self.__registered_models.add(model)
        return super(ExtendedController, self).observe_model(model)

    def relieve_model(self, model):
        """Do no longer observe the model

        The model is also removed from the internal set of tracked models.

        :param gtkmvc.Model model: The model to be relieved
        """
        self.__registered_models.remove(model)
        return super(ExtendedController, self).relieve_model(model)

    def relieve_all_models(self):
        """Relieve all registered models

        The method uses the set of registered models to relieve them.
        """
        map(self.relieve_model, list(self.__registered_models))
        self.__registered_models.clear()

    def get_root_window(self):
        if self.__parent:
            return self.__parent.get_root_window()
        return self.view.get_top_widget()
