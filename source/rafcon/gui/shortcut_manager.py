# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from builtins import str, object
from functools import partial

from rafcon.gui.config import global_gui_config
from rafcon.utils import log
logger = log.get_logger(__name__)


class ShortcutManager(object):
    """Handles shortcuts

    Holds a mapping between shortcuts and action. Actions can be subscribed to. When a listed shortcut is triggered,
    all subscribers are notified.
    """

    def __init__(self, window):
        # Setup window to listen for accelerators
        self.main_window = window
        self.accel_group = Gtk.AccelGroup()
        self.main_window.add_accel_group(self.accel_group)

        self.__action_to_callbacks = {}
        self.__action_to_shortcuts = global_gui_config.get_config_value('SHORTCUTS', {})
        self.register_shortcuts()
        self.__controller_action_callbacks = {}

    def register_shortcuts(self):
        for action in self.__action_to_shortcuts:
            # Make sure, all shortcuts are in a list
            shortcuts = self.__action_to_shortcuts[action]
            if not isinstance(shortcuts, list):
                shortcuts = [shortcuts]
                self.__action_to_shortcuts[action] = shortcuts
            # Now register the shortcuts in the window to trigger the shortcut signal
            for shortcut in shortcuts:
                keyval, modifier_mask = Gtk.accelerator_parse(shortcut)
                if keyval == 0 and modifier_mask == 0:  # No valid shortcut
                    logger.warning("No valid shortcut for shortcut %s" % str(shortcut))
                    continue
                callback = partial(self.__on_shortcut, action)  # Bind the action to the callback function
                self.accel_group.connect(keyval, modifier_mask, Gtk.AccelFlags.VISIBLE, callback)

    def __on_shortcut(self, action, accel_group, window, key_value, modifier_mask):
        pointer = self.main_window.get_pointer()
        res = self.trigger_action(action, key_value, modifier_mask, cursor_position=(pointer.x, pointer.y))
        # If returning False, the shortcut is forwarded to GTK to be used for default actions (like copy and paste in
        #  a text field). If a controller wants to prevent this, it has to return True.
        return res

    def __get_action_for_shortcut(self, lookup_shortcut):
        for action in self.__action_to_shortcuts:
            shortcuts = self.__action_to_shortcuts[action]
            for shortcut in shortcuts:
                if shortcut == lookup_shortcut:
                    return action
        return None

    def add_callback_for_action(self, action, callback):
        """Adds a callback function to an action

        The method checks whether both action and callback are valid. If so, the callback is added to the list of
        functions called when the action is triggered.

        :param str action: An action like 'add', 'copy', 'info'
        :param callback: A callback function, which is called when action is triggered. It retrieves the event as
          parameter
        :return: True is the parameters are valid and the callback is registered, False else
        :rtype: bool
        """
        if hasattr(callback, '__call__'):  # Is the callback really a function?
            if action not in self.__action_to_callbacks:
                self.__action_to_callbacks[action] = []
            self.__action_to_callbacks[action].append(callback)

            controller = None
            try:
                controller = callback.__self__
            except AttributeError:
                try:
                    # Needed when callback was wrapped using functools.partial
                    controller = callback.func.__self__
                except AttributeError:
                    pass

            if controller:
                if controller not in self.__controller_action_callbacks:
                    self.__controller_action_callbacks[controller] = {}
                if action not in self.__controller_action_callbacks[controller]:
                    self.__controller_action_callbacks[controller][action] = []
                self.__controller_action_callbacks[controller][action].append(callback)

            return True

    def remove_callback_for_action(self, action, callback):
        """ Remove a callback for a specific action

        This is mainly for cleanup purposes or a plugin that replaces a GUI widget.

        :param str action: the cation of which the callback is going to be remove
        :param callback: the callback to be removed
        """
        if action in self.__action_to_callbacks:
            if callback in self.__action_to_callbacks[action]:
                self.__action_to_callbacks[action].remove(callback)

    def remove_callbacks_for_controller(self, controller):
        if controller in self.__controller_action_callbacks:
            for action in self.__controller_action_callbacks[controller]:
                for callback in self.__controller_action_callbacks[controller][action]:
                    self.remove_callback_for_action(action, callback)
            del self.__controller_action_callbacks[controller]

    def get_shortcut_for_action(self, action):
        """Get the shortcut(s) for the specified action

        :param str action: An action like 'add', 'copy', 'info'
        :return: None, if no action is not valid or no shortcut is exiting, a single shortcut or a list of shortcuts
            if one or more shortcuts are registered for that action.
        """
        if action in self.__action_to_shortcuts:
            return self.__action_to_shortcuts[action]
        return None

    def trigger_action(self, action, key_value, modifier_mask, **kwargs):
        """Calls the appropriate callback function(s) for the given action

        :param str action: The name of the action that was triggered
        :param key_value: The key value of the shortcut that caused the trigger
        :param modifier_mask: The modifier mask of the shortcut that caused the trigger
        :param cursor_position: The position of the cursor, relative to the main window.
        :return: Whether a callback was triggered
        :rtype: bool
        """
        res = False
        if action in self.__action_to_callbacks:
            for callback_function in self.__action_to_callbacks[action]:
                try:
                    ret = callback_function(key_value, modifier_mask, **kwargs)
                    # If at least one controller returns True, the whole result becomes True
                    res |= (False if ret is None else ret)
                except Exception as e:
                    logger.exception('Exception while calling callback methods for action "{0}": {1}'.format(action, e))
        return res

    def remove_shortcuts(self):
        for action in self.__action_to_shortcuts:
            shortcuts = self.__action_to_shortcuts[action]
            for shortcut in shortcuts:
                keyval, modifier_mask = Gtk.accelerator_parse(shortcut)
                self.accel_group.disconnect_key(keyval, modifier_mask)

    def update_shortcuts(self):
        logger.info("Updating Shortcuts")
        self.__action_to_shortcuts = global_gui_config.get_config_value('SHORTCUTS', {})
        self.register_shortcuts()

    def destroy(self):
        self.remove_shortcuts()
        self.__controller_action_callbacks.clear()
        self.main_window.remove_accel_group(self.accel_group)
        self.main_window = None
        self.accel_group = None

        self.__action_to_callbacks.clear()
        # this deletes the shortcuts form the global gui config, which is unnecessary!
        # self.__action_to_shortcuts.clear()
        self.__action_to_shortcuts = None

