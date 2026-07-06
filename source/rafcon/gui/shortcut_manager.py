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

from gi.repository import Gtk, Gdk
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
        # GTK4 removed Gtk.AccelGroup; a Gtk.ShortcutController in bubble phase takes its place, so that focused
        # widgets (e.g. text entries) still receive their default key bindings first
        self.main_window = window
        # maps each registered top-level window to (Gtk.ShortcutController, [Gtk.Shortcut])
        self.__window_controllers = {}
        self.shortcut_controller = self.__create_controller_for_window(window)

        self.__action_to_callbacks = {}
        self.__action_to_shortcuts = global_gui_config.get_config_value('SHORTCUTS', {})
        self.register_shortcuts()
        self.__controller_action_callbacks = {}

    def __create_controller_for_window(self, window):
        controller = Gtk.ShortcutController()
        controller.set_scope(Gtk.ShortcutScope.GLOBAL)
        window.add_controller(controller)
        self.__window_controllers[window] = (controller, [])
        return controller

    def add_window(self, window):
        """Also listen for the registered shortcuts on an additional top-level window

        Replacement for the GTK3 pattern of adding the single Gtk.AccelGroup to several windows.
        """
        if window in self.__window_controllers:
            return
        self.__create_controller_for_window(window)
        self.__register_shortcuts_on_window(window)

    def register_shortcuts(self):
        for action in self.__action_to_shortcuts:
            # Make sure, all shortcuts are in a list
            shortcuts = self.__action_to_shortcuts[action]
            if not isinstance(shortcuts, list):
                self.__action_to_shortcuts[action] = [shortcuts]
        for window in self.__window_controllers:
            self.__register_shortcuts_on_window(window)

    def __register_shortcuts_on_window(self, window):
        controller, registered_shortcuts = self.__window_controllers[window]
        for action, shortcuts in self.__action_to_shortcuts.items():
            for shortcut in shortcuts:
                trigger = Gtk.ShortcutTrigger.parse_string(shortcut)
                success, keyval, modifier_mask = Gtk.accelerator_parse(shortcut)
                if trigger is None or not success:  # No valid shortcut
                    logger.warning("No valid shortcut for shortcut %s" % str(shortcut))
                    continue
                callback = partial(self.__on_shortcut, action, keyval, modifier_mask)
                gtk_shortcut = Gtk.Shortcut.new(trigger, Gtk.CallbackAction.new(callback))
                controller.add_shortcut(gtk_shortcut)
                registered_shortcuts.append(gtk_shortcut)

    def __on_shortcut(self, action, key_value, modifier_mask, widget, args):
        cursor_position = self.__get_pointer_position()
        res = self.trigger_action(action, key_value, modifier_mask, cursor_position=cursor_position)
        # If returning False, the shortcut is forwarded to GTK to be used for default actions (like copy and paste in
        #  a text field). If a controller wants to prevent this, it has to return True.
        return res

    def __get_pointer_position(self):
        """Returns the pointer position relative to the main window or None if it cannot be determined

        GTK4 removed Gtk.Widget.get_pointer(); the position is queried from the window surface instead.
        """
        surface = self.main_window.get_surface()
        display = self.main_window.get_display()
        if surface is None or display is None:
            return None
        seat = display.get_default_seat()
        if seat is None or seat.get_pointer() is None:
            return None
        success, x, y, _ = surface.get_device_position(seat.get_pointer())
        return (x, y) if success else None

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
        if callable(callback):  # Is the callback really a function?
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
        for controller, registered_shortcuts in self.__window_controllers.values():
            for gtk_shortcut in registered_shortcuts:
                controller.remove_shortcut(gtk_shortcut)
            del registered_shortcuts[:]

    def update_shortcuts(self):
        logger.info("Updating Shortcuts")
        self.__action_to_shortcuts = global_gui_config.get_config_value('SHORTCUTS', {})
        self.register_shortcuts()

    def destroy(self):
        self.remove_shortcuts()
        self.__controller_action_callbacks.clear()
        for window, (controller, _) in self.__window_controllers.items():
            window.remove_controller(controller)
        self.__window_controllers.clear()
        self.main_window = None
        self.shortcut_controller = None

        self.__action_to_callbacks.clear()
        # this deletes the shortcuts form the global gui config, which is unnecessary!
        self.__action_to_shortcuts = None
