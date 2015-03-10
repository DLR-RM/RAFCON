import traceback
import gtk

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from functools import partial


class ShortcutManager():
    """Handles shortcuts

    Holds a mapping between shortcuts and action. Actions can be subscribed to. When a listed shortcut is triggered,
    all subscribers are notified.
    """
    __action_to_shortcuts = dict()

    def __init__(self, window):
        # Setup window to listen for accelerators
        self.main_window = window
        self.accel_group = gtk.AccelGroup()
        self.main_window.add_accel_group(self.accel_group)

        self.__init_shortcuts()
        self.__register_shortcuts()
        self.__action_to_callbacks = dict()

    def __init_shortcuts(self):
        self.__action_to_shortcuts = {
            'copy': '<Control>C',
            'paste': '<Control>V',
            'cut': '<Control>X',
            'add': '<Control>A',
            'delete': ['<Control>D', 'Delete'],
            'group': '<Control>G',
            'ungroup': '<Control>U',
            'entry': '<Control>E',
            'fit': '<Control>space',
            'info': '<Control>q',
            'start': 'F5',
            'step_mode': 'F6',
            'pause': 'F7',
            'stop': 'F8',
            'step': 'F4',
            'backward_step_mode': 'F9',
            'reload': 'F5',
            'undo': '<Control>Z',
            'redo': ['<Control>Y', '<Control><Shift>Z']
        }

    def __register_shortcuts(self):
        for action in self.__action_to_shortcuts:
            # Make sure, all shortcuts are in a list
            shortcuts = self.__action_to_shortcuts[action]
            if not isinstance(shortcuts, list):
                shortcuts = [shortcuts]
                self.__action_to_shortcuts[action] = shortcuts
            # Now register the shortcuts in the window to trigger the shortcut signal
            for shortcut in shortcuts:
                keyval, modifier_mask = gtk.accelerator_parse(shortcut)
                if keyval == 0 and modifier_mask == 0:  # No valid shortcut
                    logger.warn("No valid shortcut for shortcut %s" % str(shortcut))
                    continue
                callback = partial(self.__on_shortcut, action)  # Bind the action to the callback function
                self.accel_group.connect_group(keyval, modifier_mask, gtk.ACCEL_VISIBLE, callback)

    def __on_shortcut(self, action, accel_group, window, key_value, modifier_mask):
        self.trigger_action(action, key_value, modifier_mask)

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
        :param action: An action like 'add', 'copy', 'info'
        :param callback: A callback function, which is called when action is triggered. It retrieves the event as
        parameter
        :return: True is the parameters are valid and the callback is registered, False else
        """
        #print "check add callback"
        if action in self.__action_to_shortcuts:  # Is the action valid?
            if hasattr(callback, '__call__'):  # Is the callback really a function?
                if action not in self.__action_to_callbacks:
                    self.__action_to_callbacks[action] = []
                assert isinstance(self.__action_to_callbacks[action], list)
                self.__action_to_callbacks[action].append(callback)
                #print "add callback"
                return True
        return False

    def get_shortcut_for_action(self, action):
        """Get the shortcut(s) for the specified action
        :param action: An action like 'add', 'copy', 'info'
        :return: None, if no action is not valid or no shortcut is exiting, a single shortcut or a list of shortcuts
        if one or more shortcuts are registered for that action.
        """
        if action in self.__action_to_shortcuts:
            return self.__action_to_shortcuts[action]
        return None

    def trigger_action(self, action, key_value, modifier_mask):
        """Calls the appropriate callback function(s) for the given action

        :param action: The name of the action that was triggered
        :param key_value: The key value of the shortcut that caused the trigger
        :param modifier_mask: The modifier mask of the shortcut that caused the trigger
        """
        if action in self.__action_to_callbacks:
            for callback_function in self.__action_to_callbacks[action]:
                try:
                    callback_function(key_value, modifier_mask)
                except Exception as e:
                    logger.error('Exception while calling callback methods for action "{0}": {1} \n{2}'.format(
                        action, e.message, traceback.format_exc()))
