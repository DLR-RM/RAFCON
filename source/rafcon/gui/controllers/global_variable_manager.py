# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: global_variable_manager
   :synopsis: A module that holds the controller to access the GlobalVariableManager by a GUI based on the
     GlobalVariableEditorView.

"""

from builtins import str
from gi.repository import Gtk
from gi.repository import GObject

from rafcon.gui.controllers.utils.tree_view_controller import ListViewController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController

from rafcon.utils import log
from rafcon.utils import type_helpers

logger = log.get_logger(__name__)


class GlobalVariableManagerController(ListViewController):
    """Controller handling the Global Variable Manager

     The controller enables to edit, add and remove global variable to the global variable manager by a tree view.
     Every global variable is accessible by it key which is in the tree view equivalent with its name and in the
     methods it is gv_name. This Controller inherit and use rudimentary methods of the ListViewController
     (therefore it introduce the ID_STORAGE_ID class attribute) and avoids to use the selection methods of those which
     need a MODEL_STORAGE_ID (there is no global variable model) and a state machine selection (is model based).
     Therefore the register view is only called for the extended controller. Because of this and the fact that
     name = key for a global variable ID_STORAGE_ID, NAME_STORAGE_ID and MODEL_STORAGE_ID are all equal.

    :param rafcon.gui.models.global_variable_manager.GlobalVariableManagerModel model: The Global Variable Manager Model
    :param rafcon.gui.views.global_variable_editor.GlobalVariableEditorView view: The GTK view showing the list of
        global variables.
    :ivar int global_variable_counter: Counter for global variables to ensure unique names for new global variables.
    :ivar Gtk.ListStore list_store: A gtk list store storing the rows of data of respective global variables in.
    """
    ID_STORAGE_ID = 0
    MODEL_STORAGE_ID = 0
    NAME_STORAGE_ID = 0
    DATA_TYPE_AS_STRING_STORAGE_ID = 1
    VALUE_AS_STRING_STORAGE_ID = 2
    IS_LOCKED_AS_STRING_STORAGE_ID = 3

    def __init__(self, model, view):
        # list store order -> gv_name, data_type, data_value, is_locked
        super(GlobalVariableManagerController, self).__init__(model, view,
                                                              view['global_variable_tree_view'],
                                                              Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING), logger)

        self.global_variable_counter = 0
        self.list_store_iterators = {}

    def register_view(self, view):
        """Called when the View was registered"""
        ExtendedController.register_view(self, view)  # no super to avoid sm based selection initialization
        view['name_text'].set_property('editable', True)
        view['value_text'].set_property('editable', True)
        view['type_text'].set_property('editable', True)

        self.tree_view.connect('key-press-event', self.tree_view_keypress_callback)
        self._apply_value_on_edited_and_focus_out(view['name_text'], self.apply_new_global_variable_name)
        self._apply_value_on_edited_and_focus_out(view['value_text'], self.apply_new_global_variable_value)
        self._apply_value_on_edited_and_focus_out(view['type_text'], self.apply_new_global_variable_type)
        view['new_global_variable_button'].connect('clicked', self.on_add)
        view['delete_global_variable_button'].connect('clicked', self.on_remove)
        view['lock_global_variable_button'].connect('clicked', self.on_lock)
        view['unlock_global_variable_button'].connect('clicked', self.on_unlock)
        self._tree_selection.set_mode(Gtk.SelectionMode.MULTIPLE)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.add_action_callback)

    def global_variable_is_editable(self, gv_name, intro_message='edit'):
        """Check whether global variable is locked

        :param str gv_name: Name of global variable to be checked
        :param str intro_message: Message which is used form a useful logger error message if needed
        :return:
        """
        if self.model.global_variable_manager.is_locked(gv_name):
            logger.error("{1} of global variable '{0}' is not possible, as it is locked".format(gv_name, intro_message))
            return False
        return True

    def on_add(self, widget, data=None):
        """Create a global variable with default value and select its row

        Triggered when the add button in the global variables tab is clicked.
        """
        gv_name = "new_global_%s" % self.global_variable_counter
        self.global_variable_counter += 1
        try:
            self.model.global_variable_manager.set_variable(gv_name, None)
        except (RuntimeError, AttributeError, TypeError) as e:
            logger.warning("Addition of new global variable '{0}' failed: {1}".format(gv_name, e))
        self.select_entry(gv_name)
        return True

    def on_lock(self, widget, data=None):
        """Locks respective selected core element"""
        path_list = None
        if self.view is not None:
            model, path_list = self.tree_view.get_selection().get_selected_rows()
        models = [self.list_store[path][self.MODEL_STORAGE_ID] for path in path_list] if path_list else []
        if models:
            if len(models) > 1:
                self._logger.warning("Please select only one element to be locked.")
            try:
                self.model.global_variable_manager.lock_variable(models[0])
            except AttributeError as e:
                self._logger.warning("The respective core element of {1}.list_store couldn't be locked. -> {0}"
                                  "".format(e, self.__class__.__name__))
            return True
        else:
            self._logger.warning("Please select an element to be locked.")

    def on_unlock(self, widget, data=None):
        """Locks respective selected core element"""
        path_list = None
        if self.view is not None:
            model, path_list = self.tree_view.get_selection().get_selected_rows()
        models = [self.list_store[path][self.MODEL_STORAGE_ID] for path in path_list] if path_list else []
        if models:
            if len(models) > 1:
                self._logger.warning("Please select only one element to be unlocked.")
            try:
                self.model.global_variable_manager.unlock_variable(models[0], None, force=True)
            except AttributeError as e:
                self._logger.warning("The respective core element of {1}.list_store couldn't be unlocked. -> {0}"
                                  "".format(e, self.__class__.__name__))
            return True
        else:
            self._logger.warning("Please select an element to be unlocked.")

    def remove_core_element(self, model):
        """Remove respective core element of handed global variable name

        :param str model: String that is the key/gv_name of core element which should be removed
        :return:
        """
        gv_name = model
        if self.global_variable_is_editable(gv_name, "Deletion"):
            try:
                self.model.global_variable_manager.delete_variable(gv_name)
            except AttributeError as e:
                logger.warning("The respective global variable '{1}' couldn't be removed. -> {0}"
                            "".format(e, model))

    def apply_new_global_variable_name(self, path, new_gv_name):
        """Change global variable name/key according handed string

        Updates the global variable name only if different and already in list store.

        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_gv_name: New global variable name
        """
        gv_name = self.list_store[path][self.NAME_STORAGE_ID]
        if gv_name == new_gv_name or not self.global_variable_is_editable(gv_name, 'Name change'):
            return

        data_value = self.model.global_variable_manager.get_representation(gv_name)
        data_type = self.model.global_variable_manager.get_data_type(gv_name)

        try:
            self.model.global_variable_manager.delete_variable(gv_name)
            self.model.global_variable_manager.set_variable(new_gv_name, data_value, data_type=data_type)
            gv_name = new_gv_name
        except (AttributeError, RuntimeError, TypeError) as e:
            logger.warning("Can not apply new name '{0}'".format(e))
        self.update_global_variables_list_store()
        self.select_entry(gv_name)

        # informing the tab key feature handler function about the changed core element id
        if hasattr(self.tree_view_keypress_callback.__func__, "core_element_id"):
            self.tree_view_keypress_callback.__func__.core_element_id = gv_name

    def apply_new_global_variable_value(self, path, new_value_as_string):
        """Change global variable value according handed string

        Updates the global variable value only if new value string is different to old representation.

        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_value_as_string: New global variable value as string
        """
        if self.list_store[path][self.DATA_TYPE_AS_STRING_STORAGE_ID] == new_value_as_string:
            return
        gv_name = self.list_store[path][self.NAME_STORAGE_ID]
        if not self.global_variable_is_editable(gv_name, 'Change of value'):
            return
        data_type = self.model.global_variable_manager.get_data_type(gv_name)
        old_value = self.model.global_variable_manager.get_representation(gv_name)

        # preserve type especially if type=NoneType
        if issubclass(data_type, (type(old_value), type(None))):
            old_type = data_type
            if issubclass(data_type, type(None)):
                old_type = type(old_value)
                logger.debug("Trying to parse '{}' to type '{}' of old global variable value '{}'".format(
                    new_value_as_string, old_type.__name__, old_value))
            try:
                new_value = type_helpers.convert_string_value_to_type_value(new_value_as_string, old_type)
            except (AttributeError, ValueError) as e:
                if issubclass(data_type, type(None)):
                    new_value = new_value_as_string
                    logger.warning("New value '{}' stored as string, previous value '{}' of global variable '{}' was "
                                   "of type '{}'".format(new_value, old_value, gv_name, type(old_value).__name__))
                else:
                    logger.warning("Restoring old value of global variable '{}': {}".format(gv_name, e))
                    return
        else:
            logger.error("Global variable '{}' with inconsistent value data type '{}' and data type '{}'".format(
                gv_name, [type(old_value).__name__, type(None).__name__], data_type.__name__))
            return

        try:
            self.model.global_variable_manager.set_variable(gv_name, new_value, data_type=data_type)
        except (RuntimeError, AttributeError, TypeError) as e:
            logger.error("Error while setting global variable '{0}' to value '{1}' -> Exception: {2}".format(
                gv_name, new_value, e))

    def apply_new_global_variable_type(self, path, new_data_type_as_string):
        """Change global variable value according handed string

        Updates the global variable data type only if different.

        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_data_type_as_string: New global variable data type as string
        """
        if self.list_store[path][self.DATA_TYPE_AS_STRING_STORAGE_ID] == new_data_type_as_string:
            return
        gv_name = self.list_store[path][self.NAME_STORAGE_ID]
        if not self.global_variable_is_editable(gv_name, 'Type change'):
            return
        old_value = self.model.global_variable_manager.get_representation(gv_name)

        # check if valid data type string
        try:
            new_data_type = type_helpers.convert_string_to_type(new_data_type_as_string)
        except (AttributeError, ValueError) as e:
            logger.error("Could not change data type to '{0}': {1}".format(new_data_type_as_string, e))
            return
        assert isinstance(new_data_type, type)

        # convert old value
        if issubclass(new_data_type, type(None)):
            new_value = old_value
        else:  # new_data_type in [str, float, int, list, dict, tuple, bool]:
            try:
                new_value = new_data_type(old_value)
            except (ValueError, TypeError) as e:
                new_value = new_data_type()
                logger.warning("Old value '{}' of global variable '{}' could not be parsed to new type '{}' and is "
                            "therefore resetted: {}".format(old_value, gv_name, new_data_type.__name__, e))

        # set value in global variable manager
        try:
            self.model.global_variable_manager.set_variable(gv_name, new_value, data_type=new_data_type)
        except (ValueError, RuntimeError, TypeError) as e:
            logger.error("Could not set new value unexpected failure '{0}' to value '{1}' -> Exception: {2}"
                         "".format(gv_name, new_value, e))

    @ListViewController.observe("global_variable_manager", after=True)
    def assign_notification_from_gvm(self, model, prop_name, info):
        """Handles gtkmvc3 notification from global variable manager

        Calls update of whole list store in case new variable was added. Avoids to run updates without reasonable change.
        Holds tree store and updates row elements if is-locked or global variable value changes.
        """

        if info['method_name'] in ['set_locked_variable'] or info['result'] is Exception:
            return

        if info['method_name'] in ['lock_variable', 'unlock_variable']:
            key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
            if key in self.list_store_iterators:
                gv_row_path = self.list_store.get_path(self.list_store_iterators[key])
                self.list_store[gv_row_path][self.IS_LOCKED_AS_STRING_STORAGE_ID] = \
                    str(self.model.global_variable_manager.is_locked(key))
        elif info['method_name'] in ['set_variable', 'delete_variable']:
            if info['method_name'] == 'set_variable':
                key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
                if key in self.list_store_iterators:
                    gv_row_path = self.list_store.get_path(self.list_store_iterators[key])
                    self.list_store[gv_row_path][self.VALUE_AS_STRING_STORAGE_ID] = \
                        str(self.model.global_variable_manager.get_representation(key))
                    self.list_store[gv_row_path][self.DATA_TYPE_AS_STRING_STORAGE_ID] = \
                        self.model.global_variable_manager.get_data_type(key).__name__
                    return
            self.update_global_variables_list_store()
        else:
            logger.warning('Notification that is not handled')

    def update_global_variables_list_store(self):
        """Updates the global variable list store

        Triggered after creation or deletion of a variable has taken place.
        """
        # logger.info("update")
        self.list_store_iterators = {}
        self.list_store.clear()
        keys = self.model.global_variable_manager.get_all_keys()
        keys.sort()
        for key in keys:
            iter = self.list_store.append([key,
                                           self.model.global_variable_manager.get_data_type(key).__name__,
                                           str(self.model.global_variable_manager.get_representation(key)),
                                           str(self.model.global_variable_manager.is_locked(key)),
                                           ])
            self.list_store_iterators[key] = iter
