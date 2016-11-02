"""
.. module:: global_variable_manager
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to access the GlobalVariableManager by a GUI based on the
     GlobalVariableEditorView.

.. moduleauthor:: Sebastian Brunner, Rico Belder


"""

import gtk
import glib

from rafcon.mvc.controllers.utils.tab_key import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils.selection import ListSelectionFeatureController

from rafcon.mvc.gui_helper import react_to_event
from rafcon.utils import log
from rafcon.utils import type_helpers

logger = log.get_logger(__name__)


class GlobalVariableManagerController(ExtendedController, ListSelectionFeatureController):
    """Controller handling the Global Variable Manager

     The controller enables to edit, add and remove global variable to the global variable manager by a tree view.
     Every global variable is accessible by it key which is in the tree view equivalent with its name and in the
     methods it is gv_name. This Controller inherit and use rudimentary methods of the ListSelectionFeatureController
     (therefore it introduce the ID_STORAGE_ID class attribute) and avoids to use the selection methods of those which
     need a MODEL_STORAGE_ID and a state machine selection and it is not registering the view to the mixed in controller.

    :param rafcon.mvc.models.global_variable_manager.GlobalVariableManagerModel model: The Global Variable Manager Model
    :param rafcon.mvc.views.global_variable_editor.GlobalVariableEditorView view: The GTK view showing the list of
        global variables.
    :ivar int global_variable_counter: Counter for global variables to ensure unique names for new global variables.
    :ivar gtk.ListStore list_store: A gtk list store storing the rows of data of respective global variables in.
    """
    ID_STORAGE_ID = 0
    NAME_STORAGE_ID = 0
    DATA_TYPE_AS_STRING_STORAGE_ID = 1
    VALUE_AS_STRING_STORAGE_ID = 2
    IS_LOCKED_AS_STRING_STORAGE_ID = 3

    def __init__(self, model, view):
        """Constructor"""
        # list store order -> gv_name, data_type, data_value, is_locked
        self.list_store = gtk.ListStore(str, str, str, str)
        ExtendedController.__init__(self, model, view)
        self.tree_view = view['global_variable_tree_view']
        self.tree_view.set_model(self.list_store)
        ListSelectionFeatureController.__init__(self, self.list_store, self.tree_view)
        self.tab_edit_controller = MoveAndEditWithTabKeyListFeatureController(self.tree_view)

        self.global_variable_counter = 0
        self.list_store_iterators = {}
        self._actual_entry = None
        self._locked = False

    def register_view(self, view):
        """Called when the View was registered"""
        view['name_text'].set_property('editable', True)
        view['value_text'].set_property('editable', True)
        view['type_text'].set_property('editable', True)

        view['name_text'].connect('edited', self.on_name_changed)
        view['name_text'].connect('editing-started', self.editing_started)
        view['name_text'].connect('editing-canceled', self.editing_canceled)
        view['value_text'].connect('edited', self.on_value_changed)
        view['value_text'].connect('editing-started', self.editing_started)
        view['value_text'].connect('editing-canceled', self.editing_canceled)
        view['type_text'].connect('edited', self.on_data_type_changed)
        view['type_text'].connect('editing-started', self.editing_started)
        view['type_text'].connect('editing-canceled', self.editing_canceled)
        view['new_global_variable_button'].connect('clicked', self.on_new_global_variable_button_clicked)
        view['delete_global_variable_button'].connect('clicked', self.on_delete_global_variable_button_clicked)
        self._tree_selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.tab_edit_controller.register_view()

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self.on_delete_global_variable_button_clicked)
        shortcut_manager.add_callback_for_action("add", self.on_new_global_variable_button_clicked)

    def global_variable_is_editable(self, gv_name, intro_message='edit'):
        """Check whether global variable is locked

        :param str gv_name: Name of global variable to be checked
        :param str intro_message: Message which is used form a useful logger error message if needed
        :return:
        """
        if gv_name not in self.list_store_iterators or \
                not self.model.global_variable_manager.variable_exist(gv_name) or \
                self.model.global_variable_manager.is_locked(gv_name):
            message = ' if not existing' if not self.model.global_variable_manager.variable_exist(gv_name) else ''
            message += ', while no iterator is registered for its row' if gv_name not in self.list_store_iterators else ''
            message += ', while it is locked.' if self.model.global_variable_manager.is_locked(gv_name) else ''
            logger.error("{2} of global variable '{0}' is not possible -> Exception: {1}".format(gv_name, message, intro_message))
            return False
        return True

    def disconnect_actual_entry_widget(self):
        """Disconnect actual observed entry widget used for edit"""
        if self._actual_entry is not None:
            self._actual_entry[0].disconnect(self._actual_entry[1])
            self._actual_entry = None

    def editing_started(self, renderer, editable, path):
        """Callback method to connect focus out event of entry widget to the respective change method"""
        if self.view['name_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_name))
        elif self.view['value_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_value))
        elif self.view['type_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_data_type))
        else:
            logger.error("Not registered Renderer was used")

    def editing_canceled(self, event):
        """Callback method to disconnect focus out event of entry widget to the respective change method"""
        self.disconnect_actual_entry_widget()

    def change_name(self, entry, event):
        """Change name based on handed entry widget

        Set the name of actual selected (row) global variable if focused out of entry widget.
        """
        # logger.info("change name {0}".format(event.type))
        if self.get_list_store_row_from_cursor_selection() is not None:
            # We have to use idle_add to prevent core dumps:
            # https://mail.gnome.org/archives/gtk-perl-list/2005-September/msg00143.html
            glib.idle_add(self.on_name_changed, entry, self.get_path(), entry.get_text())

    def change_value(self, entry, event):
        """Change value based on handed entry widget

        Set the value of actual selected (row) global variable if focused out of entry widget.
        """
        # logger.info("change value {0}".format(event.type))
        if self.get_list_store_row_from_cursor_selection() is not None:
            glib.idle_add(self.on_value_changed, entry, self.get_path(), entry.get_text())

    def change_data_type(self, entry, event):
        """Change data type method to set the data_type of actual selected (row) global variable.
        """
        # logger.info("change data type {0}".format(event.type))
        if self.get_list_store_row_from_cursor_selection() is not None:
            glib.idle_add(self.on_data_type_changed, entry, self.get_path(), entry.get_text())

    def on_new_global_variable_button_clicked(self, *event):
        """Creates a new global variable with default values and selects its row

        Triggered when the 'New' button in the global variables tab is clicked.
        """
        if react_to_event(self.view, self.tree_view, event):
            gv_name = "new_global_%s" % self.global_variable_counter
            self.global_variable_counter += 1
            try:
                self.model.global_variable_manager.set_variable(gv_name, None)
            except (RuntimeError, AttributeError, TypeError) as e:
                logger.warning("Adding of new global variable '{0}' failed -> Exception: {1}".format(gv_name, e))
            self.select_entry(gv_name)
            return True

    def on_delete_global_variable_button_clicked(self, *event):
        """Remove the selected global variables and re-select next variable row

        Triggered when the 'Delete' button in the global variables tab is clicked.
        """
        if react_to_event(self.view, self.tree_view, event):
            tree, path_list = self.tree_view.get_selection().get_selected_rows()
            old_path = self.get_path()
            gv_names = [self.list_store[path][self.ID_STORAGE_ID] for path in path_list] if path_list else []
            if gv_names:
                for gv_name in gv_names:
                    try:
                        self.model.global_variable_manager.delete_variable(gv_name)
                    except AttributeError as e:
                        logger.warning("Delete of global variable '{0}' failed -> Exception: {1}".format(gv_name, e))
                if len(self.list_store) > 0:
                    self.tree_view.set_cursor(min(old_path[0], len(self.list_store) - 1))
            return True

    def on_name_changed(self, widget, path, new_gv_name):
        """Change global variable name/key according to handed string

        Updates the global variable name only if different and already in list store.

        :param gtk.Object widget: Object which is the source of method call, e.g. signal callback
        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_gv_name: New global variable name
        """
        # logger.info("changing name widget: {0}, path: {1}, text: {2}".format(widget, path, new_gv_name))
        gv_name = self.list_store[path][self.NAME_STORAGE_ID]
        if gv_name == new_gv_name or not self.global_variable_is_editable(gv_name, 'Name change'):
            return

        data_value = self.model.global_variable_manager.get_representation(gv_name)
        data_type = self.model.global_variable_manager.get_data_type(gv_name)
        self.disconnect_actual_entry_widget()
        self._locked = True
        try:
            self.model.global_variable_manager.delete_variable(gv_name)
            self.model.global_variable_manager.set_variable(new_gv_name, data_value, data_type=data_type)
            gv_name = new_gv_name
        except (AttributeError, RuntimeError, TypeError) as e:
            logger.warning(str(e))
        self._locked = False
        self.update_global_variables_list_store()
        self.select_entry(gv_name)

    def on_value_changed(self, widget, path, new_value_as_string):
        """Change global variable value according handed string

        Updates the global variable value only if new value string is different to old representation.

        :param gtk.Object widget: Object which is the source of method call, e.g. signal callback
        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_value_as_string: New global variable value as string
        """
        # logger.info("changing value widget: {0}, path: {1}, text: {2}".format(widget, path, new_value_as_string))
        if self.list_store[path][self.DATA_TYPE_AS_STRING_STORAGE_ID] == new_value_as_string:
            return
        gv_name = self.list_store[path][self.NAME_STORAGE_ID]
        if not self.global_variable_is_editable(gv_name, 'Change of value'):
            return
        data_type = self.model.global_variable_manager.get_data_type(gv_name)
        old_value = self.model.global_variable_manager.get_representation(gv_name)

        # preserve type especially if type=NoneType
        if data_type in [type(old_value), type(None)]:
            old_type = data_type
            if data_type == type(None):
                old_type = type(old_value)
                logger.debug("Global variable list widget try to preserve type of variable '{0}' with type "
                             "'NoneType'".format(gv_name))
            try:
                new_value = type_helpers.convert_string_value_to_type_value(new_value_as_string, old_type)
            except (AttributeError, ValueError) as e:
                if data_type == type(None):
                    new_value = new_value_as_string
                    logger.warning("Value of global variable '{0}' with old value data type '{2}', with value '{3}' and"
                                   " data type NoneType was changed to string '{1}'"
                                   "".format(gv_name, new_value, type(old_value), old_value))
                else:
                    raise TypeError("Unexpected outcome of change value operation for global variable '{0}' and "
                                    "handed value '{1}' type '{2}' -> Exception: {3}"
                                    "".format(gv_name, new_value_as_string, type(new_value_as_string), e))

        else:
            raise TypeError("Global variable manager has had no consistent value data type '{0}' "
                            "and data type '{1}' for variable '{2}'.".format(data_type,
                                                                             [type(old_value), type(None)],
                                                                             gv_name))

        self.disconnect_actual_entry_widget()
        try:
            self.model.global_variable_manager.set_variable(gv_name, new_value, data_type=data_type)
        except (RuntimeError, AttributeError, TypeError) as e:
            logger.error("Error while setting global variable '{0}' to value '{1}' -> Exception: {2}"
                         "".format(gv_name, new_value, e))

    def on_data_type_changed(self, widget, path, new_data_type_as_string):
        """Change global variable value according handed string

        Updates the global variable data type only if different.

        :param gtk.Object widget: Object which is the source of method call, e.g. signal callback
        :param path: The path identifying the edited global variable tree view row, can be str, int or tuple.
        :param str new_data_type_as_string: New global variable data type as string
        """
        # logger.info("changing type widget: {0}, path: {1}, text: {2}".format(widget, path, new_data_type_as_string))
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
            logger.error("Could not change data type to '{0}' -> Exception: {1}".format(new_data_type_as_string, e))
            return
        assert isinstance(new_data_type, type)

        # convert old value
        if new_data_type == type(None):
            new_value = old_value
        else:  # new_data_type in [str, float, int, list, dict, tuple, bool]:
            try:
                new_value = new_data_type(old_value)
            except (ValueError, TypeError) as e:
                new_value = new_data_type()
                logger.info("Global variable '{0}' old value '{1}' is not convertible to new data type '{2}'"
                            "therefore becomes empty new data type object '{3}' -> Exception: {4}"
                            "".format(gv_name, old_value, new_data_type, new_value, e))

        # set value in global variable manager
        self.disconnect_actual_entry_widget()
        try:
            self.model.global_variable_manager.set_variable(gv_name, new_value, data_type=new_data_type)
        except (ValueError, RuntimeError, TypeError) as e:
            logger.error("Could not set new value unexpected failure '{0}' to value '{1}' -> Exception: {2}"
                         "".format(gv_name, new_value, e))

    @ExtendedController.observe("global_variable_manager", after=True)
    def assign_notification_state(self, model, prop_name, info):
        """Handles gtkmvc notification from global variable manager

        Calls update of whole list store in case new variable was added. Avoids to run updates without reasonable change.
        Holds tree store and updates row elements if is-locked or global variable value changes.
        """

        if info['method_name'] in ['set_locked_variable'] or self._locked or \
                info['result'] is Exception:
            return

        if info['method_name'] in ['lock_variable', 'unlock_variable']:
            key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
            if key in self.list_store_iterators:
                gv_row_path = self.list_store.get_path(self.list_store_iterators[key])
                self.list_store[gv_row_path][self.IS_LOCKED_AS_STRING_STORAGE_ID] = \
                    self.model.global_variable_manager.is_locked(key)
        elif info['method_name'] in ['set_variable', 'delete_variable']:
            if info['method_name'] == 'set_variable':
                key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
                if key in self.list_store_iterators:
                    gv_row_path = self.list_store.get_path(self.list_store_iterators[key])
                    self.list_store[gv_row_path][self.VALUE_AS_STRING_STORAGE_ID] = \
                        self.model.global_variable_manager.get_representation(key)
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
                                           self.model.global_variable_manager.is_locked(key),
                                           ])
            self.list_store_iterators[key] = iter
