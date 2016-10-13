"""
.. module:: global_variable_manager
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to access the GlobalVariableManager by a GUI based on the
     GlobalVariableEditorView.

.. moduleauthor:: Sebastian Brunner, Rico Belder


"""

from gtk import ListStore

from rafcon.mvc.controllers.utils.tab_key import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController

from rafcon.mvc.gui_helper import react_to_event
from rafcon.utils import log
from rafcon.utils import type_helpers

logger = log.get_logger(__name__)


class GlobalVariableManagerController(ExtendedController):
    """Controller handling the Global Variable Manager

    :param rafcon.mvc.models.global_variable_manager.GlobalVariableManagerModel model: The Global Variable Manager Model
    :param rafcon.mvc.views.global_variable_editor.GlobalVariableEditorView view: The GTK view showing the list of
        global variables.
    :ivar global_variable_counter: Counter for global variables to ensure unique names for new global variables.
    :ivar global_variables_list_store: A GTK list-like data structure to store global variables in.
    """

    def __init__(self, model, view):
        """Constructor"""
        ExtendedController.__init__(self, model, view)
        self.tab_edit_controller = MoveAndEditWithTabKeyListFeatureController(view['global_variable_tree_view'])

        self.global_variable_counter = 0
        # list store order -> gv_name, data_type, data_value, is_locked
        self.global_variables_list_store = ListStore(str, str, str, str)
        self.list_store_iterators = {}
        self._actual_entry = None
        self._locked = False

    def register_view(self, view):
        """Called when the View was registered"""

        view['global_variable_tree_view'].set_model(self.global_variables_list_store)

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

    def disconnect_actual_entry_widget(self):
        if self._actual_entry is not None:
            self._actual_entry[0].disconnect(self._actual_entry[1])
            self._actual_entry = None

    @property
    def cursor_in_valid_position(self):
        if self.view['global_variable_tree_view'].get_cursor()[0] is None or \
                self.view['global_variable_tree_view'].get_cursor()[0][0] is None:
            return False
        return True

    def global_variable_is_editable(self, gv_name, intro_message='edit'):

        if gv_name not in self.list_store_iterators or \
                not self.model.global_variable_manager.variable_exist(gv_name) or \
                self.model.global_variable_manager.is_locked(gv_name):
            message = ' if not existing' if not self.model.global_variable_manager.variable_exist(gv_name) else ''
            message += ', while no iterator is registered for its row' if gv_name not in self.list_store_iterators else ''
            message += ', while it is locked.' if self.model.global_variable_manager.is_locked(gv_name) else ''
            logger.error("{2} of global variable '{0}' is not possible{1}".format(gv_name, message, intro_message))
            return False
        return True

    def editing_started(self, renderer, editable, path):
        """ Callback method to connect entry-widget focus-out-event to the respective change-method.
        """
        if self.view['name_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_name))
        elif self.view['value_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_value))
        elif self.view['type_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_data_type))
        else:
            logger.error("Not registered Renderer was used")

    def editing_canceled(self, event):
        """ Callback method to disconnect entry-widget focus-out-event to the respective change-method.
        """
        self.disconnect_actual_entry_widget()

    def change_name(self, entry, event):
        """ Change-name-method to set the name of actual selected (row) global variable.
        """
        # logger.info("change name {0}".format(event.type))
        if self.cursor_in_valid_position:
            self.on_name_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0],
                                 text=entry.get_text())

    def change_value(self, entry, event):
        """ Change-value-method to set the value of actual selected (row) global variable.
        """
        # logger.info("change value {0}".format(event.type))
        if self.cursor_in_valid_position:
            self.on_value_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0],
                                  text=entry.get_text())

    def change_data_type(self, entry, event):
        """ Change-data-type-method to set the data_type of actual selected (row) data-port.
        """
        # logger.info("change data type {0}".format(event.type))
        if self.cursor_in_valid_position:
            self.on_data_type_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0],
                                      text=entry.get_text())

    def on_new_global_variable_button_clicked(self, *event):
        """Triggered when the New button in the Global Variables tab is clicked

        Creates a new global variable with default values and selects its row.
        """
        if react_to_event(self.view, self.view['global_variable_tree_view'], event):
            variable_name = "new_global_%s" % self.global_variable_counter
            self.global_variable_counter += 1
            try:
                self.model.global_variable_manager.set_variable(variable_name, None)
            except (RuntimeError, AttributeError, TypeError) as e:
                logger.warning("Adding of new global variable '{0}' failed -> Exception:{1}".format(variable_name, e))
            for row_num, iter_elem in enumerate(self.global_variables_list_store):
                if iter_elem[0] == variable_name:
                    self.view['global_variable_tree_view'].set_cursor(row_num)
                    break
            return True

    def on_delete_global_variable_button_clicked(self, *event):
        """Triggered when the Delete button in the Global Variables tab is clicked

        Deletes the selected global variable and re-selects next variable's row.
        """
        if react_to_event(self.view, self.view['global_variable_tree_view'], event):
            path = self.view["global_variable_tree_view"].get_cursor()[0]
            if path is not None:
                variable_name = self.global_variables_list_store[int(path[0])][0]
                try:
                    self.model.global_variable_manager.delete_variable(variable_name)
                except AttributeError as e:
                    logger.warning("Delete of global variable '{0}' failed -> Exception:{1}".format(variable_name, e))
                if len(self.global_variables_list_store) > 0:
                    self.view['global_variable_tree_view'].set_cursor(min(path[0], len(self.global_variables_list_store) - 1))
            return True

    def on_name_changed(self, widget, path, text):
        """Triggered when a global variable's name is edited

        Updates the global variable's name only if different and already in list store.

        :param path: The path identifying the edited variable
        :param text: New variable name
        """
        # logger.info("changing name widget: {0}, path: {1}, text: {2}".format(widget, path, text))
        variable_name = self.global_variables_list_store[int(path)][0]
        if variable_name == text or not self.global_variable_is_editable(variable_name, 'Name change'):
            return

        data_value = self.model.global_variable_manager.get_representation(variable_name)
        data_type = self.model.global_variable_manager.get_data_type(variable_name)
        self.disconnect_actual_entry_widget()
        self._locked = True
        try:
            self.model.global_variable_manager.delete_variable(variable_name)
            self.model.global_variable_manager.set_variable(text, data_value, data_type=data_type)
            variable_name = text
        except (AttributeError, RuntimeError, TypeError) as e:
            logger.warning(str(e))
        self._locked = False
        self.update_global_variables_list_store()
        self.select_entry(variable_name)

    def on_value_changed(self, widget, path, text):
        """Triggered when a global variable's value is edited.

        Updates the global variable's value only if different.

        :param path: The path identifying the edited variable
        :param text: New variable value
        """
        logger.info("changing value widget: {0}, path: {1}, text: {2}".format(widget, path, text))
        if self.global_variables_list_store[int(path)][2] == text:
            return
        global_variable_name = self.global_variables_list_store[int(path)][0]
        if not self.global_variable_is_editable(global_variable_name, 'Change of value'):
            return
        data_type = self.model.global_variable_manager.get_data_type(global_variable_name)
        old_value = self.model.global_variable_manager.get_representation(global_variable_name)

        # preserve type especially if type=NoneType
        if data_type in [type(old_value), type(None)]:
            old_type = data_type
            if data_type == type(None):
                old_type = type(old_value)
                logger.debug("Global variable list widget try to preserve type of variable {0} with type "
                            "'NoneType'".format(global_variable_name))
            try:
                new_value = type_helpers.convert_string_value_to_type_value(text, old_type)
            except (AttributeError, ValueError) as e:
                if data_type == type(None):
                    new_value = text
                    logger.warning("Value of global variable '{0}' with old value data type '{2}', with value '{3}' and"
                                   " data type NoneType was changed to string '{1}'"
                                   "".format(global_variable_name, new_value, type(old_value), old_value))
                else:
                    raise TypeError("Unexpected outcome of change value operation for global variable '{0}' and "
                                    "handed value '{1}' type '{2}' -> raised error {3}"
                                    "".format(global_variable_name, text, type(text), e))

        else:
            raise TypeError("Global variable manager has had no consistent value data type '{0}' "
                            "and data type '{1}' for variable '{2}'.".format(data_type,
                                                                             [type(old_value), type(None)],
                                                                             global_variable_name))

        self.disconnect_actual_entry_widget()
        try:
            self.model.global_variable_manager.set_variable(global_variable_name, new_value, data_type=data_type)
        except (RuntimeError, AttributeError, TypeError) as e:
            logger.error("Error while setting global variable {1} to value {2} -> raised error {0}"
                         "".format(global_variable_name, new_value, e))

    def on_data_type_changed(self, widget, path, text):
        """Try to set the port type the the newly entered one
        """
        # logger.info("changing type widget: {0}, path: {1}, text: {2}".format(widget, path, text))
        if self.global_variables_list_store[int(path)][1] == text:
            return
        global_variable_name = self.global_variables_list_store[int(path)][0]
        if not self.global_variable_is_editable(global_variable_name, 'Type change'):
            return
        old_value = self.model.global_variable_manager.get_representation(global_variable_name)

        # check if valid data type string
        try:
            new_data_type = type_helpers.convert_string_to_type(text)
        except (AttributeError, ValueError) as e:
            logger.error("Could not change data type to '{0}': {1}".format(text, e))
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
                            "therefore becomes empty new data type object '{3}' -> raised TypeError: {4}"
                            "".format(global_variable_name, old_value, new_data_type, new_value, e))

        # set value in global variable manager
        self.disconnect_actual_entry_widget()
        try:
            self.model.global_variable_manager.set_variable(global_variable_name, new_value, data_type=new_data_type)
        except (ValueError, RuntimeError, TypeError) as e:
            logger.error("Could not set new value unexpected failure {0} to value {1} -> raised error {2}"
                         "".format(global_variable_name, new_value, e))

    def select_entry(self, global_variable_name):
        """Selects the global variable entry belonging to the given global variable name"""
        for row_num, gv_row in enumerate(self.global_variables_list_store):
            if gv_row[0] == global_variable_name:
                self.view['global_variable_tree_view'].set_cursor(row_num)
                break

    @ExtendedController.observe("global_variable_manager", after=True)
    def assign_notification_state(self, model, prop_name, info):
        """Triggered when a change occurs in the Global Variable Manager.

        Calls update of hole list store in case new variable was added. Avoids to run updates without reasonable change.
        Holds tree store and updates row elements if is-locked or variable-value changes.
        """

        if info['method_name'] in ['set_locked_variable'] or self._locked or \
                info['result'] is Exception:
            return

        if info['method_name'] in ['lock_variable', 'unlock_variable']:
            key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
            if key in self.list_store_iterators:
                gv_row_path = self.global_variables_list_store.get_path(self.list_store_iterators[key])
                self.global_variables_list_store[gv_row_path][3] = self.model.global_variable_manager.is_locked(key)
        elif info['method_name'] in ['set_variable', 'delete_variable']:
            if info['method_name'] == 'set_variable':
                key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
                if key in self.list_store_iterators:
                    gv_row_path = self.global_variables_list_store.get_path(self.list_store_iterators[key])
                    self.global_variables_list_store[gv_row_path][2] = self.model.global_variable_manager.get_representation(key)
                    self.global_variables_list_store[gv_row_path][1] = self.model.global_variable_manager.get_data_type(key).__name__
                    return
            self.update_global_variables_list_store()
        else:
            logger.warning('Notification that is not handled')

    def update_global_variables_list_store(self):
        """Triggered after creation or deletion of a variable has taken place

        Updates the list of global variables.
        """
        # logger.info("update")
        self.list_store_iterators = {}
        self.global_variables_list_store.clear()
        keys = self.model.global_variable_manager.get_all_keys()
        keys.sort()
        for key in keys:
            iter = self.global_variables_list_store.append([key,
                                                            self.model.global_variable_manager.get_data_type(key).__name__,
                                                            str(self.model.global_variable_manager.get_representation(key)),
                                                            self.model.global_variable_manager.is_locked(key),
                                                            ])
            self.list_store_iterators[key] = iter
