"""
.. module:: global_variable_manager
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to access the GlobalVariableManager by a GUI based on the
     GlobalVariableEditorView.

.. moduleauthor:: Sebastian Brunner, Rico Belder


"""

from rafcon.utils import log

logger = log.get_logger(__name__)

from gtk import ListStore
import gtk

from rafcon.mvc.controllers.utils import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.utils.comparison import compare_variables


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
        self.global_variables_list_store = ListStore(str, str, str)
        self.list_store_iterators = {}
        self._actual_entry = None
        self._locked = False

    def register_view(self, view):
        """Called when the View was registered"""

        view['global_variable_tree_view'].set_model(self.global_variables_list_store)

        view['name_text'].set_property('editable', True)
        view['value_text'].set_property('editable', True)

        view['name_text'].connect('edited', self.on_name_changed)
        view['name_text'].connect('editing-started', self.editing_started)
        view['name_text'].connect('editing-canceled', self.editing_canceled)
        view['value_text'].connect('edited', self.on_value_changed)
        view['value_text'].connect('editing-started', self.editing_started)
        view['value_text'].connect('editing-canceled', self.editing_canceled)
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

    def editing_started(self, renderer, editable, path):
        """ Callback method to connect entry-widget focus-out-event to the respective change-method.
        """
        if self.view['name_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_name))
        elif self.view['value_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_value))
        else:
            logger.error("Not registered Renderer was used")

    def editing_canceled(self, event):
        """ Callback method to disconnect entry-widget focus-out-event to the respective change-method.
        """
        if self._actual_entry is not None:
            self._actual_entry[0].disconnect(self._actual_entry[1])
            self._actual_entry = None

    def change_name(self, entry, event):
        """ Change-name-method to set the name of actual selected (row) global variable.
        """
        # logger.info("change name")
        if self.view['global_variable_tree_view'].get_cursor()[0] is None or \
                not self.view['global_variable_tree_view'].get_cursor()[0][0]:
            return
        self.on_name_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0], text=entry.get_text())

    def change_value(self, entry, event):
        """ Change-value-method to set the value of actual selected (row) global variable.
        """
        # logger.info("change value {0}".format(event.type))
        if self.view['global_variable_tree_view'].get_cursor()[0] is None or \
                not self.view['global_variable_tree_view'].get_cursor()[0][0]:
            return
        self.on_value_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0], text=entry.get_text())

    def on_new_global_variable_button_clicked(self, *args):
        """Triggered when the New button in the Global Variables tab is clicked

        Creates a new global variable with default values and selects its row.
        """
        if isinstance(args[0], gtk.Button) or self.view['global_variable_tree_view'].is_focus():
            new_global_variable = "new_global_%s" % self.global_variable_counter
            self.global_variable_counter += 1
            self.model.global_variable_manager.set_variable(new_global_variable, "value")
            for row_num, iter_elem in enumerate(self.global_variables_list_store):
                if iter_elem[0] == new_global_variable:
                    self.view['global_variable_tree_view'].set_cursor(row_num)
                    break
            return True

    def on_delete_global_variable_button_clicked(self, *args):
        """Triggered when the Delete button in the Global Variables tab is clicked

        Deletes the selected global variable and re-selects next variable's row.
        """
        if isinstance(args[0], gtk.Button) or self.view['global_variable_tree_view'].is_focus():
            path = self.view["global_variable_tree_view"].get_cursor()[0]
            if path is not None:
                key = self.global_variables_list_store[int(path[0])][0]
                try:
                    self.model.global_variable_manager.delete_variable(key)
                except AttributeError as e:
                    logger.warning("Delete of globale variable '{0}' failed".format(key))
                if len(self.global_variables_list_store) > 0:
                    self.view['global_variable_tree_view'].set_cursor(min(path[0], len(self.global_variables_list_store) - 1))
            return True

    def on_name_changed(self, widget, path, text):
        """Triggered when a global variable's name is edited

        Updates the global variable's name only if different and already in list store.

        :param path: The path identifying the edited variable
        :param text: New variable name
        """
        # logger.info("changing name")
        old_key = self.global_variables_list_store[int(path)][0]
        if old_key == text or not old_key in self.list_store_iterators:
            return
        old_value = self.global_variables_list_store[int(path)][1]
        self._locked = True
        try:
            self.model.global_variable_manager.delete_variable(old_key)
            self.model.global_variable_manager.set_variable(text, old_value)
            old_key = text
        except (AttributeError, RuntimeError) as e:
            logger.warning(str(e))
        self._locked = False
        self.update_global_variables_list_store()
        self.select_entry(old_key)

    def on_value_changed(self, widget, path, text):
        """Triggered when a global variable's value is edited.

        Updates the global variable's value only if different.

        :param path: The path identifying the edited variable
        :param text: New variable value
        """
        # logger.info("changing value")
        if self.global_variables_list_store[int(path)][1] == text:
            return
        old_key = self.global_variables_list_store[int(path)][0]
        try:
            if not self.model.global_variable_manager.is_locked(old_key):
                self.model.global_variable_manager.set_variable(old_key, text)
        except RuntimeError as e:
            logger.exception(e)

    def select_entry(self, key):
        """Selects the global variable entry belonging to the given data_port_id"""
        for row_num, gv_entry in enumerate(self.global_variables_list_store):
            if gv_entry[0] == key:
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
            # logger.info("skip")
            return

        if info['method_name'] in ['lock_variable', 'unlock_variable']:
            key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
            if key in self.list_store_iterators:
                gv_row_path = self.global_variables_list_store.get_path(self.list_store_iterators[key])
                self.global_variables_list_store[gv_row_path][2] = self.model.global_variable_manager.is_locked(key)
        elif info['method_name'] in ['set_variable', 'delete_variable']:
            if info['method_name'] == 'set_variable':
                key = info.kwargs.get('key', info.args[1]) if len(info.args) > 1 else info.kwargs['key']
                if key in self.list_store_iterators:
                    gv_row_path = self.global_variables_list_store.get_path(self.list_store_iterators[key])
                    self.global_variables_list_store[gv_row_path][1] = self.model.global_variable_manager.get_representation(key)
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
                                                            self.model.global_variable_manager.get_representation(key),
                                                            self.model.global_variable_manager.is_locked(key)])
            self.list_store_iterators[key] = iter
