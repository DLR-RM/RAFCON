from rafcon.utils import log

logger = log.get_logger(__name__)

import gobject
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
        # self.tab_edit_controller = MoveAndEditWithTabKeyListFeatureController(view['global_variable_tree_view'])

        self.global_variable_counter = 0
        self.global_variables_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self._actual_entry = None

    def register_view(self, view):
        """Called when the View was registered"""
        def cell_text(column, cell_renderer, model, iter, gvm_model):
            col = column.get_name()
            global_variable = model.get_value(iter, 0)
            if col == 'name_col':
                cell_renderer.set_property('text', global_variable[0])
            elif col == 'value_col':
                cell_renderer.set_property('text', global_variable[1])
            elif col == 'locked_col':
                locked = gvm_model.global_variable_manager.is_locked(global_variable[0])
                cell_renderer.set_property('text', locked)
            else:
                logger.error("Unknown column '{col:s}' in GlobalVariableManagerView".format(col=col))

        view['global_variable_tree_view'].set_model(self.global_variables_list_store)

        view['name_col'].set_cell_data_func(view['name_text'], cell_text, self.model)
        view['name_text'].set_property('editable', True)
        view['value_col'].set_cell_data_func(view['value_text'], cell_text, self.model)
        view['value_text'].set_property('editable', True)
        view['locked_col'].set_cell_data_func(view['locked_text'], cell_text, self.model)

        view['name_text'].connect('edited', self.on_name_changed)
        # view['name_text'].connect('editing-started', self.editing_started)
        # view['name_text'].connect('editing-canceled', self.editing_canceled)
        view['value_text'].connect('edited', self.on_value_changed)
        # view['value_text'].connect('editing-started', self.editing_started)
        # view['value_text'].connect('editing-canceled', self.editing_canceled)
        view['new_global_variable_button'].connect('clicked', self.on_new_global_variable_button_clicked)
        view['delete_global_variable_button'].connect('clicked', self.on_delete_global_variable_button_clicked)

        # self.tab_edit_controller.register_view()

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
        if self.view['global_variable_tree_view'].get_cursor()[0] is None or \
                not self.view['global_variable_tree_view'].get_cursor()[0][0]:
            return
        self.on_name_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0], text=entry.get_text())

    def change_value(self, entry, event):
        """ Change-value-method to set the value of actual selected (row) global variable.
        """
        if self.view['global_variable_tree_view'].get_cursor()[0] is None or \
                not self.view['global_variable_tree_view'].get_cursor()[0][0]:
            return
        self.on_value_changed(entry, self.view['global_variable_tree_view'].get_cursor()[0][0], text=entry.get_text())

    def on_new_global_variable_button_clicked(self, *args):
        """Triggered when the New button in the Global Variables tab is clicked

        Creates a new global variable with default values.
        """
        if self.view['global_variable_tree_view'].is_focus():
            new_global_variable = "new_global_%s" % self.global_variable_counter
            self.global_variable_counter += 1
            self.model.global_variable_manager.set_variable(new_global_variable, "value")
            for row_num, iter_elem in enumerate(self.global_variables_list_store):
                if iter_elem[0][0] == new_global_variable:
                    self.view['global_variable_tree_view'].set_cursor(row_num)
                    break

    def on_delete_global_variable_button_clicked(self, *args):
        """Triggered when the Delete button in the Global Variables tab is clicked

        Deletes the selected global variable.
        """
        if self.view['global_variable_tree_view'].is_focus():
            path = self.view["global_variable_tree_view"].get_cursor()[0]
            if path is not None:
                key = self.global_variables_list_store[int(path[0])][0][0]
                try:
                    self.model.global_variable_manager.delete_variable(key)
                except AttributeError as e:
                    logger.warning("Delete of globale variable '{0}' failed".format(key))
            if len(self.global_variables_list_store) > 0:
                self.view['global_variable_tree_view'].set_cursor(min(path, len(self.global_variables_list_store) - 1))

    def on_name_changed(self, widget, path, text):
        """Triggered when a global variable's name is edited

        Updates the global variable's name.

        :param path: The path identifying the edited variable
        :param text: New variable name
        """
        old_key = self.global_variables_list_store[int(path)][0][0]
        old_value = self.global_variables_list_store[int(path)][0][1]
        self.model.global_variable_manager.delete_variable(old_key)
        self.model.global_variable_manager.set_variable(text, old_value)
        self.update_global_variables_list_store()

    def on_value_changed(self, widget, path, text):
        """Triggered when a global variable's value is edited.

        Updates the global variable's value.

        :param path: The path identifying the edited variable
        :param text: New variable value
        """
        old_key = self.global_variables_list_store[int(path)][0][0]
        self.model.global_variable_manager.set_variable(old_key, text)
        self.update_global_variables_list_store()

    @ExtendedController.observe("global_variable_manager", after=True)
    def assign_notification_state(self, model, prop_name, info):
        """Triggered when a change occurs in the Global Variable Manager."""
        self.update_global_variables_list_store()

    def update_global_variables_list_store(self):
        """Triggered after creation, deletion, or update of a variables has taken place

        Updates the list of global variables.
        """
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        keys = self.model.global_variable_manager.get_all_keys()
        for key in keys:
            tmp.append([[key, self.model.global_variable_manager.get_representation(key)]])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, compare_variables)
        tms.sort_column_changed()
        tmp = tms
        self.global_variables_list_store.clear()
        for elem in tmp:
            self.global_variables_list_store.append(elem)
