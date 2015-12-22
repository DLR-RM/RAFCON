from rafcon.utils import log

logger = log.get_logger(__name__)

import gobject
from gtk import ListStore
import gtk

from rafcon.mvc.controllers.extended_controller import ExtendedController


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

        self.global_variable_counter = 0
        self.global_variables_list_store = ListStore(gobject.TYPE_PYOBJECT)

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
        view['value_text'].connect('edited', self.on_value_changed)
        view['new_global_variable_button'].connect('clicked', self.on_new_global_variable_button_clicked)
        view['delete_global_variable_button'].connect('clicked', self.on_delete_global_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def on_new_global_variable_button_clicked(self, widget, data=None):
        """Triggered when the New button in the Global Variables tab is clicked

        Creates a new global variable.
        """
        new_global_variable = "new_global_%s" % self.global_variable_counter
        self.global_variable_counter += 1
        self.model.global_variable_manager.set_variable(new_global_variable, "value")

    def on_delete_global_variable_button_clicked(self, widget, data=None):
        """Triggered when the Delete button in the Global Variables tab is clicked

        Deletes the selected global variable.
        """
        tree_view = self.view["global_variable_tree_view"]
        path = tree_view.get_cursor()[0]
        if path is not None:
            key = self.global_variables_list_store[int(path[0])][0][0]
            self.model.global_variable_manager.delete_variable(key)

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
        tms.set_sort_func(0, self.compare_global_variables)
        tms.sort_column_changed()
        tmp = tms
        self.global_variables_list_store.clear()
        for elem in tmp:
            self.global_variables_list_store.append(elem)

    @staticmethod
    def compare_global_variables(treemodel, iter1, iter2, user_data=None):
        """Triggered upon updating the list of global variables

        Helper method to sort global variables alphabetically.

        :param treemodel: Tree model implementing the gtk.TreeSortable interface.
        :param iter1: Points at a row.
        :param iter2: Points at a row.
        """
        path1 = treemodel.get_path(iter1)[0]
        path2 = treemodel.get_path(iter2)[0]
        # get key of first variable
        name1 = treemodel[path1][0][0]
        # get key of second variable
        name2 = treemodel[path2][0][0]
        name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
        name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
        if name1_as_bits == name2_as_bits:
            return 0
        elif name1_as_bits > name2_as_bits:
            return 1
        else:
            return -1
