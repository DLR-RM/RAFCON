import gtk
from gtk import ListStore
import gobject
from gtk.keysyms import Tab as KEY_TAB, ISO_Left_Tab
import traceback

from rafcon.statemachine.states.library_state import LibraryState
from rafcon.utils import log
logger = log.get_logger(__name__)
from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils import MoveAndEditTabUtilController
from rafcon.mvc.models.state import StateModel


class ScopedVariableListController(ExtendedController):

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.tab_edit_controller = MoveAndEditTabUtilController(view.get_top_widget())

        self.new_sv_counter = 0
        self.last_entry_widget = None
        self.next_focus_column = {}
        self.prev_focus_column = {}
        self.scoped_variables_list_store = ListStore(str, str, str, int)

    def register_view(self, view):
        """Called when the View was registered
        """

        view.get_top_widget().set_model(self.scoped_variables_list_store)

        view['name_col'].add_attribute(view['name_text'], 'text', 0)
        if not isinstance(self.model.state, LibraryState):
            view['name_text'].set_property("editable", True)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', 1)
        if not isinstance(self.model.state, LibraryState):
            view['data_type_text'].set_property("editable", True)
        if view['default_value_col'] and view['default_value_text']:
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', 2)
            if not isinstance(self.model.state, LibraryState):
                view['default_value_text'].set_property("editable", True)
            view['default_value_text'].connect("edited", self.on_default_value_changed)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)

        self.tab_edit_controller.register_view()

        if hasattr(self.model, 'scoped_variables'):
            self.reload_scoped_variables_list_store()

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_port)
        shortcut_manager.add_callback_for_action("add", self.add_port)

    def add_port(self, *_):
        if self.view[self.view.top].has_focus():
            self.on_new_scoped_variable_button_clicked(None)

    def remove_port(self, *_):
        if self.view[self.view.top].has_focus():
            self.on_delete_scoped_variable_button_clicked(None)

    @ExtendedController.observe("scoped_variables", after=True)
    def scoped_variables_changed(self, model, prop_name, info):
        # store port selection
        model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
        selected_data_port_id = None
        if len(self.scoped_variables_list_store) > 0 and path_list:
            selected_data_port_id = self.scoped_variables_list_store[path_list[0][0]][3]
        self.reload_scoped_variables_list_store()
        # recover port selection
        if selected_data_port_id is not None:
            self.select_entry(selected_data_port_id)

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        new_sv_name = "scoped_%s" % str(self.new_sv_counter)
        if hasattr(self.model, 'states'):
            self.new_sv_counter += 1
            self.model.state.add_scoped_variable(new_sv_name, "int", 0)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view["scoped_variables_tree_view"]
        if hasattr(self.model, 'states'):
            path = tree_view.get_cursor()[0][0]
            if path is not None:
                scoped_variable_key = self.scoped_variables_list_store[int(path)][3]
                self.scoped_variables_list_store.clear()
                self.model.state.remove_scoped_variable(scoped_variable_key)

    def on_name_changed(self, widget, path, text):
        scoped_variable_id = self.scoped_variables_list_store[int(path)][3]
        try:
            self.model.state.scoped_variables[scoped_variable_id].name = text
        except TypeError as e:
            logger.error("Error while changing port name: {0}".format(e))

    def on_data_type_changed(self, widget, path, text):
        data_port_id = self.scoped_variables_list_store[int(path)][3]
        try:
            self.model.state.scoped_variables[data_port_id].change_data_type(text, None)
        except ValueError as e:
            logger.error("Error while changing data type: {0}".format(e))

    def on_default_value_changed(self, widget, path, text):
        data_port_id = self.scoped_variables_list_store[int(path)][3]
        try:
            self.model.state.scoped_variables[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))

    def select_entry(self, data_port_id):
        """Selects the port entry belonging to the given data_port_id"""
        ctr = 0
        for data_port_entry in self.scoped_variables_list_store:
            # Compare transition ids
            if data_port_entry[3] == data_port_id:
                self.view[self.view.top].set_cursor(ctr)
                break
            ctr += 1

    def reload_scoped_variables_list_store(self):
        """Reloads the scoped variable list store from the data port models
        """
        if hasattr(self.model, 'scoped_variables'):
            tmp = ListStore(str, str, str, int)
            for sv_model in self.model.scoped_variables:
                data_type = sv_model.scoped_variable.data_type
                # get name of type (e.g. ndarray)
                data_type_name = data_type.__name__
                # get module of type, e.g. numpy
                data_type_module = data_type.__module__
                # if the type is not a builtin type, also show the module
                if data_type_module != '__builtin__':
                    data_type_name = data_type_module + '.' + data_type_name
                tmp.append([sv_model.scoped_variable.name, data_type_name,
                            sv_model.scoped_variable.default_value, sv_model.scoped_variable.data_port_id])
            tms = gtk.TreeModelSort(tmp)
            tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
            tms.set_sort_func(0, StateModel.dataport_compare_method)
            tms.sort_column_changed()
            tmp = tms
            self.scoped_variables_list_store.clear()
            for elem in tmp:
                self.scoped_variables_list_store.append(elem)
        else:
            raise RuntimeError("The reload_scoped_variables_list_store function should be never called for "
                               "a non Container State Model")
