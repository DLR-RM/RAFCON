import gtk
from gtk import ListStore
from gtk import TreeViewColumn, CellRendererToggle

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils import MoveAndEditWithTabKeyListFeatureController
from rafcon.utils.comparison import compare_variables

from rafcon.utils import log
logger = log.get_logger(__name__)


class DataPortListController(ExtendedController):
    """Controller handling the input and output Data Port List

    :param rafcon.mvc.models.
    """

    def __init__(self, model, view, io_type):
        """Constructor"""
        ExtendedController.__init__(self, model, view)
        self.tab_edit_controller = MoveAndEditWithTabKeyListFeatureController(view.get_top_widget())
        self.type = io_type
        self.state_data_port_dict = None
        self.data_port_list_store = None

        self.new_port_counter = 0

        if self.type == "input":
            self.state_data_port_dict = self.model.state.input_data_ports
            self.data_port_model_list = self.model.input_data_ports
        elif self.type == "output":
            self.state_data_port_dict = self.model.state.output_data_ports
            self.data_port_model_list = self.model.output_data_ports

        if not isinstance(self.model.state, LibraryState):
            self.data_port_list_store = ListStore(str, str, str, int)
        else:
            self.data_port_list_store = ListStore(str, str, str, int, bool, str)
        self.reload_data_port_list_store()

    def default_value_renderer(self, tree_view_column, cell, model, iter):
        """

        :param tree_view_column: the gtk.TreeViewColumn to be rendered
        :param cell: the current CellRenderer
        :param model: the ListStore or TreeStore that is the model for TreeView
        :param iter: an iterator over the rows of the TreeStore/ListStore Model
        """
        if isinstance(self.model.state, LibraryState):
            use_runtime_value = model.get_value(iter, 4)
            if use_runtime_value:
                cell.set_property("editable", True)
                cell.set_property('text', model.get_value(iter, 5))
                cell.set_property('foreground', "white")
            else:
                cell.set_property("editable", False)
                cell.set_property('text', model.get_value(iter, 2))
                cell.set_property('foreground', "dark grey")

        return

    def register_view(self, view):
        """Called when the View was registered"""
        # top widget is a tree view => set the model of the tree view to be a list store
        view.get_top_widget().set_model(self.data_port_list_store)
        view.get_top_widget().set_cursor(0)

        view['name_col'].add_attribute(view['name_text'], 'text', 0)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', 1)
        if not isinstance(self.model.state, LibraryState):
            view['name_text'].set_property("editable", True)
            view['data_type_text'].set_property("editable", True)

        # in the linkage overview the the default value is not shown
        if view['default_value_col'] and view['default_value_text']:
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', 2)
            # if not isinstance(self.model.state, LibraryState):
            view['default_value_text'].set_property("editable", True)
            view['default_value_text'].connect("edited", self.on_default_value_changed)
            if isinstance(self.model.state, LibraryState):
                view['default_value_col'].set_title("Used value")
            view['default_value_col'].set_cell_data_func(view['default_value_text'], self.default_value_renderer)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)

        if isinstance(self.model.state, LibraryState):
            view['use_runtime_value_toggle'] = CellRendererToggle()
            view['use_runtime_value_col'] = TreeViewColumn("Use Runtime Value")
            view.get_top_widget().append_column(view['use_runtime_value_col'])
            view['use_runtime_value_col'].pack_start(view['use_runtime_value_toggle'], True)
            view['use_runtime_value_col'].add_attribute(view['use_runtime_value_toggle'], 'active', 4)
            view['use_runtime_value_toggle'].set_property("activatable", True)
            view['use_runtime_value_toggle'].connect("toggled", self.on_use_runtime_value_toggled)

            # view['runtime_value_text'] = CellRendererText()
            # view['runtime_value_col'] = TreeViewColumn("Runtime Value")
            # view.get_top_widget().append_column(view['runtime_value_col'])
            # view['runtime_value_col'].pack_start(view['runtime_value_text'], True)
            # view['runtime_value_col'].add_attribute(view['runtime_value_text'], 'text', 5)
            # view['runtime_value_text'].set_property("editable", True)
            # view['runtime_value_text'].connect("edited", self.on_runtime_value_edited)

        self.tab_edit_controller.register_view()

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_port)
        shortcut_manager.add_callback_for_action("add", self.add_port)

    def add_port(self, *_):
        """Callback method for add action
        """
        if self.view[self.view.top].has_focus():
            self.on_new_port_button_clicked(None)

    def remove_port(self, *_):
        """Callback method for remove action
        """
        if self.view[self.view.top].has_focus():
            self.on_delete_port_button_clicked(None)

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        """Reload list store and reminds selection when the model was changed"""
        if self.type == "input":
            # store port selection
            path_list = None
            selected_data_port_id = None
            if self.view is not None:
                model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
            if len(self.data_port_list_store) > 0 and path_list:
                selected_data_port_id = self.data_port_list_store[path_list[0][0]][3]
            self.reload_data_port_list_store()
            # recover port selection
            if selected_data_port_id is not None:
                self.select_entry(selected_data_port_id)

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        """Reload list store when the model was changed"""
        if self.type == "output":
            # store port selection
            path_list = None
            selected_data_port_id = None
            if self.view is not None:
                model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
            if len(self.data_port_list_store) > 0 and path_list:
                selected_data_port_id = self.data_port_list_store[path_list[0][0]][3]
            self.reload_data_port_list_store()
            # recover port selection
            if selected_data_port_id is not None:
                self.select_entry(selected_data_port_id)

    @ExtendedController.observe("state", after=True)
    def runtime_values_changed(self, model, prop_name, info):
        # handles cases for the library runtime values
        if "_input_runtime_value" in info.method_name and self.model is model:
            if self.type == "input":
                self.input_data_ports_changed(model, prop_name, info)
        elif "_output_runtime_value" in info.method_name and self.model is model:
            if self.type == "output":
                self.output_data_ports_changed(model, prop_name, info)

    def on_new_port_button_clicked(self, widget, data=None):
        """Add a new port with default values and select it"""
        new_port_name = self.type + "_{0}".format(self.new_port_counter)
        self.new_port_counter += 1
        if self.type == "input":
            data_port_id = self.model.state.add_input_data_port(new_port_name, "int", "0")
        else:
            data_port_id = self.model.state.add_output_data_port(new_port_name, "int", "0")
        self.select_entry(data_port_id)

    def on_delete_port_button_clicked(self, widget, data=None):
        """Delete the selected port and select the next one"""
        path = self.get_path()
        data_port_id = self.get_data_port_id_from_selection()
        if data_port_id is not None:
            if self.type == "input":
                self.model.state.remove_input_data_port(data_port_id)
            else:
                self.model.state.remove_output_data_port(data_port_id)
            if len(self.data_port_list_store) > 0:
                self.view[self.view.top].set_cursor(min(path, len(self.data_port_list_store) - 1))

    def get_data_port_id_from_selection(self):
        """Returns the data_port_id of the currently selected port entry"""
        path = self.get_path()
        if path is not None:
            data_port_id = self.data_port_list_store[int(path)][3]
            return data_port_id
        return None

    def get_use_runtime_value_from_selection(self):
        """Returns the use_runtime_value flag of the currently selected port entry"""
        path = self.get_path()
        if path is not None:
            use_runtime_value = self.data_port_list_store[int(path)][4]
            return use_runtime_value
        return None

    def select_entry(self, data_port_id):
        """Selects the port entry belonging to the given data_port_id"""
        ctr = 0
        for data_port_entry in self.data_port_list_store:
            # Compare transition ids
            if data_port_entry[3] == data_port_id:
                self.view[self.view.top].set_cursor(ctr)
                break
            ctr += 1

    def get_path(self):
        """Returns the path/index to the currently selected port entry"""
        cursor = self.view[self.view.top].get_cursor()
        # the cursor is a tuple containing the current path and the focused column
        if cursor[0] is None:
            return None
        return cursor[0][0]

    def on_use_runtime_value_toggled(self, widget, path):
        """Try to set the use runtime value flag to the newly entered one
        """
        # logger.info("on_use_runtime_value_edited widget: {0} path: {1}".format(widget, path))
        try:
            data_port_id = self.data_port_list_store[int(path)][3]
            if self.type == "input":
                current_value = self.model.state.use_runtime_value_input_data_ports[data_port_id]
                self.model.state.set_use_input_runtime_value(data_port_id, not current_value)
            else:
                current_value = self.model.state.use_runtime_value_output_data_ports[data_port_id]
                self.model.state.set_use_output_runtime_value(data_port_id, not current_value)
        except TypeError as e:
            logger.error("Error while trying to change the use_runtime_value flag: {0}".format(e))

    def on_name_changed(self, widget, column_id, text):
        """Try to set the port name to the newly entered one
        """
        # logger.info("on_name_changed widget: {0} path: {1} text: {2}".format(widget, column_id, text))
        try:
            data_port_id = self.get_data_port_id_from_selection()
            self.state_data_port_dict[data_port_id].name = text
        except TypeError as e:
            logger.error("Error while trying to change the port name: {0}".format(e))

    def on_data_type_changed(self, widget, column_id, text):
        """Try to set the port type the the newly entered one
        """
        # logger.info("on_data_type_changed widget: {0} path: {1} text: {2}".format(widget, column_id, text))
        try:
            data_port_id = self.get_data_port_id_from_selection()
            self.state_data_port_dict[data_port_id].change_data_type(text)
        except ValueError as e:
            logger.error("Error while changing data type: {0}".format(e))

    def on_default_value_changed(self, widget, column_id, text):
        """Try to set the port default value to the newly entered one
        """
        logger.info("on_default_value_changed widget: {0} path: {1} text: {2}".format(widget, column_id, text))
        try:
            data_port_id = self.get_data_port_id_from_selection()
            if isinstance(self.model.state, LibraryState):
                # this always have to be true, as the runtime value column can only be edited
                # if the use_runtime_value flag is True
                if self.get_use_runtime_value_from_selection():
                    if self.type == "input":
                        self.model.state.set_input_runtime_value(data_port_id, text)
                    else:
                        self.model.state.set_output_runtime_value(data_port_id, text)
            else:
                self.state_data_port_dict[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))

    def reload_data_port_list_store(self):
        """Reloads the input data port list store from the data port models"""
        if not isinstance(self.model.state, LibraryState):
            tmp = ListStore(str, str, str, int)
        else:
            tmp = ListStore(str, str, str, int, bool, str)
        for idp_model in self.data_port_model_list:
            data_type = idp_model.data_port.data_type
            # get name of type (e.g. ndarray)
            data_type_name = data_type.__name__
            # get module of type, e.g. numpy
            data_type_module = data_type.__module__
            # if the type is not a builtin type, also show the module
            if data_type_module != '__builtin__':
                data_type_name = data_type_module + '.' + data_type_name
            if idp_model.data_port.default_value is None:
                default_value = "<None>"
            else:
                default_value = idp_model.data_port.default_value

            if not isinstance(self.model.state, LibraryState):
                tmp.append([idp_model.data_port.name, data_type_name, default_value, idp_model.data_port.data_port_id])
            else:
                if self.type == "input":
                    use_runtime_value = self.model.state.use_runtime_value_input_data_ports[
                        idp_model.data_port.data_port_id]
                    runtime_value = self.model.state.input_data_port_runtime_values[idp_model.data_port.data_port_id]
                else:
                    use_runtime_value = self.model.state.use_runtime_value_output_data_ports[
                        idp_model.data_port.data_port_id]
                    runtime_value = self.model.state.output_data_port_runtime_values[idp_model.data_port.data_port_id]
                tmp.append([idp_model.data_port.name,
                            data_type_name,
                            default_value,
                            idp_model.data_port.data_port_id,
                            use_runtime_value,
                            runtime_value
                            ])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, compare_variables)
        tms.sort_column_changed()
        tmp = tms
        self.data_port_list_store.clear()
        for elem in tmp:
            self.data_port_list_store.append(elem)
