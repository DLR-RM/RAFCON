"""
.. module:: scoped_variable_list
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to list and edit all scoped_variables of a state.

.. moduleauthor:: Sebastian Brunner


"""

import gtk
from gtk import ListStore

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils.tab_key import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.models.container_state import ContainerStateModel
from rafcon.mvc.utils.comparison import compare_variables
from rafcon.utils import log

logger = log.get_logger(__name__)


class ScopedVariableListController(ExtendedController):
    """Controller handling the scoped variable list

    :param rafcon.mvc.models.state.StateModel model: The state model, holding state data.
    :param rafcon.mvc.views.scoped_variables_list.ScopedVariablesListView view: The GTK view showing the list of scoped
        variables.
    """

    def __init__(self, model, view):
        """Constructor"""
        ExtendedController.__init__(self, model, view)
        self.tab_edit_controller = MoveAndEditWithTabKeyListFeatureController(view.get_top_widget())

        self.last_entry_widget = None
        self._actual_entry = None
        self.next_focus_column = {}
        self.prev_focus_column = {}

        # variables to avoid to create and to be robust against chained notification calls
        self._do_name_change = False
        self._do_type_change = False
        self._do_value_change = False
        self._do_store_update = False

        self.scoped_variables_list_store = ListStore(str, str, str, int)

    def register_view(self, view):
        """Called when the View was registered"""
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
            view['default_value_text'].connect('editing-started', self.editing_started)
            view['default_value_text'].connect('editing-canceled', self.editing_canceled)

        view['name_text'].connect("edited", self.on_name_changed)
        view['name_text'].connect('editing-started', self.editing_started)
        view['name_text'].connect('editing-canceled', self.editing_canceled)
        view['data_type_text'].connect("edited", self.on_data_type_changed)
        view['data_type_text'].connect('editing-started', self.editing_started)
        view['data_type_text'].connect('editing-canceled', self.editing_canceled)

        self.tab_edit_controller.register_view()

        if isinstance(self.model, ContainerStateModel):
            self.reload_scoped_variables_list_store()

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        if not isinstance(self.model.state, LibraryState):
            shortcut_manager.add_callback_for_action("delete", self.remove_port)
            shortcut_manager.add_callback_for_action("add", self.add_port)

    def add_port(self, *_):
        if self.view and self.view[self.view.top].is_focus() and not isinstance(self.model.state, LibraryState):
            return self.on_new_scoped_variable_button_clicked(None)

    def remove_port(self, *_):
        if self.view and self.view[self.view.top].is_focus() and not isinstance(self.model.state, LibraryState):
            return self.on_delete_scoped_variable_button_clicked(None)

    def editing_started(self, renderer, editable, path):
        """ Callback method to connect entry-widget focus-out-event to the respective change-method.
        """
        # logger.info("CONNECT editable: {0} path: {1}".format(editable, path))
        if self.view['name_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_name))
        elif self.view['data_type_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_data_type))
        elif self.view['default_value_text'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_value))
        else:
            logger.error("Not registered Renderer was used")

    def editing_canceled(self, event):
        """ Callback method to disconnect entry-widget focus-out-event to the respective change-method.
        """
        # logger.info("DISCONNECT text: {1} event: {0}".format(event, event.get_property('text')))
        if self._actual_entry is not None:
            self._actual_entry[0].disconnect(self._actual_entry[1])
            self._actual_entry = None

    def change_name(self, entry, event):
        """ Change-name-method to set the name of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT NAME entry: {0} event: {1}".format(entry, event))
        if self.get_data_port_id_from_selection() is None:
            return

        self.on_name_changed(entry, None, text=entry.get_text())

    def change_data_type(self, entry, event):
        """ Change-data-type-method to set the data_type of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT TYPE entry: {0} event: {1}".format(entry, event))
        if self.get_data_port_id_from_selection() is None:
            return

        self.on_data_type_changed(entry, None, text=entry.get_text())

    def change_value(self, entry, event):
        """ Change-value-method to set the default_value of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT VALUE entry: {0} event: {1}".format(entry, event))
        if self.get_data_port_id_from_selection() is None:
            return

        self.on_default_value_changed(entry, None, text=entry.get_text())

    @ExtendedController.observe("scoped_variables", after=True)
    def scoped_variables_changed(self, model, prop_name, info):
        # store port selection
        path_list = None
        selected_data_port_id = None
        if self.view is not None:
            model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
        if len(self.scoped_variables_list_store) > 0 and path_list:
            selected_data_port_id = self.scoped_variables_list_store[path_list[0][0]][3]
        self.reload_scoped_variables_list_store()
        # recover port selection
        if selected_data_port_id is not None:
            self.select_entry(selected_data_port_id)

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        """Triggered when the New button in the Scoped Variables tab is clicked.

        Create a new scoped variable with default values.
        """
        num_data_ports = len(self.model.state.scoped_variables)
        if isinstance(self.model, ContainerStateModel):
            data_port_id = None
            for run_id in range(num_data_ports + 1, 0, -1):
                try:
                    data_port_id = self.model.state.add_scoped_variable("scoped_%s" % run_id, "int", 0)
                    break
                except ValueError as e:
                    if run_id == num_data_ports:
                        logger.warn("The scoped variable couldn't be added: {0}".format(e))
                        return False
            self.select_entry(data_port_id)
            return True

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        """Triggered when the Delete button in the Scoped Variables tab is clicked.

        Deletes the selected scoped variable.
        """
        if isinstance(self.model, ContainerStateModel):

            path = self.get_path()  # tree_view.get_cursor()[0][0]
            if path is not None:
                scoped_variable_key = self.scoped_variables_list_store[int(path)][3]
                self.scoped_variables_list_store.clear()
                try:
                    self.model.state.remove_scoped_variable(scoped_variable_key)
                except AttributeError as e:
                    logger.warn("The scoped variable couldn't be removed: {0}".format(e))
                    return False
            if len(self.scoped_variables_list_store) > 0:
                self.view[self.view.top].set_cursor(min(path, len(self.scoped_variables_list_store) - 1))
            return True

    def on_name_changed(self, widget, path, text):
        """Triggered when a scoped variable's name is edited

        Changes the scoped variable's name.

        :param path: The path identifying the edited variable
        :param text: New variable's name
        """
        if self._do_name_change:
            return
        self._do_name_change = True
        data_port_id = self.get_data_port_id_from_selection()
        try:
            if self.model.state.scoped_variables[data_port_id].name != text:
                self.model.state.scoped_variables[data_port_id].name = text
        except TypeError as e:
            logger.error("Error while changing port name: {0}".format(e))
        self._do_name_change = False

    def on_data_type_changed(self, widget, path, text):
        """Triggered when a scoped variable's data type is edited.

        Changes the scoped variable's data type.

        :param path: The path identifying the edited variable
        :param text: New variable's data type
        """
        if self._do_type_change:
            return
        self._do_type_change = True
        data_port_id = self.get_data_port_id_from_selection()
        try:
            if self.model.state.scoped_variables[data_port_id].data_type.__name__ != text:
                self.model.state.scoped_variables[data_port_id].change_data_type(text)
        except ValueError as e:
            logger.error("Error while changing data type: {0}".format(e))
        self._do_type_change = False

    def on_default_value_changed(self, widget, path, text):
        """Triggered when a scoped variable's value is edited.

        Changes the scoped variable's value.

        :param path: The path identifying the edited variable
        :param text: New variable's value
        """
        if self._do_value_change:
            return
        self._do_value_change = True
        data_port_id = self.get_data_port_id_from_selection()
        try:
            if str(self.model.state.scoped_variables[data_port_id].default_value) != text:
                self.model.state.scoped_variables[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))
        self._do_value_change = False

    def get_data_port_id_from_selection(self):
        """Returns the data_port_id of the currently selected port entry"""
        path = self.get_path()
        if path is not None:
            data_port_id = self.scoped_variables_list_store[int(path)][3]
            return data_port_id
        return None

    def select_entry(self, data_port_id):
        """Selects the port entry belonging to the given data_port_id"""
        for row_num, data_port_entry in enumerate(self.scoped_variables_list_store):
            # Compare transition ids
            if data_port_entry[3] == data_port_id:
                self.view[self.view.top].set_cursor(row_num)
                break

    def get_path(self):
        """Returns the path/index to the currently selected port entry"""
        cursor = self.view[self.view.top].get_cursor()
        # the cursor is a tuple containing the current path and the focused column
        if cursor[0] is None:
            return None
        return cursor[0][0]

    def reload_scoped_variables_list_store(self):
        """Reloads the scoped variable list store from the data port models"""

        if isinstance(self.model, ContainerStateModel):
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
            tms.set_sort_func(0, compare_variables)
            tms.sort_column_changed()
            tmp = tms
            if self._do_store_update:
                return
            self._do_store_update = True
            try:
                self.scoped_variables_list_store.clear()
                for elem in tmp:
                    self.scoped_variables_list_store.append(elem)
            except:
                pass
            self._do_store_update = False
        else:
            raise RuntimeError("The reload_scoped_variables_list_store function should be never called for "
                               "a non Container State Model")

