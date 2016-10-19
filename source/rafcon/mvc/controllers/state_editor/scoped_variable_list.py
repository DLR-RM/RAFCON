"""
.. module:: scoped_variable_list
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to list and edit all scoped_variables of a state.

.. moduleauthor:: Sebastian Brunner


"""

import gtk
import gobject

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils.tab_key import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.controllers.utils.selection import ListSelectionFeatureController
from rafcon.mvc.models.container_state import ContainerStateModel

from rafcon.mvc.gui_helper import react_to_event
from rafcon.mvc.utils.comparison import compare_variables
from rafcon.utils import log

logger = log.get_logger(__name__)


class ScopedVariableListController(ExtendedController, ListSelectionFeatureController):
    """Controller handling the scoped variable list

    :param rafcon.mvc.models.state.StateModel model: The state model, holding state data.
    :param rafcon.mvc.views.scoped_variables_list.ScopedVariablesListView view: The GTK view showing the list of scoped
        variables.
    """
    NAME_STORAGE_ID = 0
    DATA_TYPE_NAME_STORAGE_ID = 1
    DEFAULT_VALUE_STORAGE_ID = 2
    ID_STORAGE_ID = 3
    MODEL_STORAGE_ID = 4

    def __init__(self, model, view):
        """Constructor"""

        self.tree_view = view.get_top_widget()
        self.list_store = self.get_new_list_store()
        self._logger = logger

        ExtendedController.__init__(self, model, view)
        ListSelectionFeatureController.__init__(self, self.list_store, self.tree_view, logger)
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

        if self.model.get_sm_m_for_state_m() is not None:
            self.observe_model(self.model.get_sm_m_for_state_m())
            # print type(self).__name__, self.model.state.name, "initialized sm observation"
        else:
            logger.warning("State model has no state machine model -> state model: {0}".format(self.model))

        self.tree_view.set_model(self.list_store)

    def get_new_list_store(self):
        return gtk.ListStore(str, str, str, int, gobject.TYPE_PYOBJECT)

    def register_view(self, view):
        """Called when the View was registered"""

        view['name_col'].add_attribute(view['name_text'], 'text', self.NAME_STORAGE_ID)
        if not isinstance(self.model.state, LibraryState):
            view['name_text'].set_property("editable", True)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', self.DATA_TYPE_NAME_STORAGE_ID)
        if not isinstance(self.model.state, LibraryState):
            view['data_type_text'].set_property("editable", True)
        if view['default_value_col'] and view['default_value_text']:
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', self.DEFAULT_VALUE_STORAGE_ID)
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

        ListSelectionFeatureController.register_view(self, view)
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

    def add_port(self, *event):
        if react_to_event(self.view, self.view[self.view.top], event) and not isinstance(self.model.state, LibraryState):
            return self.on_new_scoped_variable_button_clicked(None)

    def remove_port(self, *event):
        if react_to_event(self.view, self.view[self.view.top], event) and not isinstance(self.model.state, LibraryState):
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
        if self.get_list_store_row_from_cursor_selection() is None:
            return

        self.on_name_changed(entry, None, text=entry.get_text())

    def change_data_type(self, entry, event):
        """ Change-data-type-method to set the data_type of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT TYPE entry: {0} event: {1}".format(entry, event))
        if self.get_list_store_row_from_cursor_selection() is None:
            return

        self.on_data_type_changed(entry, None, text=entry.get_text())

    def change_value(self, entry, event):
        """ Change-value-method to set the default_value of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT VALUE entry: {0} event: {1}".format(entry, event))
        if self.get_list_store_row_from_cursor_selection() is None:
            return

        self.on_default_value_changed(entry, None, text=entry.get_text())

    def get_state_machine_selection(self):
        # print type(self).__name__, "get state machine selection"
        sm_selection = self.model.get_sm_m_for_state_m().selection
        return sm_selection, sm_selection.scoped_variables

    @ExtendedController.observe("selection", after=True)
    def state_machine_selection_changed(self, model, prop_name, info):
        if "scoped_variables" == info['method_name']:
            self.update_selection_sm_prior()

    @ExtendedController.observe("scoped_variables", after=True)
    def scoped_variables_changed(self, model, prop_name, info):
        # store port selection
        path_list = None
        if self.view is not None:
            model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
        selected_data_port_ids = [self.list_store[path[0]][self.ID_STORAGE_ID] for path in path_list] if path_list else []
        self.reload_scoped_variables_list_store()
        # recover port selection
        if selected_data_port_ids:
            [self.select_entry(selected_data_port_id, False) for selected_data_port_id in selected_data_port_ids]

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
                scoped_variable_key = self.list_store[int(path)][self.ID_STORAGE_ID]
                self.list_store.clear()
                try:
                    self.model.state.remove_scoped_variable(scoped_variable_key)
                except AttributeError as e:
                    logger.warn("The scoped variable couldn't be removed: {0}".format(e))
                    return False
            if len(self.list_store) > 0:
                self.view[self.view.top].set_cursor(min(path, len(self.list_store) - 1))
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
        data_port_id = self.get_list_store_row_from_cursor_selection()[self.ID_STORAGE_ID]
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
        data_port_id = self.get_list_store_row_from_cursor_selection()[self.ID_STORAGE_ID]
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
        data_port_id = self.get_list_store_row_from_cursor_selection()[self.ID_STORAGE_ID]
        try:
            if str(self.model.state.scoped_variables[data_port_id].default_value) != text:
                self.model.state.scoped_variables[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))
        self._do_value_change = False

    def on_right_click_menu(self):
        logger.debug("do right click menu")

    def reload_scoped_variables_list_store(self):
        """Reloads the scoped variable list store from the data port models"""

        if isinstance(self.model, ContainerStateModel):
            tmp = self.get_new_list_store()
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
                            sv_model.scoped_variable.default_value, sv_model.scoped_variable.data_port_id, sv_model])
            tms = gtk.TreeModelSort(tmp)
            tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
            tms.set_sort_func(0, compare_variables)
            tms.sort_column_changed()
            tmp = tms
            if self._do_store_update:
                return
            self._do_store_update = True
            try:
                self.list_store.clear()
                for elem in tmp:
                    self.list_store.append(elem)
            except:
                pass
            self._do_store_update = False
        else:
            raise RuntimeError("The reload_scoped_variables_list_store function should be never called for "
                               "a non Container State Model")

