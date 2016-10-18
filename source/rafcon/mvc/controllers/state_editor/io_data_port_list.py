"""
.. module:: io_data_port_list
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to list and edit all input- and output-data-ports of a state.

.. moduleauthor:: Sebastian Brunner


"""

import gtk
import gobject
from gtk import ListStore
from gtk import TreeViewColumn, CellRendererToggle

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.controllers.utils.tab_key import MoveAndEditWithTabKeyListFeatureController
from rafcon.mvc.models.abstract_state import AbstractStateModel

from rafcon.mvc.gui_helper import react_to_event
from rafcon.mvc.utils.comparison import compare_variables
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

        if self.type == "input":
            self.state_data_port_dict = self.model.state.input_data_ports
            self.data_port_model_list = self.model.input_data_ports
        elif self.type == "output":
            self.state_data_port_dict = self.model.state.output_data_ports
            self.data_port_model_list = self.model.output_data_ports

        self._actual_entry = None

        # variables to avoid to create and to be robust against chained notification calls
        self._do_name_change = False
        self._do_type_change = False
        self._do_value_change = False
        self._do_store_update = False
        self._do_selection_update = False

        self.data_port_list_store = self.get_new_list_store()
        self.reload_data_port_list_store()
        if self.model.get_sm_m_for_state_m() is not None:
            self.observe_model(self.model.get_sm_m_for_state_m())
        else:
            logger.warning("State model has no state machine model -> state model: {0}".format(self.model))

    def get_new_list_store(self):
        if not isinstance(self.model.state, LibraryState):
            return ListStore(str, str, str, int, gobject.TYPE_PYOBJECT)
        else:
            return ListStore(str, str, str, int, bool, str, gobject.TYPE_PYOBJECT)

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
            view['default_value_text'].connect('editing-started', self.editing_started)
            view['default_value_text'].connect('editing-canceled', self.editing_canceled)
            if isinstance(self.model.state, LibraryState):
                view['default_value_col'].set_title("Used value")
            view['default_value_col'].set_cell_data_func(view['default_value_text'], self.default_value_renderer)

        view['name_text'].connect("edited", self.on_name_changed)
        view['name_text'].connect('editing-started', self.editing_started)
        view['name_text'].connect('editing-canceled', self.editing_canceled)
        view['data_type_text'].connect("edited", self.on_data_type_changed)
        view['data_type_text'].connect('editing-started', self.editing_started)
        view['data_type_text'].connect('editing-canceled', self.editing_canceled)
        view.get_top_widget().connect('button_press_event', self.mouse_click)
        view.get_top_widget().get_selection().connect('changed', self.selection_changed)

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
        selection = view.get_top_widget().get_selection()
        selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.update_selection_sm_prior()

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
        """Callback method for add action
        """
        if react_to_event(self.view, self.view[self.view.top], event) and not isinstance(self.model.state, LibraryState):
            self.on_new_port_button_clicked(None)
            return True

    def remove_port(self, *event):
        """Callback method for remove action
        """
        if react_to_event(self.view, self.view[self.view.top], event) and not isinstance(self.model.state, LibraryState):
            self.on_delete_port_button_clicked(None)
            return True

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
        """ Change-value-method to set the default_value or runtime_value of actual selected (row) data-port.
        """
        # logger.info("FOCUS_OUT VALUE entry: {0} event: {1}".format(entry, event))
        if self.get_data_port_id_from_selection() is None:
            return

        self.on_default_value_changed(entry, None, text=entry.get_text())

    def mouse_click(self, widget, event=None):

        # if event.type == gtk.gdk.BUTTON_PRESS:
        #     pthinfo = self.view.get_top_widget().get_path_at_pos(int(event.x), int(event.y))
        #     if pthinfo is None:
        #         # logger.info("deselect rows")
        #         tree_selection = self.view.get_top_widget().get_selection()
        #         for path in tree_selection.get_selected_rows()[1]:
        #             tree_selection.unselect_path(path)

        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            pthinfo = self.view.get_top_widget().get_path_at_pos(int(event.x), int(event.y))
            if pthinfo is not None:
                logger.debug("do right click menu")
                self.view.get_top_widget().grab_focus()
                return True

    def get_selections(self):
        sm_selection = self.model.get_sm_m_for_state_m().selection
        sm_selected_port_m_list = None
        if self.type == 'input':
            sm_selected_port_m_list = sm_selection.input_data_ports
        elif self.type == 'output':
            sm_selected_port_m_list = sm_selection.output_data_ports

        tree_selection = self.view.get_top_widget().get_selection()
        model, paths = tree_selection.get_selected_rows()
        selected_port_m_list = []
        for row_num, path in enumerate(paths):
            if not isinstance(self.model.state, LibraryState):
                port_m = self.data_port_list_store[path[0]][4]
            else:
                port_m = self.data_port_list_store[path[0]][6]
            selected_port_m_list.append(port_m)
        return selected_port_m_list, tree_selection, sm_selected_port_m_list, sm_selection

    def update_selection_sm_prior(self):
        if self._do_selection_update:
            return
        self._do_selection_update = True
        selected_port_m_list, tree_selection, sm_selected_port_m_list, sm_selection = self.get_selections()
        for path, port_row in enumerate(self.data_port_list_store):
            port_m = port_row[4] if not isinstance(self.model.state, LibraryState) else port_row[6]
            if port_m not in sm_selected_port_m_list and port_m in selected_port_m_list:
                # print type(self).__name__, self.type, "sm un-select port_m", port_m
                tree_selection.unselect_path((path,))
            if port_m in sm_selected_port_m_list and port_m not in selected_port_m_list:
                # print type(self).__name__, self.type, "sm select port_m", port_m
                tree_selection.select_path((path,))

            # else:
            #     if port_m in sm_selected_port_m_list and port_m in selected_port_m_list:
            #         print type(self).__name__, self.type, "sm is selected"
        selected_port_m_list, tree_selection, sm_selected_port_m_list, sm_selection = self.get_selections()
        # if self.type == 'input':
        #     print type(self).__name__, self.type, sm_selection.get_num_input_data_ports(), sm_selection.get_input_data_ports(), len(selected_port_m_list), selected_port_m_list
        # elif self.type == 'output':
        #     print type(self).__name__, self.type, sm_selection.get_num_output_data_ports(), sm_selection.get_output_data_ports(), len(selected_port_m_list), selected_port_m_list
        self._do_selection_update = False

    def update_selection_self_prior(self):
        if self._do_selection_update:
            return
        self._do_selection_update = True
        selected_port_m_list, tree_selection, sm_selected_port_m_list, sm_selection = self.get_selections()
        for port_row in self.data_port_list_store:
            port_m = port_row[4] if not isinstance(self.model.state, LibraryState) else port_row[6]
            if port_m in sm_selected_port_m_list and port_m not in selected_port_m_list:
               sm_selection.remove(port_m)
            if port_m not in sm_selected_port_m_list and port_m in selected_port_m_list:
                print type(self).__name__, self.type, "select port_m", port_m
                sm_selection.add(port_m)

            # else:
            #     if port_m in sm_selected_port_m_list and port_m in selected_port_m_list:
            #         print type(self).__name__, self.type, "is selected"
        # if self.type == 'input':
        #     print type(self).__name__, self.type, sm_selection.get_num_input_data_ports(), sm_selection.get_input_data_ports()
        # elif self.type == 'output':
        #     print type(self).__name__, self.type, sm_selection.get_num_output_data_ports(), sm_selection.get_output_data_ports()
        self._do_selection_update = False

    def selection_changed(self, widget, event=None):
        # print type(self).__name__, self.type, "select changed"
        self.update_selection_self_prior()

    @ExtendedController.observe("selection", after=True)
    def state_machine_selection_changed(self, model, prop_name, info):
        # print model, prop_name, info
        if "{}_data_ports".format(self.type) == info['method_name']:
            self.update_selection_sm_prior()

    @ExtendedController.observe("input_data_ports", after=True)
    @ExtendedController.observe("output_data_ports", after=True)
    def data_ports_changed(self, model, prop_name, info):
        """Reload list store and reminds selection when the model was changed"""
        if "{}_data_ports".format(self.type) == prop_name and isinstance(model, AbstractStateModel):
            # store port selection
            path_list = None
            if self.view is not None:
                model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
            selected_data_port_ids = [self.data_port_list_store[path[0]][3] for path in path_list] if path_list else []
            self.reload_data_port_list_store()
            # recover port selection
            if selected_data_port_ids:
                [self.select_entry(selected_data_port_id, False) for selected_data_port_id in selected_data_port_ids]

    @ExtendedController.observe("state", after=True)
    def runtime_values_changed(self, model, prop_name, info):
        """Handle cases for the library runtime values"""
        if ("_{}_runtime_value".format(self.type) in info.method_name or
                info.method_name in ['use_runtime_value_{}_data_ports'.format(self.type),
                                     '{}_data_port_runtime_values'.format(self.type)]) and \
                self.model is model:
            self.data_ports_changed(model, "{}_data_ports".format(self.type), info)

    def on_new_port_button_clicked(self, widget, data=None):
        """Add a new port with default values and select it"""

        if self.type == "input":
            num_data_ports = len(self.model.state.input_data_ports)
        else:
            num_data_ports = len(self.model.state.output_data_ports)
        for run_id in range(num_data_ports + 1, 0, -1):
            new_port_name = self.type + "_{0}".format(run_id)
            try:
                if self.type == "input":
                    data_port_id = self.model.state.add_input_data_port(new_port_name, "int", "0")
                else:
                    data_port_id = self.model.state.add_output_data_port(new_port_name, "int", "0")
                break
            except ValueError as e:
                if run_id == num_data_ports:
                    logger.warn("The {1} data port couldn't be added: {0}".format(e, self.type))
                    return False
        self.select_entry(data_port_id)
        return True

    def on_delete_port_button_clicked(self, widget, data=None):
        """Delete the selected port and select the next one"""
        path_list = None
        if self.view is not None:
            model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
        data_port_ids = [self.data_port_list_store[path[0]][3] for path in path_list] if path_list else []
        if data_port_ids:
            for data_port_id in data_port_ids:
                if self.type == "input":
                    self.model.state.remove_input_data_port(data_port_id)
                else:
                    self.model.state.remove_output_data_port(data_port_id)
            if len(self.data_port_list_store) > 0:
                self.view[self.view.top].set_cursor(min(path[0], len(self.data_port_list_store) - 1))
            return True

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

    def select_entry(self, data_port_id, by_cursor=True):
        """Selects the port entry belonging to the given data_port_id"""
        for row_num, data_port_entry in enumerate(self.data_port_list_store):
            # Compare data port ids
            if data_port_entry[3] == data_port_id:
                if by_cursor:
                    self.view[self.view.top].set_cursor(row_num)
                else:
                    self.view[self.view.top].get_selection().select_path((row_num, ))
                break

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
        if self._do_name_change:
            return
        self._do_name_change = True
        try:
            data_port_id = self.get_data_port_id_from_selection()
            if self.state_data_port_dict[data_port_id].name != text:
                self.state_data_port_dict[data_port_id].name = text
        except (TypeError, ValueError) as e:
            logger.error("Error while trying to change the port name: {0}".format(e))
        self._do_name_change = False

    def on_data_type_changed(self, widget, column_id, text):
        """Try to set the port type the the newly entered one
        """
        # logger.info("on_data_type_changed widget: {0} path: {1} text: {2}".format(widget, column_id, text))
        if self._do_type_change:
            return
        self._do_type_change = True
        try:
            data_port_id = self.get_data_port_id_from_selection()
            if self.state_data_port_dict[data_port_id].data_type.__name__ != text:
                self.state_data_port_dict[data_port_id].change_data_type(text)
        except ValueError as e:
            logger.error("Error while changing data type: {0}".format(e))
        self._do_type_change = False

    def on_default_value_changed(self, widget, column_id, text):
        """Try to set the port default value to the newly entered one
        """
        # logger.info("on_default_value_changed widget: {0} path: {1} text: {2}".format(widget, column_id, text))
        if self._do_value_change:
            return
        self._do_value_change = True
        try:
            data_port_id = self.get_data_port_id_from_selection()
            if isinstance(self.model.state, LibraryState):
                # this always have to be true, as the runtime value column can only be edited
                # if the use_runtime_value flag is True
                if self.get_use_runtime_value_from_selection():
                    if self.type == "input":
                        if str(self.model.state.input_data_port_runtime_values[data_port_id]) != text:
                            self.model.state.set_input_runtime_value(data_port_id, text)
                    else:
                        if str(self.model.state.output_data_port_runtime_values[data_port_id]) != text:
                            self.model.state.set_output_runtime_value(data_port_id, text)
            else:
                if str(self.state_data_port_dict[data_port_id].default_value) != text:
                    self.state_data_port_dict[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))
        self._do_value_change = False

    def reload_data_port_list_store(self):
        """Reloads the input data port list store from the data port models"""

        tmp = self.get_new_list_store()
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
                default_value = "None"
            else:
                default_value = idp_model.data_port.default_value

            if not isinstance(self.model.state, LibraryState):
                tmp.append([idp_model.data_port.name, data_type_name, default_value, idp_model.data_port.data_port_id,
                            idp_model])
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
                            runtime_value,
                            idp_model,
                            ])

        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, compare_variables)
        tms.sort_column_changed()
        tmp = tms
        if self._do_store_update:
            return
        self._do_store_update = True
        try:
            self.data_port_list_store.clear()
            for elem in tmp:
                self.data_port_list_store.append(elem)
        except:
            pass
        self._do_store_update = False
