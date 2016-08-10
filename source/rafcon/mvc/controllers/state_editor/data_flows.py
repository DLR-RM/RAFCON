"""
.. module:: state_data_flows
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to list and edit all internal and related external data flows of a
     state.

.. moduleauthor:: Rico Belder


"""

import gobject
from gtk import ListStore, TreeStore

from rafcon.statemachine.state_elements.scope import ScopedVariable
from rafcon.statemachine.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.models.container_state import ContainerStateModel
from rafcon.mvc.utils.notification_overview import NotificationOverview

from rafcon.utils.constants import BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS, RAFCON_TEMP_PATH_BASE
from rafcon.utils import log, type_helpers

logger = log.get_logger(__name__)
PORT_TYPE_TAG = {InputDataPort: 'IP', OutputDataPort: 'OP', ScopedVariable: 'SV'}


class StateDataFlowsListController(ExtendedController):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.data_flow.DataFlowListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param rafcon.mvc.models.ContainerStateModel model: The container state model containing the data
    :param rafcon.mvc.views.DataFlowListView view: The GTK view showing the data flows as a table
    """

    free_to_port_internal = None
    free_to_port_external = None
    from_port_internal = None
    from_port_external = None

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        # TreeStore for: id, from-state, from-key, to-state, to-key, is_external,
        #                   name-color, to-state-color, data-flow-object, state-object, is_editable
        self.view_dict = {'data_flows_internal': True, 'data_flows_external': True}
        self.tree_store = TreeStore(int, str, str, str, str, bool,
                                    str, str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT, bool)

        view.get_top_widget().set_model(self.tree_store)

        self.tree_dict_combos = {'internal': {},
                                 'external': {}}
        self.data_flow_dict = {'internal': {},
                               'external': {}}
        self.no_update = False  # used to reduce the update cost of the widget (e.g while no focus or complex changes)
        self.no_update_state_destruction = False
        self.no_update_self_or_parent_state_destruction = False
        self.debug_log = False
        self._actual_overview = None

        # register other model and fill tree_store the model of the view
        if not model.state.is_root_state:
            self.observe_model(model.parent)

        self._update_internal_data_base()
        self._update_tree_store()

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            df_id = model.get_value(iter, 0)
            in_external = 'internal'
            if model.get_value(iter, 5):
                in_external = 'external'
            if column.get_title() == 'Source State':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['from_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Source Port':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['from_key'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Target State':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['to_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Target Port':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['to_key'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            else:
                logger.warning("Column has no cell_data_func %s %s" % (column.get_name(), column.get_title()))

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_key_col'].set_cell_data_func(view['from_key_combo'], cell_text, self.model)
        view['to_key_col'].set_cell_data_func(view['to_key_combo'], cell_text, self.model)

        view['from_state_combo'].connect("edited", self.on_combo_changed_from_state)
        view['from_key_combo'].connect("edited", self.on_combo_changed_from_key)
        view['to_state_combo'].connect("edited", self.on_combo_changed_to_state)
        view['to_key_combo'].connect("edited", self.on_combo_changed_to_key)
        # view['external_toggle'].connect("edited", self.on_external_toggled)
        view.tree_view.connect("grab-focus", self.on_focus)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_focus(self, widget, data=None):
        path = self.view.tree_view.get_cursor()
        # logger.debug("DATAFLOWS_LIST get new FOCUS %s" % str(path[0]))
        self._update_internal_data_base()
        self._update_tree_store()
        if path[0]:
            self.view.tree_view.set_cursor(path[0])

    def find_free_and_valid_data_flows(self, depend_to_state_id=None):
        # print "\n internal from %s \n\n internal to %s" % (self.free_to_port_internal, self.from_port_internal)
        internal_data_flows = []
        if self.free_to_port_internal and self.from_port_internal:
            for from_state_id, elems in self.from_port_internal.iteritems():
                # print "\n\nfrom_state %s and ports %s" % (from_state_id, [(elem.name, elem.data_type) for elem in elems])
                for from_port in elems:
                    for to_state_id, elems in self.free_to_port_internal.iteritems():
                        # print "\nto_state %s and ports %s" % (to_state_id, [(elem.name, elem.data_type) for elem in elems])
                        for to_port in elems:
                            if type_helpers.type_inherits_of_type(from_port.data_type, to_port.data_type):
                                if depend_to_state_id is None or depend_to_state_id in [from_state_id, to_state_id]:
                                    internal_data_flows.append((from_state_id, from_port.data_port_id,
                                                                to_state_id, to_port.data_port_id))

        # print "\n\n\n" + 60*"-" + "\n external from %s \n\n external to %s" % (self.free_to_port_external, self.from_port_external)
        external_data_flows = []
        if self.free_to_port_external and self.from_port_external:
            for from_state_id, elems in self.from_port_external.iteritems():
                # print "\n\nfrom_state %s and ports %s" % (from_state_id, [(elem.name, elem.data_type) for elem in elems])
                for from_port in elems:
                    for to_state_id, elems in self.free_to_port_external.iteritems():
                        # print "\nto_state %s and ports %s" % (to_state_id, [(elem.name, elem.data_type) for elem in elems])
                        for to_port in elems:
                            if type_helpers.type_inherits_of_type(from_port.data_type, to_port.data_type):
                                if depend_to_state_id is None or depend_to_state_id in [from_state_id, to_state_id]:
                                    external_data_flows.append((from_state_id, from_port.data_port_id,
                                                                to_state_id, to_port.data_port_id))

        return internal_data_flows, external_data_flows

    def on_add(self, button, info=None):
        # print "ADD DATA_FLOW"
        own_state_id = self.model.state.state_id
        [possible_internal_data_flows, possible_external_data_flows] = self.find_free_and_valid_data_flows(own_state_id)
        # print "\n\npossible internal data_flows\n %s" % possible_internal_data_flows
        # print "\n\npossible external data_flows\n %s" % possible_external_data_flows

        from_key = None
        if self.view_dict['data_flows_internal'] and possible_internal_data_flows:
            # print self.from_port_internal
            from_state_id = possible_internal_data_flows[0][0]
            from_key = possible_internal_data_flows[0][1]
            # print from_state_id, from_key, self.model.state.state_id
            to_state_id = possible_internal_data_flows[0][2]
            to_key = possible_internal_data_flows[0][3]
            # print "NEW DATA_FLOW INTERNAL IS: ", from_state_id, from_key, to_state_id, to_key
            try:
                data_flow_id = self.model.state.add_data_flow(from_state_id, from_key, to_state_id, to_key)
                # print "NEW DATA_FLOW INTERNAL IS: ", self.model.state.data_flows[data_flow_id]
            except (AttributeError, ValueError) as e:
                logger.error("Data Flow couldn't be added: {0}".format(e))
                return
        elif self.view_dict['data_flows_external'] and possible_external_data_flows:  # self.free_to_port_external:
            from_state_id = possible_external_data_flows[0][0]
            # print from_state_id, self.model.state.output_data_ports
            from_key = possible_external_data_flows[0][1]
            to_state_id = possible_external_data_flows[0][2]
            to_key = possible_external_data_flows[0][3]
            # print "NEW DATA_FLOW EXTERNAL IS: ", from_state_id, from_key, to_state_id, to_key, \
            #     get_state_model(self.model.parent, to_state_id).state.get_data_port_by_id(to_key)
            try:
                data_flow_id = self.model.parent.state.add_data_flow(from_state_id, from_key, to_state_id, to_key)
                # print "NEW DATA_FLOW EXTERNAL IS: ", self.model.parent.state.data_flows[data_flow_id]
            except (AttributeError, ValueError) as e:
                logger.error("Data Flow couldn't be added: {0}".format(e))
                return
        else:
            logger.warning("NO OPTION TO ADD DATA FLOW")
            return

        # set focus on this new element
        # - at the moment every new element is the last -> easy work around :(
        self.view.tree_view.set_cursor(len(self.tree_store) - 1)
        return True

    def on_remove(self, button, info=None):
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        if path:
            try:
                if self.tree_store[path[0][0]][5]:
                    self.model.parent.state.remove_data_flow(self.tree_store[path[0][0]][0])
                else:
                    self.model.state.remove_data_flow(self.tree_store[path[0][0]][0])
            except (AttributeError, ValueError) as e:
                logger.error("Data Flow couldn't be removed: {0}".format(e))
                return
        else:
            logger.warning("Please select the data flow to be deleted")
            return

        # selection to next element
        row_number = path[0][0]
        if len(self.tree_store) > 0:
            self.view.tree_view.set_cursor(min(row_number, len(self.tree_store) - 1))
        return True

    def on_combo_changed_from_state(self, widget, path, text):
        if text is None or self.tree_store[path][1] == text:
            return
        text = text.split('.')
        new_from_state_id = text[-1]
        data_flow_id = self.tree_store[path][0]
        is_external_data_flow = self.tree_store[path][5]
        new_from_data_port_id = None  # df.from_key
        if is_external_data_flow:
            new_from_data_port_id = self.from_port_external[new_from_state_id][0].data_port_id
            data_flow_parent_state = self.model.parent.state
        else:
            data_flow_parent_state = self.model.state
            state_m = get_state_model(self.model, new_from_state_id)
            # Find arbitrary origin data port in origin state
            if state_m is self.model:  # Data flow origin is parent state
                if len(state_m.input_data_ports) > 0:
                    new_from_data_port_id = state_m.input_data_ports[0].data_port.data_port_id
                elif len(state_m.scoped_variables) > 0:
                    new_from_data_port_id = state_m.scoped_variables[0].scoped_variable.data_port_id
            else:  # Data flow origin is child state
                if len(state_m.output_data_ports) > 0:
                    new_from_data_port_id = state_m.output_data_ports[0].data_port.data_port_id

        if new_from_data_port_id is None:
            logger.error("Could not change from state: No data port for data flow found")
            return
        try:
            data_flow_parent_state.data_flows[data_flow_id].modify_origin(new_from_state_id, new_from_data_port_id)
        except ValueError as e:
            logger.error("Could not change from state: {0}".format(e))

    def on_combo_changed_from_key(self, widget, path, text):
        if text is None:
            return
        new_from_data_port_id = int(text.split('.#')[-1].split('.')[0])
        data_flow_id = self.tree_store[path][0]
        is_external_data_flow = self.tree_store[path][5]
        if is_external_data_flow:
            data_flow_parent_state = self.model.parent.state
        else:
            data_flow_parent_state = self.model.state
        if new_from_data_port_id == data_flow_parent_state.data_flows[data_flow_id].from_key:
            return

        try:
            data_flow_parent_state.data_flows[data_flow_id].from_key = new_from_data_port_id
        except ValueError as e:
            logger.error("Could not change from data port: {0}".format(e))

    def on_combo_changed_to_state(self, widget, path, text):
        if text is None or self.tree_store[path][3] == text:
            return
        text = text.split('.')
        new_to_state_id = text[-1]
        data_flow_id = self.tree_store[path][0]
        is_external_data_flow = self.tree_store[path][5]
        if is_external_data_flow:
            data_flow_parent_state = self.model.parent.state
            new_to_data_port_id = self.free_to_port_external[new_to_state_id][0].data_port_id
        else:
            data_flow_parent_state = self.model.state
            new_to_data_port_id = self.free_to_port_internal[new_to_state_id][0].data_port_id
        try:
            data_flow_parent_state.data_flows[data_flow_id].modify_target(new_to_state_id, new_to_data_port_id)
        except ValueError as e:
            logger.error("Could not change to state: {0}".format(e))

    def on_combo_changed_to_key(self, widget, path, text):
        if text is None:
            return
        new_to_data_port_id = int(text.split('.#')[-1].split('.')[0])
        data_flow_id = self.tree_store[path][0]
        is_external_data_flow = self.tree_store[path][5]
        if is_external_data_flow:
            data_flow_parent_state = self.model.parent.state
        else:
            data_flow_parent_state = self.model.state

        if new_to_data_port_id == data_flow_parent_state.data_flows[data_flow_id].to_key:
            return

        try:
            data_flow_parent_state.data_flows[data_flow_id].to_key = new_to_data_port_id
        except ValueError as e:
            logger.error("Could not change to data port: {0}".format(e))

    def update(self):
        self._update_internal_data_base()
        self._update_tree_store()

    def _update_internal_data_base(self):
        [free_to_int, free_to_ext, from_int, from_ext] = update_data_flows(self.model, self.data_flow_dict,
                                                                          self.tree_dict_combos)
        self.free_to_port_internal = free_to_int
        self.free_to_port_external = free_to_ext
        self.from_port_internal = from_int
        self.from_port_external = from_ext

    def _update_tree_store(self):
        self.tree_store.clear()

        if self.view_dict['data_flows_internal'] and isinstance(self.model, ContainerStateModel):
            for data_flow in self.model.state.data_flows.values():

                # print "type: ", type(data_flow)
                if data_flow.data_flow_id in self.data_flow_dict['internal'].keys():
                    df_dict = self.data_flow_dict['internal'][data_flow.data_flow_id]
                    # TreeStore for: id, from-state, from-key, to-state, to-key, is_external,
                    #       name-color, to-state-color, data-flow-object, state-object, is_editable
                    # print 'insert int: ', data_flow.data_flow_id, df_dict['from_state'], df_dict['from_key'], \
                    #     df_dict['to_state'], df_dict['to_key']
                    self.tree_store.append(None, [data_flow.data_flow_id,
                                                  df_dict['from_state'],
                                                  df_dict['from_key'],
                                                  df_dict['to_state'],
                                                  df_dict['to_key'],
                                                  False,
                                                  '#f0E5C7', '#f0E5c7', data_flow, self.model.state, True])

        if self.view_dict['data_flows_external'] and not self.model.state.is_root_state:
            for data_flow in self.model.parent.state.data_flows.values():
                # data_flow = row[0]
                if data_flow.data_flow_id in self.data_flow_dict['external'].keys():
                    df_dict = self.data_flow_dict['external'][data_flow.data_flow_id]
                    # TreeStore for: id, from-state, from-key, to-state, to-key, is_external,
                    #       name-color, to-state-color, data-flow-object, state-object, is_editable
                    # print 'insert ext: ', data_flow.data_flow_id, df_dict['from_state'], df_dict['from_key'], \
                    #     df_dict['to_state'], df_dict['to_key']
                    self.tree_store.append(None, [data_flow.data_flow_id,
                                                  df_dict['from_state'],
                                                  df_dict['from_key'],
                                                  df_dict['to_state'],
                                                  df_dict['to_key'],
                                                  True,
                                                  '#f0E5C7', '#f0E5c7', data_flow, self.model.state, True])

    @ExtendedController.observe("root_state", assigned=True)
    def root_state_changed(self, model, prop_name, info):
        """ Relieve all observed models to avoid updates on old root state.
        """
        # TODO may re-observe if the states-editor supports this feature
        self.relieve_all_models()

    @ExtendedController.observe("state", before=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        """ Set the no update flag to avoid updates in between of a state removal.
        """
        if info['method_name'] == "remove_state":
            if info.instance.state_id == self.model.state.state_id:
                self.no_update_state_destruction = True
                # print "NOUPDATE ", self.no_update_state_destruction, self.model.state.state_id
            else:
                if info.args[1] == self.model.state.state_id or \
                        not self.model.state.is_root_state and info.args[1] == self.model.parent.state.state_id:
                    self.no_update_self_or_parent_state_destruction = True
                    self.relieve_all_models()
                # print "DNOUPDATE_PARENT ", self.no_update_self_or_parent_state_destruction, info.args[1], self.model.state.state_id

    @ExtendedController.observe("change_state_type", before=True)
    @ExtendedController.observe("change_root_state_type", before=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        """ Set the no update flag to avoid updates in between of a state-type-change.
        """
        self.no_update = True

    @ExtendedController.observe("state", after=True)
    def after_notification_state(self, model, prop_name, info):
        # The method causing the change raised an exception, thus nothing was changed
        # avoid updates because of execution status updates
        if info['method_name'] == "remove_state":
            if info.instance.state_id == self.model.state.state_id:
                self.no_update_state_destruction = True
                # print "DNOUPDATE ", self.no_update_state_destruction, self.model.state.state_id
            # TODO introduce re-observe new objects to use commented code
            # else:
            #     if info.args[1] == self.model.state.state_id or \
            #             not self.model.state.is_root_state and info.args[1] == self.model.parent.state.state_id:
            #         self.no_update_self_or_parent_state_destruction = False
            #     # print "DNOUPDATE_PARENT ", self.no_update_self_or_parent_state_destruction, info.args[1], self.model.state.state_id

        if info['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return

        overview = NotificationOverview(info, False, self.__class__.__name__)
        self._actual_overview = overview
        # if isinstance(overview['result'][-1], str) and "CRASH" in overview['result'][-1] or \
        #         isinstance(overview['result'][-1], Exception):
        #     return
        if overview['method_name'][-1] == 'parent' and overview['instance'][-1] is self.model.state:
            self.update()
        self._actual_overview = None

    @ExtendedController.observe("states", after=True)
    @ExtendedController.observe("change_state_type", after=True)
    @ExtendedController.observe("change_root_state_type", after=True)
    @ExtendedController.observe("input_data_ports", after=True)
    @ExtendedController.observe("output_data_ports", after=True)
    @ExtendedController.observe("scoped_variables", after=True)
    @ExtendedController.observe("data_flows", after=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        # The method causing the change raised an exception, thus nothing was changed
        # overview = NotificationOverview(info, False, 'DataFlowWidget')
        # if isinstance(overview['result'][-1], str) and "CRASH" in overview['result'][-1] or \
        #         isinstance(overview['result'][-1], Exception):
        #     return
        return
        # avoid updates because of execution status updates
        if 'kwargs' in info and 'method_name' in info['kwargs'] and \
                info['kwargs']['method_name'] in BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS:
            return

        # self.notification_logs(model, prop_name, info)
        if self.no_update and info.method_name in ["change_state_type", "change_root_state_type"]:
            # print "DO_UNLOCK DATA-FLOW WIDGET"
            self.no_update = False

        if self.no_update or self.no_update_state_destruction or self.no_update_self_or_parent_state_destruction:
            return

        overview = NotificationOverview(info, False, self.__class__.__name__)
        # print self, self.model.state.get_path(), overview

        # avoid updates because of unimportant methods
        if overview['prop_name'][0] in ['states', 'input_data_ports', 'output_data_ports', 'scoped_variables', 'data_flows'] and \
                overview['method_name'][-1] not in ['append', '__setitem__', '__delitem__', 'remove', 'name',
                                                    'change_data_type', 'from_key', 'to_key', 'from_state', 'to_state',
                                                    'modify_origin', 'modify_target']:
            return
        # print "DUPDATE ", self, overview

        try:
            self.update()
        except Exception as e:
            if self.debug_log:
                import traceback
                self.store_debug_log_file(str(overview))
                self.store_debug_log_file(str(traceback.format_exc()))
            logger.error("update of data_flow widget fails while detecting change in state %s %s" %
                         (self.model.state.name, self.model.state.state_id))

    def store_debug_log_file(self, string):
        with open(RAFCON_TEMP_PATH_BASE + '/data_flow_widget_debug_log_file.txt', 'a+') as f:
            f.write(string)

    def notification_logs(self, model, prop_name, info):

        if model.state.state_id == self.model.state.state_id:
            relative_str = "SELF"
            from_state = self.model.state.state_id
        elif self.model.parent and model.state.state_id == self.model.parent.state.state_id:
            relative_str = "PARENT"
            from_state = self.model.parent.state.state_id
        else:
            relative_str = "OTHER"
            from_state = model.state.state_id

        if prop_name == 'data_flows':
            logger.debug(
                "%s gets notified by data_flows from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'input_data_ports':
            logger.debug("%s gets notified by input_data_ports from %s %s" % (
                self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'output_data_ports':
            logger.debug("%s gets notified by output_data_ports from %s %s" % (
                self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'scoped_variables':
            logger.debug("%s gets notified by scoped_variables from %s %s" % (
                self.model.state.state_id, relative_str, from_state))
        else:
            logger.debug("IP OP SV or DF !!! FAILURE !!! %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
                         (self.model.state.state_id, prop_name, info.instance, info.method_name, info.result))


def get_key_combos(ports, keys_store, not_key=None):

    if not_key is not None and not_key in ports:  # in case of type changes not_key is not always in the list
        port = ports[not_key]
        keys_store.append([PORT_TYPE_TAG.get(type(port), 'None') + '.#' +
                           str(not_key) + '.' +
                           (port.data_type.__name__ or 'None') + '.' +
                           port.name])
    for key in ports.keys():
        if not not_key == key:
            port = ports[key]
            keys_store.append([PORT_TYPE_TAG.get(type(port), 'None') + '.#' +
                               str(key) + '.' +
                               (port.data_type.__name__ or 'None') + '.' +
                               port.name])
    # print "final store: ", keys_store
    return keys_store


def get_state_model(state_m, state_id):
    if state_id == state_m.state.state_id:
        return state_m
    elif isinstance(state_m, ContainerStateModel) and state_id in state_m.states:
        return state_m.states[state_id]
    return None


def update_data_flows(model, data_flow_dict, tree_dict_combos):
    """ Updates data flow dictionary and combo dictionary of the widget according handed model.

    :param model: model for which the data_flow_dict and tree_dict_combos should be updated
    :param data_flow_dict: dictionary that holds all internal and external data-flows and those respective row labels
    :param tree_dict_combos: dictionary that holds all internal and external data-flow-adaptation-combos
    :return:
    """
    data_flow_dict['internal'] = {}
    data_flow_dict['external'] = {}
    tree_dict_combos['internal'] = {}
    tree_dict_combos['external'] = {}

    # free input ports and scopes are real to_keys and real states
    [free_to_port_internal, from_ports_internal] = find_free_keys(model)
    [free_to_port_external, from_ports_external] = find_free_keys(model.parent)

    def take_from_dict(from_dict, key):
        if key in from_dict:
            return from_dict[key]
        else:
            logger.warning("Key '%s' is not in %s" % (key, from_dict))
            pass

    # from_state, to_key, to_state, to_key, external
    if isinstance(model, ContainerStateModel):
        for data_flow in model.state.data_flows.values():  # model.data_flow_list_store:

            # TREE STORE LABEL
            # check if from Self_state
            if data_flow.from_state == model.state.state_id:
                from_state = model.state
                from_state_label = 'self.' + model.state.name + '.' + data_flow.from_state
            else:
                if take_from_dict(model.state.states, data_flow.from_state):
                    from_state = take_from_dict(model.state.states, data_flow.from_state)
                    from_state_label = from_state.name + '.' + data_flow.from_state
                else:
                    # print data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                    logger.warning("DO break in ctrl/data_flow.py -1")
                    break
            # check if to Self_state
            if data_flow.to_state == model.state.state_id:
                to_state = model.state
                to_state_label = 'self.' + model.state.name + '.' + data_flow.to_state
            else:
                if take_from_dict(model.state.states, data_flow.to_state):
                    to_state = take_from_dict(model.state.states, data_flow.to_state)
                    to_state_label = to_state.name + '.' + data_flow.to_state
                else:
                    # print data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                    logger.warning("DO break in ctrl/data_flow.py 0")
                    break

            from_key_port = from_state.get_data_port_by_id(data_flow.from_key)
            from_key_label = ''
            if from_key_port is not None:
                from_key_label = PORT_TYPE_TAG.get(type(from_key_port), 'None') + '.' + \
                                 from_key_port.data_type.__name__ + '.' + \
                                 from_key_port.name

            to_key_port = to_state.get_data_port_by_id(data_flow.to_key)
            # to_key_label = ''
            if to_key_port is not None:
                to_key_label = PORT_TYPE_TAG.get(type(to_key_port), 'None') + '.' + \
                               (to_key_port.data_type.__name__ or 'None') + '.' + \
                               to_key_port.name
            data_flow_dict['internal'][data_flow.data_flow_id] = {'from_state': from_state_label,
                                                                  'from_key': from_key_label,
                                                                  'to_state': to_state_label,
                                                                  'to_key': to_key_label}

            # ALL INTERNAL COMBOS
            from_states_store = ListStore(str)
            to_states_store = ListStore(str)
            if isinstance(model, ContainerStateModel):
                if model.state.state_id in free_to_port_internal or model.state.state_id == data_flow.to_state:
                    to_states_store.append(['self.' + model.state.name + '.' + model.state.state_id])
                if model.state.state_id in from_ports_internal or model.state.state_id == data_flow.from_state:
                    from_states_store.append(['self.' + model.state.name + '.' + model.state.state_id])
                for state_model in model.states.itervalues():
                    if state_model.state.state_id in free_to_port_internal or \
                                    state_model.state.state_id == data_flow.to_state:
                        to_states_store.append([state_model.state.name + '.' + state_model.state.state_id])
                    if state_model.state.state_id in from_ports_internal or \
                                    state_model.state.state_id == data_flow.from_state:
                        from_states_store.append([state_model.state.name + '.' + state_model.state.state_id])

            from_keys_store = ListStore(str)
            if model.state.state_id == data_flow.from_state:
                # print "input_ports", model.state.input_data_ports
                # print type(model)
                if isinstance(model, ContainerStateModel):
                    # print "scoped_variables", model.state.scoped_variables
                    combined_ports = {}
                    combined_ports.update(model.state.scoped_variables)
                    combined_ports.update(model.state.input_data_ports)
                    get_key_combos(combined_ports, from_keys_store, data_flow.from_key)
                else:
                    get_key_combos(model.state.input_data_ports, from_keys_store, data_flow.from_key)
            else:
                # print "output_ports", model.states[data_flow.from_state].state.output_data_ports
                get_key_combos(model.state.states[data_flow.from_state].output_data_ports,
                               from_keys_store, data_flow.from_key)

            to_keys_store = ListStore(str)
            if model.state.state_id == data_flow.to_state:
                # print "output_ports", model.state.output_data_ports
                # print type(model)
                if isinstance(model, ContainerStateModel):
                    # print "scoped_variables", model.state.scoped_variables
                    combined_ports = {}
                    combined_ports.update(model.state.scoped_variables)
                    combined_ports.update(model.state.output_data_ports)
                    get_key_combos(combined_ports, to_keys_store, data_flow.to_key)
                else:
                    get_key_combos(model.state.output_data_ports, to_keys_store, data_flow.to_key)
            else:
                # print "input_ports", model.states[data_flow.to_state].state.input_data_ports
                get_key_combos(model.state.states[data_flow.to_state].input_data_ports
                               , to_keys_store, data_flow.to_key)
            tree_dict_combos['internal'][data_flow.data_flow_id] = {'from_state': from_states_store,
                                                                    'from_key': from_keys_store,
                                                                    'to_state': to_states_store,
                                                                    'to_key': to_keys_store}
            # print "internal", data_flow_dict['internal'][data_flow.data_flow_id]

    if not model.state.is_root_state:
        for data_flow in model.parent.state.data_flows.values():  # model.parent.data_flow_list_store:

            # TREE STORE LABEL
            # check if from Self_state
            if model.state.state_id == data_flow.from_state:
                from_state = model.state
                from_state_label = 'self.' + model.state.name + '.' + data_flow.from_state
            else:
                if model.parent.state.state_id == data_flow.from_state:
                    from_state = model.parent.state
                    from_state_label = 'parent.' + model.parent.state.name + '.' + data_flow.from_state
                else:
                    if take_from_dict(model.parent.state.states, data_flow.from_state):
                        from_state = take_from_dict(model.parent.state.states, data_flow.from_state)
                        from_state_label = from_state.name + '.' + data_flow.from_state
                    else:
                        # print "#", data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                        logger.warning("DO break in ctrl/data_flow.py 1")
                        break

            # check if to Self_state
            if model.state.state_id == data_flow.to_state:
                to_state = model.state
                to_state_label = 'self.' + model.state.name + '.' + data_flow.to_state
            else:
                if model.parent.state.state_id == data_flow.to_state:
                    to_state = model.parent.state
                    to_state_label = 'parent.' + model.parent.state.name + '.' + data_flow.to_state
                else:
                    if take_from_dict(model.parent.state.states, data_flow.to_state):
                        to_state = take_from_dict(model.parent.state.states, data_flow.to_state)
                        to_state_label = to_state.name + '.' + data_flow.to_state
                    else:
                        # print "##", data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                        logger.warning("DO break in ctrl/data_flow.py 2")
                        break
            if model.state.state_id in [data_flow.from_state, data_flow.to_state]:
                from_key_port = from_state.get_data_port_by_id(data_flow.from_key)
                if from_key_port is None:
                    continue
                from_key_label = PORT_TYPE_TAG.get(type(from_key_port), 'None') + '.' + \
                                 from_key_port.data_type.__name__ + '.' + \
                                 from_key_port.name
                to_key_port = to_state.get_data_port_by_id(data_flow.to_key)
                if to_key_port is None:
                    continue
                to_key_label = PORT_TYPE_TAG.get(type(to_key_port), 'None') + '.' + \
                               to_key_port.data_type.__name__ + '.' + \
                               to_key_port.name
                data_flow_dict['external'][data_flow.data_flow_id] = {'from_state': from_state_label,
                                                                      'from_key': from_key_label,
                                                                      'to_state': to_state_label,
                                                                      'to_key': to_key_label}

            # ALL EXTERNAL COMBOS
            if model.state.state_id in [data_flow.from_state, data_flow.to_state]:

                # only self-state
                from_states_store = ListStore(str)
                for state_id in from_ports_external.keys():
                    if model.parent.state.state_id == state_id:
                        state_model = model.parent
                    else:
                        state_model = model.parent.states[state_id]
                    if state_model.state.state_id == model.state.state_id:
                        from_states_store.append(['self.' + state_model.state.name + '.' + state_model.state.state_id])
                    else:
                        from_states_store.append([state_model.state.name + '.' + state_model.state.state_id])
                        # from_states_store.append(['self.' + model.state.name + '.' + model.state.state_id])

                # only outports of self
                from_keys_store = ListStore(str)
                if model.parent.state.state_id == data_flow.from_state:
                    # print "output_ports", model.parent.states[data_flow.from_state].state.output_data_ports
                    combined_ports = {}
                    combined_ports.update(model.parent.state.input_data_ports)
                    combined_ports.update(model.parent.state.scoped_variables)
                    get_key_combos(combined_ports, from_keys_store, data_flow.to_key)
                elif data_flow.from_state in [state_m.state.state_id for state_m in model.parent.states.values()]:
                    get_key_combos(model.parent.state.states[data_flow.from_state].output_data_ports,
                                   from_keys_store, data_flow.to_key)
                else:
                    logger.error(
                        "---------------- FAILURE %s ------------- external from_state PARENT or STATES" % model.state.state_id)

                # all states and parent-state
                to_states_store = ListStore(str)
                for state_id in free_to_port_external.keys():
                    if model.parent.state.state_id == state_id:
                        state_model = model.parent
                    else:
                        state_model = model.parent.states[state_id]
                    if state_model.state.state_id == model.state.state_id:
                        to_states_store.append(['self.' + state_model.state.name + '.' + state_model.state.state_id])
                    else:
                        to_states_store.append([state_model.state.name + '.' + state_model.state.state_id])

                # all keys of actual to-state
                to_keys_store = ListStore(str)
                if get_state_model(model.parent, data_flow.to_state):
                    to_state_model = get_state_model(model.parent, data_flow.to_state)
                    port = to_state_model.state.get_data_port_by_id(data_flow.to_key)
                    # if not port.data_port_id == data_flow.from_key:
                    to_keys_store.append([PORT_TYPE_TAG.get(type(port), 'None') + '.#' +
                                          str(port.data_port_id) + '.' +
                                          port.data_type.__name__ + '.' +
                                          port.name])
                if data_flow.to_state in free_to_port_external:
                    for port in free_to_port_external[data_flow.to_state]:
                        # to_state = get_state_model(model.parent, data_flow.to_state).state
                        if not port.data_port_id == data_flow.from_key:
                            to_keys_store.append([PORT_TYPE_TAG.get(type(port), 'None') + '.#' +
                                                  str(port.data_port_id) + '.' +
                                                  port.data_type.__name__ + '.' +
                                                  port.name])

                tree_dict_combos['external'][data_flow.data_flow_id] = {'from_state': from_states_store,
                                                                        'from_key': from_keys_store,
                                                                        'to_state': to_states_store,
                                                                        'to_key': to_keys_store}
                # print "external", data_flow_dict['external'][data_flow.data_flow_id]

    # print "ALL SCANNED: ", data_flow_dict['internal'].keys(), data_flow_dict['external'].keys(), \
    #     tree_dict_combos['internal'].keys(), tree_dict_combos['external'].keys()
    return free_to_port_internal, free_to_port_external, from_ports_internal, from_ports_external


def find_free_keys(model):
    free_to_ports = {}
    nfree_to_ports = {}
    from_ports = {}

    # check for container state
    if model is not None and isinstance(model, ContainerStateModel):
        free_container_ports = []
        container_from_ports = []
        nfree_container_ports = []
        free_container_ports.extend(model.state.scoped_variables.values())
        container_from_ports.extend(model.state.scoped_variables.values())
        container_from_ports.extend(model.state.input_data_ports.values())
        # free_container_ports.extend(model.state.scoped_variables.keys())
        nfree_container_ports.extend([s.name for s in model.state.scoped_variables.values()])
        if model.state.output_data_ports:
            port_keys = model.state.output_data_ports.keys()
            # print "actual keys: ", port_keys
            for data_flow in model.state.data_flows.values():
                port_keys = filter(lambda port_id: not (model.state.state_id == data_flow.to_state and
                                                        port_id == data_flow.to_key), port_keys)
                # print "actual keys: ", port_keys
            # print "found free prots: ", port_keys
            free_container_ports.extend([model.state.output_data_ports[i] for i in port_keys])
            # free_container_ports.extend(port_keys)
            nfree_container_ports.extend([model.state.output_data_ports[i].name for i in port_keys])

        if free_container_ports:
            free_to_ports[model.state.state_id] = free_container_ports
            nfree_to_ports[model.state.name] = nfree_container_ports
        if container_from_ports:
            from_ports[model.state.state_id] = container_from_ports

        # check every single state
        for state_model in model.states.values():
            if state_model.state.output_data_ports:
                from_ports[state_model.state.state_id] = state_model.state.output_data_ports.values()

            if state_model.state.input_data_ports:
                port_keys = state_model.state.input_data_ports.keys()
                # print "actual keys: ", port_keys
                for data_flow in model.state.data_flows.values():
                    port_keys = filter(lambda port_id: not (state_model.state.state_id == data_flow.to_state and
                                                            port_id == data_flow.to_key), port_keys)
                    # print "actual keys: ", port_keys

                # print "found free prots: ", port_keys
                if port_keys:
                    free_to_ports[state_model.state.state_id] = [state_model.state.input_data_ports[i] for i in
                                                                 port_keys]
                    nfree_to_ports[state_model.state.name] = [state_model.state.input_data_ports[i].name for i in
                                                              port_keys]

    # print "\nFOUND FREE PORTS: \n", nfree_to_ports, "\n", free_to_ports, "\n",  from_ports

    return free_to_ports, from_ports


class StateDataFlowsEditorController(ExtendedController):
    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.df_list_ctrl = StateDataFlowsListController(model, view.data_flows_listView)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """

        view['add_d_button'].connect('clicked', self.df_list_ctrl.on_add)
        view['remove_d_button'].connect('clicked', self.df_list_ctrl.on_remove)
        view['connected_to_d_checkbutton'].connect('toggled', self.toggled_button, 'data_flows_external')
        view['internal_d_checkbutton'].connect('toggled', self.toggled_button, 'data_flows_internal')

        if isinstance(self.model.state, LibraryState):
            view['internal_d_checkbutton'].set_sensitive(False)
            view['internal_d_checkbutton'].set_active(False)

        if self.model.parent is not None and isinstance(self.model.parent.state, LibraryState):
            view['add_d_button'].set_sensitive(False)
            view['remove_d_button'].set_sensitive(False)

        if self.model.state.is_root_state:
            self.df_list_ctrl.view_dict['data_flows_external'] = False
            view['connected_to_d_checkbutton'].set_active(False)

        if not isinstance(self.model, ContainerStateModel):
            self.df_list_ctrl.view_dict['data_flows_internal'] = False
            view['internal_d_checkbutton'].set_active(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        # self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_data_flow)
        shortcut_manager.add_callback_for_action("add", self.add_data_flow)

    def add_data_flow(self, *_):
        if self.view and self.view.data_flows_listView.tree_view.is_focus():
            return self.df_list_ctrl.on_add(None)

    def remove_data_flow(self, *_):
        if self.view and self.view.data_flows_listView.tree_view.is_focus():
            return self.df_list_ctrl.on_remove(None)

    def toggled_button(self, button, name=None):

        if name in ['data_flows_external'] and not self.model.state.is_root_state:
            self.df_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['data_flows_internal']:
            self.df_list_ctrl.view_dict['data_flows_external'] = False
            button.set_active(False)

        if name in ['data_flows_internal'] and isinstance(self.model, ContainerStateModel):
            self.df_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['data_flows_external']:
            self.df_list_ctrl.view_dict['data_flows_internal'] = False
            button.set_active(False)

        self.df_list_ctrl._update_internal_data_base()
        self.df_list_ctrl._update_tree_store()
