
import gobject
from gtk import ListStore, TreeStore
from gtkmvc import Observer
from awesome_tool.mvc.controllers.extended_controller import ExtendedController

from awesome_tool.statemachine.states.state import State
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class StateDataFlowsListController(ExtendedController):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.data_flow.DataFlowListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param awesome_tool.mvc.models.ContainerStateModel model: The container state model containing the data
    :param awesome_tool.mvc.views.DataFlowListView view: The GTK view showing the data flows as a table
    """

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

        self.tree_dict_combos = {'internal':    {},
                                 'external':    {}}
        self.data_flow_dict = {'internal':    {},
                               'external':    {}}

        # register other model and fill tree_store the model of the view
        if model.parent is not None:
            self.observe_model(model.parent)

        self.update_internal_data_base()
        self.update_tree_store()

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            df_id = model.get_value(iter, 0)
            in_external = 'internal'
            if model.get_value(iter, 5):
                in_external = 'external'
            if column.get_title() == 'From-State':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['from_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'From-Key':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['from_key'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'To-State':
                cell_renderer.set_property("model", self.tree_dict_combos[in_external][df_id]['to_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'To-Key':
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
        #view['external_toggle'].connect("edited", self.on_external_toggled)
        view.tree_view.connect("grab-focus", self.on_focus)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_focus(self, widget, data=None):
        path = self.view.tree_view.get_cursor()
        logger.debug("DATAFLOWS_LIST get new FOCUS %s" % str(path[0]))
        self.update_internal_data_base()
        self.update_tree_store()
        if path[0]:
            self.view.tree_view.set_cursor(path[0])

    def on_add(self, button, info=None):
        # print "ADD DATA_FLOW"
        if self.view_dict['data_flows_internal'] and self.free_to_port_internal:
            # print self.from_port_internal
            from_state_id = self.from_port_internal.keys()[0]
            from_key = self.from_port_internal[from_state_id][0].data_port_id
            # print from_state_id, from_key, self.model.state.state_id
            to_state_id = self.free_to_port_internal.keys()[0]
            to_key = self.free_to_port_internal[to_state_id][0].data_port_id
            # print "NEW DATA_FLOW INTERNAL IS: ", from_state_id, from_key, to_state_id, to_key
            data_flow_id = self.model.state.add_data_flow(from_state_id, from_key, to_state_id, to_key)
            # print "NEW DATA_FLOW INTERNAL IS: ", self.model.state.data_flows[data_flow_id]

        elif self.view_dict['data_flows_external'] and self.model.state.output_data_ports:  # self.free_to_port_external:
            from_state_id = self.model.state.state_id
            # print from_state_id, self.model.state.output_data_ports
            from_key = self.model.state.output_data_ports.keys()[0]
            to_state_id = self.free_to_port_external.keys()[0]
            to_key = self.free_to_port_external[to_state_id][0].data_port_id
            # print "NEW DATA_FLOW EXTERNAL IS: ", from_state_id, from_key, to_state_id, to_key, \
            #     get_state_model(self.model.parent, to_state_id).state.get_data_port_by_id(to_key)
            data_flow_id = self.model.parent.state.add_data_flow(from_state_id, from_key, to_state_id, to_key)
            # print "NEW DATA_FLOW EXTERNAL IS: ", self.model.parent.state.data_flows[data_flow_id]
        else:
                logger.warning("NO OPTION TO ADD TRANSITION")

        # set focus on this new element
        # - at the moment every new element is the last -> easy work around :(
        self.view.tree_view.set_cursor(len(self.tree_store)-1)

    def on_remove(self, button, info=None):
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        if path:
            if self.tree_store[path[0][0]][5]:
                self.model.parent.state.remove_data_flow(self.tree_store[path[0][0]][0])
            else:
                self.model.state.remove_data_flow(self.tree_store[path[0][0]][0])
        else:
            logger.warning("Please select the data flow to be deleted")
            return

        # selection to next element
        row_number = path[0][0]
        if len(self.tree_store) > 0:
            self.view.tree_view.set_cursor(min(row_number, len(self.tree_store)-1))

    def on_combo_changed_from_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        if text is None:
            return
        text = text.split('.')
        df_id = self.tree_store[path][0]
        # df = self.tree_store[path][8]
        fk = None  # df.from_key
        if self.tree_store[path][5]:  # external
            fk = self.from_port_external[text[-1]][0].data_port_id
            self.model.parent.state.modify_data_flow_from_state(data_flow_id=df_id, from_state=text[-1], from_key=fk)
        else:
            state_model = get_state_model(self.model, text[-1])
            if state_model.state.state_id == self.model.state.state_id:
                if self.model.state.input_data_ports:
                    fk = self.model.state.input_data_ports.values()[0].data_port_id
                if self.model.scoped_variables:
                    fk = self.model.scoped_variables[0].scoped_variable.data_port_id
            else:  # child-state
                if state_model.state.output_data_ports:
                    fk = state_model.state.output_data_ports.values()[0].data_port_id
            if fk:
                self.model.state.modify_data_flow_from_state(data_flow_id=df_id, from_state=text[-1], from_key=fk)

    def on_combo_changed_from_key(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        if text is None:
            return
        text = text.split('.')
        df_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # external
            self.model.parent.state.modify_data_flow_from_key(data_flow_id=df_id, from_key=int(text[-1]))
        else:
            self.model.state.modify_data_flow_from_key(data_flow_id=df_id, from_key=int(text[-1]))

    def on_combo_changed_to_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        if text is None:
            return
        #self.combo['free_ext_from_outcomes_dict']
        text = text.split('.')
        df_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # external
            tk = self.free_to_port_external[text[-1]][0].data_port_id
            self.model.parent.state.modify_data_flow_to_state(data_flow_id=df_id, to_state=text[-1], to_key=tk)
        else:
            tk = self.free_to_port_internal[text[-1]][0].data_port_id
            self.model.state.modify_data_flow_to_state(data_flow_id=df_id, to_state=text[-1], to_key=tk)

    def on_combo_changed_to_key(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        if text is None:
            return
        text = text.split('.')
        df_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # external
            self.model.parent.state.modify_data_flow_to_key(data_flow_id=df_id, to_key=int(text[-1]))
        else:
            self.model.state.modify_data_flow_to_key(data_flow_id=df_id, to_key=int(text[-1]))

    def update_internal_data_base(self):
        [free_to_int, free_to_ext, from_int, from_ext] = update_data_flow(self.model, self.data_flow_dict, self.tree_dict_combos)
        self.free_to_port_internal = free_to_int
        self.free_to_port_external = free_to_ext
        self.from_port_internal = from_int
        self.from_port_external = from_ext

    def update_tree_store(self):
        self.tree_store.clear()

        if self.view_dict['data_flows_internal'] and hasattr(self.model.state, 'data_flows'):
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

        if self.view_dict['data_flows_external'] and self.model.parent:
            for data_flow in self.model.parent.state.data_flows.values():
                #data_flow = row[0]
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

    @ExtendedController.observe("input_data_ports", after=True)
    @ExtendedController.observe("output_data_ports", after=True)
    @ExtendedController.observe("scoped_variables", after=True)
    @ExtendedController.observe("data_flows", after=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        # self.notification_logs(model, prop_name, info)

        self.update_internal_data_base()
        self.update_tree_store()

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
            logger.debug("%s gets notified by data_flows from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'input_data_ports':
            logger.debug("%s gets notified by input_data_ports from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'output_data_ports':
            logger.debug("%s gets notified by output_data_ports from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'scoped_variables':
            logger.debug("%s gets notified by scoped_variables from %s %s" % (self.model.state.state_id, relative_str, from_state))
        else:
            logger.debug("IP OP SV or DF !!! FAILURE !!! %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
                         (self.model.state.state_id, prop_name, info.instance, info.method_name, info.result))


def get_key_combos(ports, keys_store, port_type, not_key=None):

    if (port_type == "input_port" or port_type == "output_port") and not type(ports) is list:
        for key in ports.keys():
            port = ports[key]
            keys_store.append([port_type + '.' + port.name + '.' + str(key)])
    else:  # scoped_variable
        # print type(ports), "\n", ports
        if type(ports) == type(list):
            for scope in ports:
                keys_store.append([scope.data_port.data_type + '.' + scope.data_port.name + '.' + scope.data_port.port_id])
        elif ports:
            # print ports
            try:
                for scope in ports.values():
                    # print scope
                    keys_store.append([port_type + '.' + scope.name + '.' + str(scope.data_port_id)])
            except AttributeError:
                for scope in ports:
                    # print scope, scope.scoped_variable
                    scope = scope.scoped_variable
                    keys_store.append([port_type + '.' + scope.name + '.' + str(scope.data_port_id)])

    # print "final store: ", keys_store
    return keys_store


def get_state_model(model, state_id):
    state_model = None
    if state_id == model.state.state_id:
        state_model = model
    elif hasattr(model, 'states') and state_id in model.states:
        state_model = model.states[state_id]
    return state_model


def update_data_flow(model, data_flow_dict, tree_dict_combos):
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
    if hasattr(model.state, 'states'):
        for data_flow in model.state.data_flows.values():  # model.data_flow_list_store:

            # TREE STORE LABEL
            # check if from Self_state
            if data_flow.from_state == model.state.state_id:
                fstate = model.state
                from_state = 'self.' + model.state.name + '.' + data_flow.from_state
            else:
                if take_from_dict(model.states, data_flow.from_state):
                    fstate = take_from_dict(model.states, data_flow.from_state).state
                    from_state = fstate.name + '.' + data_flow.from_state
                else:
                    # print data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                    break
            # check if to Self_state
            if data_flow.to_state == model.state.state_id:
                tstate = model.state
                to_state = 'self.' + model.state.name + '.' + data_flow.to_state
            else:
                if take_from_dict(model.states, data_flow.to_state):
                    tstate = take_from_dict(model.states, data_flow.to_state).state
                    to_state = tstate.name + '.' + data_flow.to_state
                else:
                    # print data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                    break

            from_key_port = fstate.get_data_port_by_id(data_flow.from_key)
            from_key_label = from_key_port.data_type + '.' + from_key_port.name + '.' + str(data_flow.from_key)
            to_key_port = tstate.get_data_port_by_id(data_flow.to_key)

            to_key_label = (to_key_port.data_type or 'None') + '.' + to_key_port.name + '.' + str(data_flow.to_key)
            data_flow_dict['internal'][data_flow.data_flow_id] = {'from_state': from_state,
                                                                  'from_key': from_key_label,
                                                                  'to_state': to_state,
                                                                  'to_key': to_key_label}

            # ALL INTERNAL COMBOS
            from_states_store = ListStore(str)
            to_states_store = ListStore(str)
            if hasattr(model, 'states'):
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
                get_key_combos(model.state.input_data_ports, from_keys_store, 'input_port', data_flow.to_key)
                # print type(model)
                if hasattr(model, 'states'):
                    # print "scoped_variables", model.state.scoped_variables
                    get_key_combos(model.state.scoped_variables, from_keys_store, 'scoped_variable', data_flow.to_key)
            else:
                # print "output_ports", model.states[data_flow.from_state].state.output_data_ports
                get_key_combos(model.states[data_flow.from_state].state.output_data_ports,
                               from_keys_store, 'output_port', data_flow.to_key)

            to_keys_store = ListStore(str)
            if model.state.state_id == data_flow.to_state:
                # print "output_ports", model.state.output_data_ports
                get_key_combos(model.state.output_data_ports, to_keys_store, 'output_port', data_flow.from_key)
                # print type(model)
                if hasattr(model, 'states'):
                    # print "scoped_variables", model.state.scoped_variables
                    get_key_combos(model.state.scoped_variables, to_keys_store, 'scoped_variable', data_flow.from_key)
            else:
                # print "input_ports", model.states[data_flow.to_state].state.input_data_ports
                get_key_combos(model.states[data_flow.to_state].state.input_data_ports
                               , to_keys_store, 'input_port', data_flow.from_key)
            tree_dict_combos['internal'][data_flow.data_flow_id] = {'from_state': from_states_store,
                                                                    'from_key': from_keys_store,
                                                                    'to_state': to_states_store,
                                                                    'to_key': to_keys_store}
            # print "internal", data_flow_dict['internal'][data_flow.data_flow_id]

    if model.parent is not None:  # if it is the top container state
        for data_flow in model.parent.state.data_flows.values():  # model.parent.data_flow_list_store:

            # TREE STORE LABEL
            # check if from Self_state
            if model.state.state_id == data_flow.from_state:
                fstate = model.state
                from_state = 'self.' + model.state.name + '.' + data_flow.from_state
            else:
                if model.parent.state.state_id == data_flow.from_state:
                    fstate = model.parent.state
                    from_state = 'parent.' + model.parent.state.name + '.' + data_flow.from_state
                else:
                    if take_from_dict(model.parent.states, data_flow.from_state):
                        fstate = take_from_dict(model.parent.states, data_flow.from_state).state
                        from_state = fstate.name + '.' + data_flow.from_state
                    else:
                        # print "#", data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                        logger.warning("DO break in ctrl/data_flow.py 1")
                        break

            # check if to Self_state
            if model.state.state_id == data_flow.to_state:
                tstate = model.state
                to_state = 'self.' + model.state.name + '.' + data_flow.to_state
            else:
                if model.parent.state.state_id == data_flow.to_state:
                    tstate = model.parent.state
                    to_state = 'parent.' + model.parent.state.name + '.' + data_flow.to_state
                else:
                    if take_from_dict(model.parent.states, data_flow.to_state):
                        tstate = take_from_dict(model.parent.states, data_flow.to_state).state
                        to_state = fstate.name + '.' + data_flow.to_state
                    else:
                        # print "##", data_flow.from_state, data_flow.from_key, data_flow.to_state, data_flow.to_key
                        logger.warning("DO break in ctrl/data_flow.py 2")
                        break
            if model.state.state_id in [data_flow.from_state, data_flow.to_state]:
                from_key_port = fstate.get_data_port_by_id(data_flow.from_key)
                from_key_label = from_key_port.data_type + '.' + from_key_port.name + '.' + str(data_flow.from_key)
                to_key_port = tstate.get_data_port_by_id(data_flow.to_key)
                to_key_label = to_key_port.data_type + '.' + to_key_port.name + '.' + str(data_flow.to_key)
                data_flow_dict['external'][data_flow.data_flow_id] = {'from_state': from_state,
                                                                      'from_key': from_key_label,
                                                                      'to_state': to_state,
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
                    #from_states_store.append(['self.' + model.state.name + '.' + model.state.state_id])

                # only outports of self
                from_keys_store = ListStore(str)
                if model.parent.state.state_id == data_flow.from_state:
                    # print "output_ports", model.parent.states[data_flow.from_state].state.output_data_ports
                    get_key_combos(model.parent.state.input_data_ports,
                                   from_keys_store, 'input_port', data_flow.to_key)
                    get_key_combos(model.parent.state.scoped_variables,
                                   from_keys_store, 'scoped_variable', data_flow.to_key)
                elif data_flow.from_state in [state_m.state.state_id for state_m in model.parent.states.values()]:
                    get_key_combos(model.parent.states[data_flow.from_state].state.output_data_ports,
                                   from_keys_store, 'output_port', data_flow.to_key)
                else:
                    logger.error("---------------- FAILURE %s ------------- external from_state PARENT or STATES" % model.state.state_id)

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
                if data_flow.to_state in free_to_port_external:
                    for port in free_to_port_external[data_flow.to_state]:
                        to_state = get_state_model(model.parent, data_flow.to_state).state
                        if not port.data_port_id == data_flow.from_key:
                            to_keys_store.append([port.data_type + '.' + port.name + '.' + str(port.data_port_id)])
                else:
                    if get_state_model(model.parent, data_flow.to_state):
                        to_state_model = get_state_model(model.parent, data_flow.to_state)
                        port = to_state_model.state.get_data_port_by_id(data_flow.to_key)
                        if not port.data_port_id == data_flow.from_key:
                            to_keys_store.append([port.data_type + '.' + port.name + '.' + str(port.data_port_id)])

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
    if hasattr(model, 'states'):
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
                    free_to_ports[state_model.state.state_id] = [state_model.state.input_data_ports[i] for i in port_keys]
                    nfree_to_ports[state_model.state.name] = [state_model.state.input_data_ports[i].name for i in port_keys]

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

        if self.model.parent is None:
            self.df_list_ctrl.view_dict['data_flows_external'] = False
            view['connected_to_d_checkbutton'].set_active(False)

        if not hasattr(self.model, 'states'):
            self.df_list_ctrl.view_dict['data_flows_internal'] = False
            view['internal_d_checkbutton'].set_active(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_data_flow)
        shortcut_manager.add_callback_for_action("add", self.add_data_flow)

    def add_data_flow(self, *_):
        if self.view.data_flows_listView.tree_view.has_focus():
            self.df_list_ctrl.on_add(None)

    def remove_data_flow(self, *_):
        if self.view.data_flows_listView.tree_view.has_focus():
            self.df_list_ctrl.on_remove(None)

    def toggled_button(self, button, name=None):

        if name in ['data_flows_external'] and self.model.parent is not None:
            self.df_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['data_flows_internal']:
            self.df_list_ctrl.view_dict['data_flows_external'] = False
            button.set_active(False)

        if name in ['data_flows_internal'] and hasattr(self.model, 'states'):
            self.df_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['data_flows_external']:
            self.df_list_ctrl.view_dict['data_flows_internal'] = False
            button.set_active(False)

        self.df_list_ctrl.update_internal_data_base()
        self.df_list_ctrl.update_tree_store()
