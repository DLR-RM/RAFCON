
from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
from gtk import ListStore
import gobject

from mvc.models import ContainerStateModel, StateModel
from mvc.models.data_port import DataPortModel


class DataFlowListController(Controller):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.data_flow.DataFlowListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.ContainerStateModel model: The container state model containing the data
    :param mvc.views.DataFlowListView view: The GTK view showing the data flows as a table
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.tree_dict_combos = {'internal':    {'from': {'state': ListStore(gobject.TYPE_PYOBJECT),
                                                          'key': ListStore(gobject.TYPE_PYOBJECT)},
                                                 'to':   {'state': ListStore(gobject.TYPE_PYOBJECT),
                                                          'key': ListStore(gobject.TYPE_PYOBJECT)}},
                                 'external':    {'from': {'state': ListStore(gobject.TYPE_PYOBJECT),
                                                          'key': ListStore(gobject.TYPE_PYOBJECT)},
                                                 'to':   {'state': ListStore(gobject.TYPE_PYOBJECT),
                                                          'key': ListStore(gobject.TYPE_PYOBJECT)}}}
        self.dataflow_dict = {'internal':    {},
                              'external':    {}}

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            col = column.get_name()
            states_store = ListStore(str, str)
            states_store.append([container_model.state.state_id, container_model.state.name])
            data_flow = model.get_value(iter, 0)
            if isinstance(container_model, ContainerStateModel):
                for state_model in container_model.states.itervalues():
                    states_store.append([state_model.state.state_id, state_model.state.name])
            else:
                states_store.append([container_model.state.state_id, container_model.state.name])
            keys_store = ListStore(str, str)
            if col == 'from_state_col':
                if container_model.state.state_id == data_flow.from_state:
                    text = 'Sel. state'
                else:
                    text = container_model.state.states[data_flow.from_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'to_state_col':
                if container_model.state.state_id == data_flow.to_state:
                    text = 'Sel. state'
                else:
                    text = container_model.state.states[data_flow.to_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'from_key_col':
                if container_model.state.state_id == data_flow.from_state:
                    print "input_ports", container_model.input_data_ports
                    self.get_key_combos(container_model.input_data_ports, keys_store, 'input_ports')
                    print type(container_model)
                    if type(container_model) is ContainerStateModel:
                        print "scoped_variables", container_model.scoped_variables
                        self.get_key_combos(container_model.scoped_variables, keys_store, 'scoped_variable')
                else:
                    print "output_ports", container_model.state.states[data_flow.from_state].output_data_ports
                    self.get_key_combos(container_model.state.states[data_flow.from_state].output_data_ports,
                                        keys_store, 'output_port')
                cell_renderer.set_property('text', data_flow.from_key)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', keys_store)
            elif col == 'to_key_col':
                if container_model.state.state_id == data_flow.to_state:
                    print "output_ports", container_model.output_data_ports
                    self.get_key_combos(container_model.output_data_ports, keys_store, 'output_ports')
                    print type(container_model)
                    if type(container_model) is ContainerStateModel:
                        print "scoped_variables", container_model.scoped_variables
                        self.get_key_combos(container_model.scoped_variables, keys_store, 'scoped_variable')
                else:
                    print "input_ports", container_model.state.states[data_flow.to_state].input_data_ports
                    self.get_key_combos(container_model.state.states[data_flow.to_state].input_data_ports,
                                        keys_store, 'input_port')
                cell_renderer.set_property('text', data_flow.to_key)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', keys_store)
            else:
                logger.error("Unknown column '{col:s}' in DataFlowListView".format(col=col))

        self.update_combos()
        self.update_data_flow()

        if isinstance(self.model.parent, StateModel):
            import gobject
            data_flow_list_store = ListStore(gobject.TYPE_PYOBJECT)
            if isinstance(self.model, ContainerStateModel):
                for elem in self.model.data_flow_list_store:
                    data_flow_list_store.append(elem)
            for elem in self.model.parent.data_flow_list_store:
                data_flow_list_store.append(elem)
            view.get_top_widget().set_model(data_flow_list_store)
            # if isinstance(self.model, ContainerStateModel):
            #     view.get_top_widget().set_model(self.model.data_flow_list_store)
        else:
            if isinstance(self.model, ContainerStateModel):
                view.get_top_widget().set_model(self.model.data_flow_list_store)

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_key_col'].set_cell_data_func(view['from_key_combo'], cell_text, self.model)
        view['to_key_col'].set_cell_data_func(view['to_key_combo'], cell_text, self.model)

        view['from_state_combo'].connect("edited", self.on_combo_changed)
        view['from_key_combo'].connect("edited", self.on_combo_changed)
        view['to_state_combo'].connect("edited", self.on_combo_changed)
        view['to_key_combo'].connect("edited", self.on_combo_changed)
        #view['external_toggle'].connect("edited", self.on_external_toggled)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def get_key_combos(self, ports, keys_store, port_type):

        if (port_type == "input_port" or port_type == "output_port") and not type(ports) is list:
            for key in ports.keys():
                keys_store.append([port_type, key])
        else:  # scoped_variable

            for scope in ports:
                if type(scope) is DataPortModel:
                    keys_store.append([scope.data_port.data_type, scope.data_port.name])
                else:
                    print scope
                    keys_store.append([port_type, scope.scoped_variable.name])
        print "final store: ", keys_store
        return keys_store

    def update_combos(model):
        pass

    def update_data_flow(model):
        pass

    def on_combo_changed(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

def update_combos(model,):
    tree_dict_combos = {'internal':    {'from': {'state': {},
                                                          'key': {}},
                                                 'to':   {'state': {},
                                                          'key': {}}},
                                 'external':    {'from': {'state': {},
                                                          'key': {}},
                                                 'to':   {'state': {},
                                                          'key': {}}}}
    ListStore(str)
    pass

def update_data_flow(model):
    dataflow_dict = {'internal':    {},
                     'external':    {}}
    # from_state, to_key, to_state, to_key, external
    pass