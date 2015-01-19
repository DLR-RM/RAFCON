
from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
from gtk import ListStore


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


    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            col = column.get_name()
            states_store = ListStore(str, str)
            states_store.append([container_model.state.state_id, container_model.state.name])
            data_flow = model.get_value(iter, 0)
            for state_model in container_model.states.itervalues():
                states_store.append([state_model.state.state_id, state_model.state.name])
            if col == 'from_state_col':
                text = 'Sel. state' if container_model.state.state_id == data_flow.from_state else  \
                    container_model.state.states[data_flow.from_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'to_state_col':
                text = 'Sel. state' if container_model.state.state_id == data_flow.to_state else  \
                    container_model.state.states[data_flow.to_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'from_key_col':
                cell_renderer.set_property('text', data_flow.from_key)
            elif col == 'to_key_col':
                cell_renderer.set_property('text', data_flow.to_key)
            else:
                logger.error("Unknown column '{col:s}' in DataFlowListView".format(col=col))



        view.get_top_widget().set_model(self.model.data_flow_list_store)

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_key_col'].set_cell_data_func(view['from_key_combo'], cell_text, self.model)
        view['to_key_col'].set_cell_data_func(view['to_key_combo'], cell_text, self.model)


        view['from_state_combo'].connect("edited", self.on_combo_changed)
        view['to_state_combo'].connect("edited", self.on_combo_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_combo_changed(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

