from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
from gtkmvc import Observer

from statemachine.states.state import DataPort


class InputDataPortListObserver(Observer):

    @Observer.observe("state_input_data_ports", after=True)
    def assign_notification_idp(self, model, prop_name, info):
        print "call_notification: AFTER: %s\n %s\n %s\n %s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        model.update_input_data_port_list_store()

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        model.update_input_data_port_list_store()


class InputDataPortListController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)


    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            col = column.get_name()
            data_port = model.get_value(iter, 0)
            if col == 'name_col':
                name = container_model.state.input_data_ports[data_port.name].name
                cell_renderer.set_property('text', name)
            elif col == 'data_type_col':
                data_type = container_model.state.input_data_ports[data_port.name].data_type
                cell_renderer.set_property('text', data_type)
            elif col == 'default_value_col':
                default_value = container_model.state.input_data_ports[data_port.name].default_value
                cell_renderer.set_property('text', default_value)
            else:
                logger.error("Unknown column '{col:s}' in TransitionListView".format(col=col))

        #top widget is a tree view => set the model of the tree view to be a list store
        view.get_top_widget().set_model(self.model.input_data_port_list_store)

        view['name_col'].set_cell_data_func(view['name_text'], cell_text, self.model)
        view['name_text'].set_property("editable", True)
        view['data_type_col'].set_cell_data_func(view['data_type_text'], cell_text, self.model)
        view['data_type_text'].set_property("editable", True)
        view['default_value_col'].set_cell_data_func(view['default_value_text'], cell_text, self.model)
        view['default_value_text'].set_property("editable", True)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)
        view['default_value_text'].connect("edited", self.on_default_value_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_name_changed(self, widget, path, text):
        print path
        import copy
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        #print self.view.get_top_widget().get_selection().get_selected_rows()
        key = self.model.input_data_port_list_store[int(path)][0].name
        old_data_port = copy.copy(self.model.state.input_data_ports[key])
        del self.model.state.input_data_ports[key]
        print old_data_port
        #the text is the new key
        self.model.state.input_data_ports[text] = DataPort(text, old_data_port.data_type, old_data_port.default_value)
        self.model.update_input_data_port_list_store()
        self.view.get_top_widget().set_model(self.model.input_data_port_list_store)

    def on_data_type_changed(self, widget, path, text):
        print path
        old_data_port = self.model.input_data_port_list_store[int(path)][0]
        print old_data_port
        self.model.state.input_data_ports[old_data_port.name].data_type = text
        self.model.state.input_data_ports[old_data_port.name].default_value = None

    def on_default_value_changed(self, widget, path, text):
        print path
        old_data_port = self.model.input_data_port_list_store[int(path)][0]
        print old_data_port
        self.model.state.input_data_ports[old_data_port.name].default_value = text




