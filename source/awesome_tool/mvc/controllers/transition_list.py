
from utils import log
logger = log.get_logger(__name__)

import gtk
from gtkmvc import Controller
from gtkmvc.adapters import UserClassAdapter
from gtk import ListStore


class TransitionListController(Controller):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.state_properties.ContainerStateView` and the properties of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.StatePropertiesView view: The GTK view showing the data as a table
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
            states_dic = {-1: 'Parent'}
            states_store = ListStore(str, str)
            states_store.append([container_model.container_state.state_id, container_model.container_state.name])
            for state_model in container_model.states:
                states_dic[state_model.state.state_id] = state_model.state.name
                states_store.append([state_model.state.state_id, state_model.state.name])
            if col == 'from_state_col':
                cell_renderer.set_property('text', states_dic[model.get_value(iter, 0).from_state])
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'to_state_col':
                cell_renderer.set_property('text', states_dic[model.get_value(iter, 0).to_state])
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)

        view.get_top_widget().set_model(self.model.transition_list_store)

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)

        # self['transition_list_view'].set_model(model)
        #
        # self['from_state_col'].add_attribute(self['from_state_combo'], "text", 1)
        # self['from_state_combo'].set_property("text-column", 1)
        # self['from_state_combo'].set_property("model", states)
        #
        # self['to_state_combo'].set_property("text", 0)
        # self['to_state_combo'].set_property("model", states)
        # self['to_state_col'].add_attribute(self['to_state_combo'], "text", 0)
        # self['to_state_combo'].set_property("editable", True)
        # self['to_state_combo'].set_property("text-column", 0)


        view['from_state_combo'].connect("edited", self.on_combo_changed)
        view['to_state_combo'].connect("edited", self.on_combo_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))


    def on_combo_changed(self, widget, path, text):
        #print widget.get_model()
        #print widget.get_model()[path]
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

    # def __property_edited(self, _, row, value):
    #     outcome = self.model.update_row(row, value)
    #     if type(outcome) != bool:
    #         logger.warning("Invalid value: %s" % outcome)
    #
    # def __state_property_adapter(self, attr_name, label, view=None, value_error=None):
    #     """Helper method returning an adapter for a state property
    #
    #     The method creates a custom adapter connecting a widget/label in the View with an attribute of the state model.
    #
    #     :param attr_name: The name of the attribute
    #     :param label: The label of the widget element
    #     :param view: A reference to the view containing the widget. If left out, the view of the controller is used.
    #     :param val_error_fun: An optional function handling a value_error exception. By default a debug message is
    #         print out and the widget value is updated to the previous value.
    #     :return: The custom created adapter, which can be used in :func:`register_adapter`
    #     """
    #     if view is None:
    #         view = self.view
    #
    #     if value_error is None:
    #         value_error = self._value_error
    #
    #     adapter = UserClassAdapter(self.model, "state",
    #                                getter=lambda state: state.__getattribute__(attr_name),
    #                                setter=lambda state, value: state.__setattr__(attr_name, value),
    #                                value_error=value_error)
    #     adapter.connect_widget(view[label])
    #     return adapter
    #
    # @staticmethod
    # def _value_error(adapt, prop_name, value):
    #     logger.warning("Invalid value '{val:s}' for key '{prop:s}'.".format(val=value, prop=prop_name))
    #     adapt.update_widget()  # Update widget values with values from model
    #
    # @Controller.observe("state", before=True)
    # def before_state_change(self, model, _, info):
    #     """Called before an attribute of the state is set
    #
    #     The attributes of the state should all be set via function (setters). An observer observes these functions
    #     and calls this function before the actual setter call. It passes several parameters for information purpose.
    #
    #     The function is empty at the moment (except a debug output), but can be filled with logic executed before a
    #     certain attribute is changed. The method contains a comment with example code.
    #
    #     :param mvc.models.StateModel model: The model of the state being handled by the controller
    #     :param sm.State _: The state that was changed (can also be accessed via the model)
    #     :param info: Additional information, such as the method name that was called to change an attribute. With
    #         this method name, the property being changed can be determined. The parameter also contains the new desired
    #         value.
    #
    #     """
    #
    #     logger.debug("before_state_change -- Attribute: %s, before: %s, desired: %s",
    #                  info.method_name, model.state.__getattribute__(info.method_name), info.args[1])
    #
    #     # The name of the method called th change the attribute should coincide with the attribute's name
    #     # attr = info.method_name
    #     #
    #     # if attr == "id"
    #     #     # The ID of the state is being changed
    #     #     pass
    #     # elif attr == "name"
    #     #     # The name of the state is being changed
    #     #     pass
    #
    #     pass
    #
    #
    #
    # @Controller.observe("state", after=True)
    # def after_state_change(self, model, _, info):
    #     """Called after an attribute of the state was set
    #
    #     The attributes of the state should all be set via function (setters). An observer observes these functions
    #     and calls this function after the actual setter call. It passes several parameters for information purpose.
    #
    #     The function is empty at the moment (except a debug output), but can be filled with logic executed before a
    #     certain attribute is changed. See :func:`before_state_change` for example code.
    #
    #     :param mvc.models.StateModel model: The model of the state being handled by the controller
    #     :param sm.State _: The state that was changed (can also be accessed via the model)
    #     :param info: Additional information, such as the method name that was called to change an attribute. With
    #         this method name, the property being changed can be determined. The parameter also contains the new desired
    #         value and the return value of the setter function. By comparing the passed attribute with the current
    #         one, it can be determined whether the value was successfully changed or not.
    #
    #     """
    #
    #     logger.debug("after_state_change -- Attribute: %s, after: %s, desired: %s, returned: %s",
    #                  info.method_name, model.state.__getattribute__(info.method_name), info.args[1], info.result)
    #
    #     if model.state.__getattribute__(info.method_name) == info.args[1]:  # Change was successful
    #
    #         if self.view is None:  # View hasn't been created, yet
    #             self.model.update_attributes()
    #
    #         # If the view has been created, store the current selection of the table and restore the selection,
    #         # after the table has been updated. This is needed, as the selection is lost when the table is cleared.
    #         else:
    #             view = self.view['state_properties_view']
    #
    #             selection = view.get_selection()
    #             (paths, _) = selection.get_selected_rows()
    #             selected_paths = []
    #             for path in paths:
    #                 if selection.path_is_selected(path.path):
    #                     selected_paths.append(path.path)
    #
    #             self.model.update_attributes()
    #
    #             for path in selected_paths:
    #                 selection.select_path(path)

