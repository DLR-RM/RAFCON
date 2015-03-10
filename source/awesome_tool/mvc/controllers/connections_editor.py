
from awesome_tool.utils import log
logger = log.get_logger(__name__)

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers.state_transitions import StateTransitionsListController, StateTransitionsEditorController
from awesome_tool.mvc.controllers.state_data_flows import StateDataFlowsListController, StateDataFlowsEditorController


class StateConnectionsEditorController(ExtendedController):

    """Controller handling the view of properties/attributes of the ContainerStateModel and StateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.state_properties.ContainerStateView` and the properties of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param awesome_tool.mvc.models.StateModel model: The state model containing the data
    :param awesome_tool.mvc.views.StatePropertiesView view: The GTK view showing the data as a table
    """

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        # self.transitions_ctrl = StateTransitionsListController(model, view.transitions_view)
        # self.data_flows_ctrl = StateDataFlowsListController(model, view.data_flows_view)
        self.add_controller('trans_editor_ctrl', StateTransitionsEditorController(model, view.transitions_view))
        self.add_controller('f_editor_ctrl', StateDataFlowsEditorController(model, view.data_flows_view))
        # self.view_dict = {'transitions_internal': True, 'transitions_external': True,
        #                   'data_flows_internal': True, 'data_flows_external': True}
        # self.data_flows_ctrl.view_dict = self.view_dict
        # self.transitions_ctrl.view_dict = self.view_dict

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        # view['add_t_button'].connect('clicked', self.transitions_ctrl.on_add)
        # #view['cancel_t_edit_button'].connect('clicked', self.on_cancel_transition_edit_clicked)
        # view['remove_t_button'].connect('clicked', self.transitions_ctrl.on_remove)
        # view['connected_to_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_external')
        # view['internal_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_internal')
        #
        # view['add_d_button'].connect('clicked', self.data_flows_ctrl.on_add)
        # #view['cancel_d_edit_button'].connect('clicked', self.on_cancel_dataflow_edit_clicked)
        # view['remove_d_button'].connect('clicked', self.data_flows_ctrl.on_remove)
        # view['connected_to_d_checkbutton'].connect('toggled', self.toggled_button, 'data_flows_external')
        # view['internal_d_checkbutton'].connect('toggled', self.toggled_button, 'data_flows_internal')
        #
        # if self.model.parent is None:
        #     self.view_dict['transitions_external'] = False
        #     view['connected_to_t_checkbutton'].set_active(False)
        #     self.view_dict['data_flows_external'] = False
        #     view['connected_to_d_checkbutton'].set_active(False)
        # if not hasattr(self.model, 'states'):
        #     self.view_dict['transitions_internal'] = False
        #     view['internal_t_checkbutton'].set_active(False)
        #     self.view_dict['data_flows_internal'] = False
        #     view['internal_d_checkbutton'].set_active(False)
        # # view['state_properties_view'].set_model(self.model.list_store)
        # #
        # # view['value_renderer'].connect('edited', self.__property_edited)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def toggled_button(self, button, name=None):
        pass
        # if name in ['transitions_external', 'data_flows_external'] and self.model.parent is not None:
        #     self.view_dict[name] = button.get_active()
        #     # print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
        # elif not name in ['transitions_internal', 'data_flows_internal']:
        #     self.view_dict['transitions_external'] = False
        #     self.view_dict['data_flows_external'] = False
        #     button.set_active(False)
        #
        # if name in ['transitions_internal', 'data_flows_internal'] and hasattr(self.model, 'states'):
        #     self.view_dict[name] = button.get_active()
        #     # print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
        # elif not name in ['transitions_external', 'data_flows_external']:
        #     self.view_dict['transitions_internal'] = False
        #     self.view_dict['data_flows_internal'] = False
        #     button.set_active(False)
        #
        # self.transitions_ctrl.update_transitions_store()
        # self.transitions_ctrl.update_stores()
        # self.transitions_ctrl.update_model()
        #
        # self.data_flows_ctrl.update_stores()
        # self.data_flows_ctrl.update_model()

    def __state_property_adapter(self, attr_name, label, view=None, value_error=None):
        """Helper method returning an adapter for a state property

        The method creates a custom adapter connecting a widget/label in the View with an attribute of the state model.

        :param attr_name: The name of the attribute
        :param label: The label of the widget element
        :param view: A reference to the view containing the widget. If left out, the view of the controller is used.
        :param val_error_fun: An optional function handling a value_error exception. By default a debug message is
            print out and the widget value is updated to the previous value.
        :return: The custom created adapter, which can be used in :func:`register_adapter`
        """
        # if view is None:
        #     view = self.view
        #
        # if value_error is None:
        #     value_error = self._value_error
        #
        # adapter = UserClassAdapter(self.model, "state",
        #                            getter=lambda state: state.__getattribute__(attr_name),
        #                            setter=lambda state, value: state.__setattr__(attr_name, value),
        #                            value_error=value_error)
        # adapter.connect_widget(view[label])
        # return adapter
    
    @staticmethod
    def _value_error(adapt, prop_name, value):
        logger.warning("Invalid value '{val:s}' for key '{prop:s}'.".format(val=value, prop=prop_name))
        adapt.update_widget()  # Update widget values with values from model

