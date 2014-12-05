
from utils import log
logger = log.get_logger(__name__)

import gtk
from gtkmvc import Controller
from gtkmvc.adapters import UserClassAdapter


class StatePropertiesController(Controller):
    """Controller handling the table view of properties of the StateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK table view
    :class:`mvc.views.state_properties.StatePropertiesView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
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

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['window1'].connect('destroy', gtk.main_quit)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        self.adapt(self.__state_property_adapter("name", "input_name"))

    def __state_property_adapter(self, attr_name, label, view=None, value_error=None):
        """Helper method returning an adapter for a state property

        The method creates a custom adapter connecting a widget/label in the View with an attribute of the state model.
        :param attr_name: The name of the attribute
        :param label: The label of the widget element
        :param view: A reference to the view containing the widget. If left out, the view of the controller is used.
        :param value_error: An optional function handling a value_error exception. By default a debug message is
            print out and the widget value is updated to the previous value.
        :return: The custom created adapter, which can be used in :func:`register_adapter`
        """
        if view is None:
            view = self.view

        if value_error is None:
            def value_error(adapt, prop_name, value):
                print "Error:", adapt, prop_name, value
                adapt.update_widget()

        adapter = UserClassAdapter(self.model, "state",
                                   getter=lambda state: state.__getattribute__(attr_name),
                                   setter=lambda state, value: state.__setattr__(attr_name, value),
                                   value_error=value_error)
        adapter.connect_widget(view[label])
        return adapter

    @Controller.observe("state", before=True)
    def before_state_change(self, model, _, info):
        """Called before an attribute of the state is set

        The attributes of the state should all be set via function (setters). An observer observes these functions
        and calls this function before the actual setter call. It passes several parameters for information purpose.

        The function is empty at the moment (except a debug output), but can be filled with logic executed before a
        certain attribute is changed. The method contains a comment with example code.

        :param mvc.models.StateModel model: The model of the state being handled by the controller
        :param sm.State _: The state that was changed (can also be accessed via the model)
        :param info: Additional information, such as the method name that was called to change an attribute. With
            this method name, the property being changed can be determined. The parameter also contains the new desired
            value.

        """

        logger.debug("before_state_change -- Attribute: %s, before: %s, desired: %s",
                     info.method_name, model.state.__getattribute__(info.method_name), info.args[1])

        # The name of the method called th change the attribute should coincide with the attribute's name
        # attr = info.method_name
        #
        # if attr == "id"
        #     # The ID of the state is being changed
        #     pass
        # elif attr == "name"
        #     # The name of the state is being changed
        #     pass

        pass



    @Controller.observe("state", after=True)
    def after_state_change(self, model, _, info):
        """Called after an attribute of the state was set

        The attributes of the state should all be set via function (setters). An observer observes these functions
        and calls this function after the actual setter call. It passes several parameters for information purpose.

        The function is empty at the moment (except a debug output), but can be filled with logic executed before a
        certain attribute is changed. See :func:`before_state_change` for example code.

        :param mvc.models.StateModel model: The model of the state being handled by the controller
        :param sm.State _: The state that was changed (can also be accessed via the model)
        :param info: Additional information, such as the method name that was called to change an attribute. With
            this method name, the property being changed can be determined. The parameter also contains the new desired
            value and the return value of the setter function. By comparing the passed attribute with the current
            one, it can be determined whether the value was successfully changed or not.

        """
        logger.error("test")
        logger.debug("after_state_change -- Attribute: %s, after: %s, desired: %s, returned: %s",
                     info.method_name, model.state.__getattribute__(info.method_name), info.args[1], info.result)
