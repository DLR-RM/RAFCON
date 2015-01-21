import gtk
from gtkmvc import View
from gtkmvc import Controller


class SingleWidgetWindowView(View):

    def __init__(self, view_class):
        View.__init__(self)

        w = gtk.Window()
        w.resize(width=500, height=500)
        self.widget_view = view_class()
        w.add(self.widget_view.get_top_widget())
        w.show_all()

        self['main_frame'] = self.widget_view
        self['main_window'] = w

    pass  # class end


class SingleWidgetWindowController(Controller):
    """Controller handling the view of properties/attributes of ...
    """

    def __init__(self, model, view, ctrl_class):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.widget_ctrl = ctrl_class(model, view['main_frame'])

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['main_window'].connect('destroy', gtk.main_quit)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))
    pass  # class end


if __name__ == '__main__':
    from mvc.views.source_editor import SourceEditorView
    from mvc.controllers.source_editor import SourceEditorController

    from statemachine.states.execution_state import ExecutionState as State
    from mvc.models import StateModel, ContainerStateModel
    state1 = State('Rico2')
    m = StateModel(state1)

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(SourceEditorView)
    #c = SingleWidgetWindowController(m, v, SourceEditorController)
    c = SingleWidgetWindowController(ctr_model, v, SourceEditorController)

    # from mvc.views.state_editor import StateEditorView
    # from mvc.controllers.state_editor import StateEditorController
    # w = gtk.Window()
    # v = StateEditorView()
    # c = StateEditorController(m, v)
    # w.add(v.get_top_widget())#['main_frame'])
    # w.show_all()
    print "init gtk main"

    gtk.main()