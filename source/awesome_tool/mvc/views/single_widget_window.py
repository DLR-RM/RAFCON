import gtk
from gtkmvc import View
from gtkmvc import Controller
from statemachine.states.execution_state import ExecutionState


class SingleWidgetWindowView(View):

    def __init__(self, view_class, width=500, height=500, title=None, pos=None):
        View.__init__(self)

        w = gtk.Window()
        if title is None:
            w.set_title(str(view_class))
        else:
            w.set_title(title)
        w.resize(width=width, height=height)
        if pos is not None:
            w.set_position(pos)
        self.widget_view = view_class()
        w.add(self.widget_view.get_top_widget())
        w.show_all()

        self['main_frame'] = self.widget_view
        self['main_window'] = w

    pass  # class end


class TestButtonsView(View, Controller):

    top = 'window_with_test_buttons'

    def __init__(self, model):

        View.__init__(self)
        Controller.__init__(self, model, self)
        self.model = model

        window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        window.set_property("default-height", 100)
        window.set_property("default-width", 400)
        window.connect("destroy", self.destroy)
        window.set_border_width(10)

        button1 = gtk.Button("Button 1")
        button1.connect("clicked", self.on_button1_pressed, None)
        button1.show()

        button2 = gtk.Button("Button 2")
        button2.connect("clicked", self.on_button2_pressed, None)
        button2.show()

        button3 = gtk.Button("Button 3")
        button3.connect("clicked", self.on_button3_pressed, None)
        button3.show()

        hbox = gtk.HBox(False, 0)
        hbox.pack_start(button1, True, True, 0)
        hbox.pack_start(button2, True, True, 0)
        hbox.pack_start(button3, True, True, 0)
        window.add(hbox)
        hbox.show()
        window.show()
        self['window_with_test_buttons'] = window

    def destroy(self, widget, data=None):
        print "destroy signal occurred"
        gtk.main_quit()

    def on_button1_pressed(self, widget, data=None):
        print "Pressed button1"
        execution_state = ExecutionState("Test Execution State")
        self.model.state.add_state(execution_state)

    def on_button2_pressed(self, widget, data=None):
        print "Pressed button2"

    def on_button3_pressed(self, widget, data=None):
        print "Pressed button3"

    def register_view(self, view):
        """Called when the View was registered
        """

    def register_adapters(self):
        """Adapters should be registered in this method call
        """


if __name__ == '__main__':
    from mvc.views.source_editor import SourceEditorView
    from mvc.controllers import SourceEditorController, SingleWidgetWindowController

    from statemachine.states.execution_state import ExecutionState as State
    from mvc.models import StateModel, ContainerStateModel
    state1 = State('Rico2')
    m = StateModel(state1)

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(SourceEditorView, title='Source Editor')
    c = SingleWidgetWindowController(ctr_model, v, SourceEditorController)

    gtk.main()