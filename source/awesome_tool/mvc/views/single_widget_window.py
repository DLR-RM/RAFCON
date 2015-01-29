import gtk
from gtkmvc import View
from gtkmvc import Controller
from statemachine.states.execution_state import ExecutionState
import statemachine.singleton


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
        window.set_property("default-height", 200)
        window.set_property("default-width", 500)
        window.connect("destroy", self.destroy)
        window.set_border_width(10)

        top_vbox = gtk.VBox(False, 0)

        #################################################################################
        # Execution buttons
        #################################################################################

        start_button = gtk.Button("Start")
        start_button.connect("clicked", self.on_button3_pressed, None)
        start_button.show()

        stop_button = gtk.Button("Stop")
        stop_button.connect("clicked", self.on_button4_pressed, None)
        stop_button.show()

        pause_button = gtk.Button("Pause")
        pause_button.connect("clicked", self.on_button5_pressed, None)
        pause_button.show()

        step_mode_button = gtk.Button("Step-Mode")
        step_mode_button.connect("clicked", self.on_button6_pressed, None)
        step_mode_button.show()

        step_button = gtk.Button("Step")
        step_button.connect("clicked", self.on_button2_pressed, None)
        step_button.show()

        execution_hbox = gtk.HBox(True, 10)
        execution_hbox.pack_start(start_button, True, True, 0)
        execution_hbox.pack_start(stop_button, True, True, 0)
        execution_hbox.pack_start(pause_button, True, True, 0)
        execution_hbox.pack_start(step_mode_button, True, True, 0)
        execution_hbox.pack_start(step_button, True, True, 0)

        execution_label = gtk.Label("Execution Buttons")
        execution_label.show()
        top_vbox.pack_start(execution_label, True, False, 0)
        top_vbox.pack_start(execution_hbox, False, False, 0)

        #################################################################################
        # State modification buttons
        #################################################################################

        state_modification_label = gtk.Label("State modification Buttons")
        state_modification_label.show()
        top_vbox.pack_start(state_modification_label, True, False, 0)
        add_state_button = gtk.Button("Add state")
        add_state_button.connect("clicked", self.on_add_state_button_pressed, None)
        add_state_button.show()
        top_vbox.pack_start(add_state_button, False, False, 0)

        data_port_change_button = gtk.Button("Change name of data port")
        data_port_change_button.connect("clicked", self.on_data_port_change_button_pressed, None)
        data_port_change_button.show()
        top_vbox.pack_start(data_port_change_button, False, False, 0)

        scoped_variable_changed_button = gtk.Button("Change name of scoped variable")
        scoped_variable_changed_button.connect("clicked", self.on_scoped_variable_change_button_pressed, None)
        scoped_variable_changed_button.show()
        top_vbox.pack_start(scoped_variable_changed_button, False, False, 0)

        window.add(top_vbox)
        top_vbox.show()
        execution_hbox.show()
        window.show()
        self['window_with_test_buttons'] = window

    def destroy(self, widget, data=None):
        print "destroy signal occurred"
        gtk.main_quit()

    def on_add_state_button_pressed(self, widget, data=None):
        print "Add state button pressed"
        execution_state = ExecutionState("Test Execution State")
        self.model.state.add_state(execution_state)

    def on_data_port_change_button_pressed(self, widget, data=None):
        print "Data port change button pressed"
        key = self.model.state.input_data_ports.keys()[0]
        self.model.state.input_data_ports[key].name = "New name for input port"

    def on_scoped_variable_change_button_pressed(self, widget, data=None):
        print "Scoped variable change button pressed"
        key = self.model.state.scoped_variables.keys()[0]
        self.model.state.scoped_variables[key].name = "New name for scoped variable"

    def on_button2_pressed(self, widget, data=None):
        print "Step button pressed"
        statemachine.singleton.state_machine_execution_engine.step()

    def on_button3_pressed(self, widget, data=None):
        print "Start button pressed"
        statemachine.singleton.state_machine_execution_engine.start()

    def on_button4_pressed(self, widget, data=None):
        print "Stop button pressed"
        statemachine.singleton.state_machine_execution_engine.stop()

    def on_button5_pressed(self, widget, data=None):
        print "Pause button pressed"
        statemachine.singleton.state_machine_execution_engine.pause()

    def on_button6_pressed(self, widget, data=None):
        print "Step-mode button pressed"
        statemachine.singleton.state_machine_execution_engine.step_mode()

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