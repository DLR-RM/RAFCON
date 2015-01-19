from gtkmvc import Controller


class SourceEditorController(Controller):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.source_editor.SourceEditorView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.SourceEditorView view: The GTK view showing the data as a table
    """

    # TODO Missing functions
    # - Code syntax check
    # - Code wrapping
    # - Code completion

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view.get_buffer().connect('changed', self.code_changed)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)
        view.set_text(self.model.state.script.script)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    #===============================================================
    def code_changed(self, source):
        self.view.apply_tag('default')

    #===============================================================
    def apply_clicked(self, button):
        #self.model.state.script = self.view.get_buffer()
        # bypath the observer
        # self.model.state.script.script = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        tbuffer = self.view.get_buffer()
        script = self.model.state.script
        script.script = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        self.model.state.script = script

        self.view.set_text(self.model.state.script.script)

    #===============================================================
    def cancel_clicked(self, button):
        self.view.set_text(self.model.state.script.script)

