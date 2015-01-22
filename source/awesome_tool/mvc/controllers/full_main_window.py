import gtk
from gtkmvc import Controller
from mvc.controllers import StateEditorController, LibraryTreeController, StateMachineTreeController, GraphicalEditorController

from gtkmvc import Observer


class FullMainWindowController(Controller):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.source_editor.SourceEditorView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.SourceEditorView view: The GTK view showing the data as a table
    """

    # TODO Missing functions

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        self.library_tree_ctrl = LibraryTreeController(model, view.library_tree)
        self.state_machine_tree_ctrl = StateMachineTreeController(model, view.state_machine_tree)
        self.state_editors_ctrl = StateEditorController(model, view.state_editors)
        self.state_machines_ctrl = GraphicalEditorController(model, view.state_machines_editors)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        # connect toolbar

        # connect keys

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))


if __name__ == '__main__':
    from mvc.views.full_main_window import FullMainWindowView

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()
    # TODO create new model to control and edit multiple statemachines

    #w = gtk.Window()
    v = FullMainWindowView()
    c = FullMainWindowController(ctr_model, v)
    #w.add(v.get_top_widget())#['main_frame'])
    #w.show_all()

    gtk.main()