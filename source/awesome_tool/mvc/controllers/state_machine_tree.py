import gtk
from gtkmvc import Controller
from mvc.models import ContainerStateModel


class StateMachineTreeController(Controller):
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
        self.tree_store = gtk.TreeStore(str, str, str)
        view.set_model(self.tree_store)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        self.view.connect('cursor-changed', self.on_select)
        self.update()

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def update(self):
        #self.statemachine = sm_model.statemachine
        sm_model = self.model
        s_model = sm_model

        self.tree_store.clear()

        for state_id, smodel in s_model.states.items():
            self.insert_rec(None, smodel)

    def insert_rec(self, parent, state_model):
        #print 'Inserting %s' % str((state.title, state.id, utils.const2str(Point.Point, state.type)))
        parent = self.tree_store.insert_before(parent, None,
                                               (state_model.state.name,
                                                state_model.state.state_id,
                                                state_model.state.state_type))
        print "I use this model %s" % state_model
        if type(state_model) is ContainerStateModel:
            print "found container state: %s" % state_model.state.name
            for state_id, smodel in state_model.states.items():
                self.insert_rec(parent, smodel)

    def on_select(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        print "SM_Tree state selected: %s, %s" % (model, row)
        #selected_state = self.model.statemachine.get_graph().find_node(model.get_value(row, 1))

        #self.model.statemachine.selection.set([selected_state])


if __name__ == '__main__':
    from mvc.views import StateMachineTreeView, SingleWidgetWindowView
    from mvc.controllers import SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(StateMachineTreeView, width=500, height=200, title='State Machine Tree')
    c = SingleWidgetWindowController(ctr_model, v, StateMachineTreeController)

    gtk.main()