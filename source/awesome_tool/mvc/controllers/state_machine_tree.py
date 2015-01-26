import gtk
from gtkmvc import Controller
from gtkmvc import Observer
from mvc.models import ContainerStateModel

#TODO: comment

class StateMachineTreeController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.model = model
        self.tree_store = gtk.TreeStore(str, str, str)
        view.set_model(self.tree_store)

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_select)
        self.update()

    def register_adapters(self):
        pass

    def update(self):
        self.tree_store.clear()
        parent = self.tree_store.insert_before(None, None,
                                      (self.model.state.name,
                                        self.model.state.state_id,
                                        self.model.state.state_type))
        for state_id, smodel in self.model.states.items():
            self.insert_rec(parent, smodel)

    def insert_rec(self, parent, state_model):
        #print 'Inserting %s' % str((state.title, state.id, utils.const2str(Point.Point, state.type)))
        parent = self.tree_store.insert_before(parent, None,
                                               (state_model.state.name,
                                                state_model.state.state_id,
                                                state_model.state.state_type))
        if type(state_model) is ContainerStateModel:
            #print "Inserte container state %s recursively" % state_model.state.name
            for state_id, smodel in state_model.states.items():
                self.insert_rec(parent, smodel)

    def on_select(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        print "SM_Tree state selected: %s, %s" % (model, row)
        #selected_state = self.model.statemachine.get_graph().find_node(model.get_value(row, 1))
        #self.model.statemachine.selection.set([selected_state])

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        self.update()

if __name__ == '__main__':
    from mvc.views import StateMachineTreeView, SingleWidgetWindowView
    from mvc.controllers import SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(StateMachineTreeView, width=500, height=200, title='State Machine Tree')
    c = SingleWidgetWindowController(ctr_model, v, StateMachineTreeController)

    gtk.main()