import gtk, gobject

from gtkmvc import Controller, Observer


class StateOutcomesTreeController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.tree_store = view.tree_store

        self.to_state_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)
        self.to_outcome_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)
        self.list_to_other_state = {}
        self.list_to_other_outcome = {}
        self.list_from_other_state = {}
        self.update_stores()

    def register_view(self, view):
        """Called when the View was registered
        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            outcome = model.get_value(iter, 6)
            state = model.get_value(iter, 7)
            if column.get_title() == 'ID':
                if int(outcome.outcome_id) < 0:
                    cell_renderer.set_property('editable', False)
                #cell_renderer.set_property('text', outcome.outcome_id)
            elif column.get_title() == 'Name':
                if int(outcome.outcome_id) < 0:
                    cell_renderer.set_property('editable', False)
                else:
                    cell_renderer.set_property('editable', True)
                #cell_renderer.set_property('text', outcome.name)
            elif column.get_title() == 'To-State':
                cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_state_combo_list)
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
                #print "to_state: ", type(cell_renderer)
                # find to state by from_state == model.state.state_id and from_key == outcome.name
            elif column.get_title() == 'To-Outcome':
                cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_outcome_combo_list)
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
                #print "to_outcome: ", type(cell_renderer)
                # find to outcome by from_outcome == model.state.state_id and from_outcome == outcome.name
            else:
                print "not allowed", column.get_name(), column.get_title()

        self.update_model()

        view['tree_view'].set_model(self.tree_store)
        view['id_col'].set_cell_data_func(view['id_cell'], cell_text, self.model)
        view['name_col'].set_cell_data_func(view['name_cell'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text, self.model)

        view['name_cell'].connect('edited', self.on_name_modification)
        view['to_state_combo'].connect("edited", self.on_to_state_modification)
        view['to_outcome_combo'].connect("edited", self.on_to_outcome_modification)

    def on_name_modification(self, widget, path, text):
        self.model.model_changed(self.model, 'outcome.name', 'before')
        self.tree_store[path][1] = text
        self.tree_store[path][6].name = text
        self.model.model_changed(self.model, 'outcome.name', 'after')
        print "change name of outcome: ", path, self.tree_store[path][6].name

    def on_to_state_modification(self, widget, path, text):
        print "change to state: ", path

    def on_to_outcome_modification(self, widget, path, text):
        print "change to outcome: ", path

    def on_add(self, button, info=None):
        print "add outcome"
        outcome_id = self.model.state.add_outcome('success' + str(len(self.model.state.outcomes)-1))
        outcome = self.model.state.outcomes[outcome_id]
        self.tree_store.append(None, [outcome_id, outcome.name, '', '', '#f0E5C7', '#f0E5c7', outcome, self.model.state])
        #self.update_path()

    def on_remove(self, button, info=None):
        print "remove outcome"
        print self.model.state.outcomes
        # self.update_path()
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        #print path, tree
        tree, iter = self.view.tree_view.get_selection().get_selected()
        if path and not self.tree_store[path[0][0]][6].outcome_id < 0:
            outcome_id = self.tree_store[path[0][0]][6].outcome_id
            if outcome_id in self.list_to_other_outcome:
                elem = self.list_to_other_outcome[outcome_id]
                transition = self.model.parent.state.transitions[elem[2]]  # transition by transition_id
                print "remove_to_other_outcome: ", elem[2], transition.transition_id, self.model.parent.state.name, \
                    transition.to_outcome, transition.to_state, transition.to_outcome, transition.to_state
                self.model.parent.state.remove_transition(transition.transition_id)
            if outcome_id in self.list_to_other_state:
                elem = self.list_to_other_state[outcome_id]
                transition = self.model.parent.state.transitions[elem[2]]  # transition by transition_id
                print "remove_to_other_state: ", elem[2], transition.transition_id, self.model.parent.state.name, \
                    transition.from_outcome, transition.from_state, transition.to_outcome, transition.to_state
                self.model.parent.state.remove_transition(transition.transition_id)
            if outcome_id in self.list_from_other_state:
                for elem in self.list_from_other_state[outcome_id]:
                    transition = self.model.state.transitions[elem[2]]  # transition by transition_id
                    print "remove_from_other_state: ", elem[2], transition.transition_id, self.model.state.name, \
                        transition.to_outcome, transition.to_state
                    self.model.state.remove_transition(transition.transition_id)

            self.model.state.remove_outcome(outcome_id)
            parent = self.tree_store.remove(iter)
            print path, parent, tree, iter
        print self.model.state.outcomes

    def update_stores(self):

        model = self.model

        print "clean stores"
        self.to_state_combo_list.clear()
        self.to_outcome_combo_list.clear()
        self.list_to_other_state.clear()
        self.list_to_other_outcome.clear()
        self.list_from_other_state.clear()

        if hasattr(model, 'parent') and model.parent is not None:
            # check for "to state combos" -> so all states in parent
            parent_id = model.parent.state.state_id
            for smdl in model.parent.states.values():
                if not model.state.state_id == smdl.state.state_id:
                    self.to_state_combo_list.append([smdl.state.name, smdl.state.state_id, parent_id])
            # check for "to outcome combos" -> so all outcomes of parent
            for outcome in model.parent.state.outcomes.values():
                print "type outcome: ", type(outcome)
                self.to_outcome_combo_list.append([outcome.name, outcome.outcome_id, parent_id])
            for transition_id, transition in model.parent.state.transitions.items():
                print transition.from_state, transition.from_outcome, \
                        transition.to_state, transition.to_outcome, model.parent.state.name, \
                    model.parent.state.state_id, \
                    transition_id, transition.transition_id, model.parent.state.transitions[transition_id].transition_id
                # check for "to other state" connections -> so from self-state and self-outcome "external" transitions
                if transition.from_state == model.state.state_id and transition.from_outcome in model.state.outcomes.keys():
                    # check for "to other outcomes" connections -> so to parent-state and parent-outcome "ext" transitions
                    if transition.to_state is None:
                        to_state_name = model.parent.state.name
                        to_state_id = model.parent.state.state_id
                        self.list_to_other_outcome[transition.from_outcome] = [to_state_name + '.' + str(transition.to_outcome),
                                                                               to_state_id,
                                                                               transition.transition_id]
                    else:  # or to other state
                        to_state_name = model.parent.states[transition.to_state].state.name
                        self.list_to_other_state[transition.from_outcome] = [to_state_name + '.' + transition.to_state,
                                                                             '',
                                                                             transition.transition_id]
        if hasattr(model.state, 'transitions'):
            # check for "from other state" connections -> so to self-state and self-outcome "internal" transitions
            for transition_id, transition in model.state.transitions.items():
                print transition.from_state, transition.from_outcome, \
                        transition.to_state, transition.to_outcome, model.state.name, model.state.state_id, \
                    transition_id, transition.transition_id
                if transition.to_state is None:  # no to_state means self
                    if transition.to_outcome in self.list_from_other_state:
                        self.list_from_other_state[transition.to_outcome].append([transition.from_state, transition.from_outcome, transition.transition_id])
                    else:
                        self.list_from_other_state[transition.to_outcome] = [[transition.from_state, transition.from_outcome, transition.transition_id]]

        print "to_state: ", self.list_to_other_state
        print "to_outcome: ", self.list_to_other_outcome
        print "from state: ", self.list_from_other_state
        print "state.name: ", self.model.state.name

    def update_model(self):

        self.tree_store.clear()
        for outcome in self.model.state.outcomes.values():
            to_state = None
            if outcome.outcome_id in self.list_to_other_state.keys():
                to_state = self.list_to_other_state[outcome.outcome_id][0]
            to_outcome = None
            if outcome.outcome_id in self.list_to_other_outcome.keys():
                to_outcome = self.list_to_other_outcome[outcome.outcome_id][0]
                to_state = 'parent'
            from_state = None
            if outcome.outcome_id in self.list_from_other_state.keys():
                from_state = self.list_from_other_state[outcome.outcome_id][0]
            print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]
            self.tree_store.append(None, [outcome.outcome_id, outcome.name, to_state, to_outcome,
                                          '#f0E5C7', '#f0E5c7', outcome, self.model.state])

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        if info.method_name == "add_outcome" or info.method_name == "remove_outcome":
            self.update_stores()
            self.update_model()
        elif info.method_name == "add_transition" or info.method_name == "remove_transition":
            self.update_stores()

    # @Observer.observe("", after=True)
    # def assign_notification_state(self, model, prop_name, info):
    #     print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
    #           (prop_name, info.instance, info.method_name, info.result)
    #     self.update_stores()
    #     self.update_model()

    # @Observer.observe("parent.state", after=True)
    # def assign_notification_state(self, model, prop_name, info):
    #     print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
    #           (prop_name, info.instance, info.method_name, info.result)
    #     if info.method_name == "add_transition" or info.method_name == "remove_transition":
    #         self.update_stores()


class StateOutcomesEditorController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.tree_ctrl = StateOutcomesTreeController(model, view.treeView)

    def register_view(self, view):
        """Called when the View was registered
        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """

        view['add_button'].connect("clicked", self.tree_ctrl.on_add)
        view['remove_button'].connect("clicked", self.tree_ctrl.on_remove)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """

if __name__ == '__main__':
    from mvc.controllers import SingleWidgetWindowController
    from mvc.views import SingleWidgetWindowView, StateOutcomesEditorView, StateOutcomesTreeView

    import mvc.main as main
    import gtk

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    #tree_view = StateOutcomesTreeView()
    #tree_ctrl = StateOutcomesTreeController(ctr_model, tree_view)
    v = SingleWidgetWindowView(StateOutcomesEditorView, width=500, height=200, title='Outcomes Editor')
    c = SingleWidgetWindowController(ctr_model.states.values()[1], v, StateOutcomesEditorController)

    gtk.main()