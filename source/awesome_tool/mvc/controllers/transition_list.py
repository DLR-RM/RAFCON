
from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller, Observer
import gtk
import gobject

from mvc.models import ContainerStateModel, StateModel


class ParentObserver(Observer):

    def __init__(self, model, key, funct_handle_list):
        Observer.__init__(self, model)
        self.func_handle_list = funct_handle_list
        # self.observe(self.notify, "state", after=True)
        self.method_list = ["add_transition", "remove_transition",
                            "add_outcome", "remove_outcome",
                            "add_state", "remove_state", "modify_outcome_name"]

    @Observer.observe('state', after=True)
    def notification(self, model, prop_name, info):
        print "parent call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        #if info.method_name in self.method_list:
        for func_handle in self.func_handle_list:
            func_handle()


class TransitionListController(Controller):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.transitions.TransitionListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.ContainerStateModel model: The container state model containing the data
    :param mvc.views.TransitionListView view: The GTK view showing the transitions as a table
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        if model.parent is not None:
            self.parent_observer = ParentObserver(model.parent, "state", [self.update_stores, self.update_model])
            #self.parent_observer.observe(model.parent)

        # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
        #                   name-color, to-state-color, transition-object, state-object, is_editable
        self.tree_store = gtk.TreeStore(int, str, str, str, str, bool,
                                        str, str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT, bool)
        self.combo = {}

        self.update_stores()
        self.new_version = True
        self.view_dict = {}
        view.get_top_widget().set_model(self.tree_store)

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            t_id = model.get_value(iter, 0)
            state = model.get_value(iter, 9)
            in_external = 'internal'
            if model.get_value(iter, 5):
                in_external = 'external'
            #print t_id, in_external, self.combo[in_external]
            if column.get_title() == 'From-State':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['from_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'From-Outcome':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['from_outcome'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'To-State':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['to_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'To-Outcome':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['to_outcome'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
                #print "to_outcome: ", type(cell_renderer)
                # find to outcome by from_outcome == model.state.state_id and from_outcome == outcome.name
            else:
                print "not allowed", column.get_name(), column.get_title()

        self.update_model()

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_outcome_col'].set_cell_data_func(view['from_outcome_combo'], cell_text, self.model)
        view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text, self.model)

        view['from_state_combo'].connect("edited", self.on_combo_changed_from_state)
        view['from_outcome_combo'].connect("edited", self.on_combo_changed_from_outcome)
        view['to_state_combo'].connect("edited", self.on_combo_changed_to_state)
        view['to_outcome_combo'].connect("edited", self.on_combo_changed_to_outcome)

        #view['external_toggle'].set_radio(True)
        view['external_toggle'].connect("toggled", self.on_external_toggled)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_external_toggled(self, widget, path):
        logger.debug("Widget: {widget:s} - Path: {path:s}".format(widget=widget, path=path))

        if widget.get_active():
            print "change to INTERNAL ", path
            #self.tree_store[path][5] = False
        else:
            print "change to EXTERNAL ", path
            #self.tree_store[path][5] = True

    def on_add(self, button, info=None):
        print "add transition"
        print type(self.combo['free_from_state_models']), self.combo['free_from_state_models'], '\n', self.combo['free_from_outcomes_dict']
        if self.view_dict['transitions_internal'] and len(self.combo['free_from_outcomes_dict']) > 0:
            from_state_id = self.combo['free_from_outcomes_dict'].keys()[0]
            from_outcome = self.combo['free_from_outcomes_dict'][from_state_id][0].outcome_id
            to_state_id = None  # self.model.state.state_id
            to_outcome = self.model.state.outcomes.values()[0].outcome_id
            print "NEW TRANSITION INTERNAL IS: ", from_state_id, from_outcome, to_state_id, to_outcome
            transition_id = self.model.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            print "NEW TRANSITION INTERNAL IS: ", self.model.state.transitions[transition_id]

        elif self.view_dict['transitions_external']:
            from_state_id = self.model.state.state_id
            from_outcome = self.model.state.outcomes.values()[0].outcome_id
            to_state_id = None  # self.model.parent.state.state_id
            to_outcome = self.model.parent.state.outcomes.values()[0].outcome_id
            print "NEW TRANSITION EXTERNAL IS: ", from_state_id, from_outcome, to_state_id, to_outcome
            transition_id = self.model.parent.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            print "NEW TRANSITION EXTERNAL IS: ", self.model.parent.state.transitions[transition_id]
        else:
            print "NO OPTION TO ADD TRANSITION"

    def on_remove(self, button, info=None):
        print "remove transition"
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        print path, tree
        if path:
            print "Transition: ", self.tree_store[path[0][0]][0]
            transition_id = self.tree_store[path[0][0]][0]
            if self.tree_store[path[0][0]][5]:
                print self.model.parent.state.transitions
                self.model.parent.state.remove_transition(int(transition_id))
            else:
                self.model.state.remove_transition(int(transition_id))

    def on_combo_changed_from_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        #self.combo['free_from_outcomes_dict']
        text = text.split('.')
        t_id = self.tree_store[path][0]
        t = self.tree_store[path][8]
        fs = t.from_state
        ts = t.to_state
        to = t.to_outcome
        if self.tree_store[path][5]:  # external
            fo = self.combo['free_ext_from_outcomes_dict'][text[-1]][0].outcome_id
            self.model.parent.state.remove_transition(t_id)
            self.model.parent.state.add_transition(text[-1], fo, ts, to, transition_id=t_id)
        else:
            fo = self.combo['free_from_outcomes_dict'][text[-1]][0].outcome_id
            self.model.state.remove_transition(t_id)
            self.model.state.add_transition(text[-1], fo, ts, to, transition_id=t_id)

    def on_combo_changed_from_outcome(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        text = text.split('.')
        t_id = self.tree_store[path][0]
        t = self.tree_store[path][8]
        fs = t.from_state
        fo = t.from_outcome
        ts = t.to_state
        to = t.to_outcome
        if self.tree_store[path][5]:  # external
            self.model.parent.state.remove_transition(t_id)
            self.model.parent.state.add_transition(fs, int(text[-1]), ts, to, transition_id=t_id)
        else:
            self.model.state.remove_transition(t_id)
            self.model.state.add_transition(fs, int(text[-1]), ts, to, transition_id=t_id)

    def on_combo_changed_to_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        #self.combo['free_ext_from_outcomes_dict']
        text = text.split('.')
        t_id = self.tree_store[path][0]
        t = self.tree_store[path][8]
        fs = t.from_state
        fo = t.from_outcome
        if self.tree_store[path][5]:  # external
            self.model.parent.state.remove_transition(t_id)
            self.model.parent.state.add_transition(fs, fo, to_state_id=text[-1], transition_id=t_id)
        else:
            self.model.state.remove_transition(t_id)
            self.model.state.add_transition(fs, fo, to_state_id=text[-1], transition_id=t_id)

    def on_combo_changed_to_outcome(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        text = text.split('.')
        t_id = self.tree_store[path][0]
        t = self.tree_store[path][8]
        fs = t.from_state
        fo = t.from_outcome
        if self.tree_store[path][5]:  # external
            self.model.parent.state.remove_transition(t_id)
            self.model.parent.state.add_transition(fs, fo, to_outcome=int(text[-1]), transition_id=t_id)
        else:
            self.model.state.remove_transition(t_id)
            self.model.state.add_transition(fs, fo, to_outcome=int(text[-1]), transition_id=t_id)

    def get_free_state_combos_for_transition(self, trans, model, self_model, external=False):
        from_state_combo = gtk.ListStore(str)
        from_outcome_combo = gtk.ListStore(str)
        to_state_combo = gtk.ListStore(str)
        to_outcome_combo = gtk.ListStore(str)

        trans_dict = model.state.transitions

        # for from-outcome-combo filter all outcome already used
        from_state = model.states[trans.from_state].state
        free_from_outcomes_dict = {}
        for state_model in model.states.values():
            from_o_combo = state_model.state.outcomes.values()
            #print from_o_combo

            #print [o.outcome_id for o in from_o_combo], state_model.state.state_id
            for transition in trans_dict.values():

                #print transition, [[o.outcome_id == transition.from_outcome, transition.from_state == state_model.state.state_id] for o in from_o_combo]
                from_o_combo = filter(lambda o: not (o.outcome_id == transition.from_outcome and
                                                transition.from_state == state_model.state.state_id), from_o_combo)

                #print [o.outcome_id for o in from_o_combo]
            if len(from_o_combo) > 0:
                free_from_outcomes_dict[state_model.state.state_id] = from_o_combo

        #print "free outcomes: ", free_from_outcomes_dict
        # for state_id in free_from_outcomes_dict.keys():
        #     if not state_id == trans.to_state:
        #         for outcome in free_from_outcomes_dict[state_id]:
        #             state = model.states[state_id].state
        #             if state_id == self_model.state.state_id:
        #                 from_outcome_combo.append(["self." + outcome.name + "." + str(outcome.outcome_id)])
        #             else:
        #                 from_outcome_combo.append([state.name + "." + outcome.name + "." + str(outcome.outcome_id)])
        for outcome in free_from_outcomes_dict[from_state.state_id]:
            state = model.states[from_state.state_id].state
            if from_state.state_id == self_model.state.state_id:
                from_outcome_combo.append(["self." + outcome.name + "." + str(outcome.outcome_id)])
            else:
                from_outcome_combo.append([state.name + "." + outcome.name + "." + str(outcome.outcome_id)])

        # for from-state-combo us all states with free outcomes and from_state
        #print "model.states: ", model.states, free_from_outcomes_dict
        free_from_state_models = filter(lambda smodel: smodel.state.state_id in free_from_outcomes_dict.keys(), model.states.values())
        if not from_state.state_id in free_from_outcomes_dict.keys():
            if from_state.state_id == self_model.state.state_id:
                from_state_combo.append(['self.' + from_state.state_id])
            else:
                from_state_combo.append([from_state.name + '.' + from_state.state_id])
        for state_model in free_from_state_models:
            if state_model.state.state_id == self_model.state.state_id:
                from_state_combo.append(['self.' + state_model.state.state_id])
            else:
                from_state_combo.append([state_model.state.name + '.' + state_model.state.state_id])

        # for to-state-combo  filter self state
        to_state_models = filter(lambda s: not s.state.state_id == trans.from_state, model.states.values())
        for state_model in to_state_models:
            if state_model.state.state_id == self_model.state.state_id:
                to_state_combo.append(["self." + state_model.state.state_id])
            else:
                to_state_combo.append([state_model.state.name + '.' + state_model.state.state_id])

        # for to-outcome-combo use parent combos
        to_outcome = model.state.outcomes.values()
        for outcome in to_outcome:
            if external:
                to_outcome_combo.append(['parent.' + outcome.name + "." + str(outcome.outcome_id)])
            else:
                to_outcome_combo.append(['self.' + outcome.name + "." + str(outcome.outcome_id)])

        return from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_state_models, free_from_outcomes_dict

    def update_transitions_store(self):
        from mvc.models import TransitionModel
        print "update store"

        def update_transition_list_store(model):
            model.transitions = []
            model.transition_list_store.clear()
            for transition in model.state.transitions.values():
                model.transitions.append(TransitionModel(transition, model))
                print "insert: ", transition
                model.transition_list_store.append([transition])

        if hasattr(self.model, 'transitions'):
            print "CLEAN"
            update_transition_list_store(self.model)

        if hasattr(self.model, 'parent'):
            print "CLEAN"
            update_transition_list_store(self.model.parent)

    def update_stores(self):

        self.update_transitions_store()
        model = self.model

        print "clean stores"

        ### FOR COMBOS
        # internal transitions
        # - take all internal states
        # - take all not used internal outcomes of this states

        # external transitions
        # - take all external states
        # - take all external outcomes
        # - take all not used own outcomes

        ### LINKING
        # internal  -> from_state -> outcome combos
        #           -> to
        # external -> state -> outcome combos
        self.combo['internal'] = {}
        self.combo['external'] = {}

        if hasattr(model.state, 'transitions'):
            # check for internal combos
            for transition_id, transition in model.state.transitions.items():
                self.combo['internal'][transition_id] = {}

                [from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_state_models, free_from_outcomes_dict] = \
                    self.get_free_state_combos_for_transition(transition, self.model, self.model)
                print transition

                self.combo['internal'][transition_id]['from_state'] = from_state_combo
                self.combo['internal'][transition_id]['from_outcome'] = from_outcome_combo
                self.combo['internal'][transition_id]['to_state'] = to_state_combo
                self.combo['internal'][transition_id]['to_outcome'] = to_outcome_combo

                self.combo['free_from_state_models'] = free_from_state_models
                self.combo['free_from_outcomes_dict'] = free_from_outcomes_dict
                print "FREE: ", self.combo['free_from_outcomes_dict'].keys(), self.combo['free_from_outcomes_dict']

        if hasattr(model, 'parent'):
            # check for internal combos
            for transition_id, transition in model.parent.state.transitions.items():
                if transition.from_state == model.state.state_id or transition.to_state == model.state.state_id:
                    self.combo['external'][transition_id] = {}

                    [from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_state_models, free_from_outcomes_dict] = \
                        self.get_free_state_combos_for_transition(transition, self.model.parent, self.model, True)
                    print transition

                    self.combo['external'][transition_id]['from_state'] = from_state_combo
                    self.combo['external'][transition_id]['from_outcome'] = from_outcome_combo
                    self.combo['external'][transition_id]['to_state'] = to_state_combo
                    self.combo['external'][transition_id]['to_outcome'] = to_outcome_combo

                    self.combo['free_ext_from_state_models'] = free_from_state_models
                    self.combo['free_ext_from_outcomes_dict'] = free_from_outcomes_dict

        # print "to_state: ", self.list_to_other_state
        # print "to_outcome: ", self.list_to_other_outcome
        # print "from state: ", self.list_from_other_state
        print "state.name: ", self.model.state.name

    def update_model(self):

        self.tree_store.clear()
        if self.view_dict['transitions_internal'] and len(self.model.state.transitions) > 0:
            for transition_id in self.combo['internal'].keys():
                print "TRANSITION_ID: ", transition_id, self.model.state.transitions
                t = self.model.state.transitions[transition_id]
                from_state = self.model.states[t.from_state].state

                if t.to_state is None:
                    to_state_label = None
                    to_outcome_label = 'self.' + self.model.state.outcomes[t.to_outcome].name + "." + str(t.to_outcome)
                else:
                    print t.to_state, self.model.states, "\n", self.model.parent.states
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self." + self.model.state.name + "." + t.to_state
                    else:
                        to_state_label = self.model.states[t.to_state].state.name + "." + t.to_state
                    to_outcome_label = None

                #print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]

                # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
    #                   name-color, to-state-color, transition-object, state-object, is_editable
                self.tree_store.append(None, [transition_id,  # id
                                              from_state.name + "." + from_state.state_id,  # from-state
                                              from_state.outcomes[t.from_outcome].name + "." + str(t.from_outcome),  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              False,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])
        if self.view_dict['transitions_external'] and len(self.model.parent.state.transitions) > 0:
            for transition_id in self.combo['external'].keys():
                print "TRANSITION_ID: ", transition_id, self.model.parent.state.transitions
                t = self.model.parent.state.transitions[transition_id]
                from_state = self.model.parent.states[t.from_state].state

                if from_state.state_id == self.model.state.state_id:
                    from_state_label = "self." + from_state.name + "." + from_state.state_id
                    from_outcome_label = "self." + from_state.outcomes[t.from_outcome].name + "." + str(t.from_outcome)
                else:
                    from_state_label = from_state.name + "." + from_state.state_id
                    from_outcome_label = from_state.outcomes[t.from_outcome].name + "." + str(t.from_outcome)

                if t.to_state is None:
                    to_state_label = 'parent'
                    to_outcome_label = self.model.parent.state.outcomes[t.to_outcome].name + "." + str(t.to_outcome)
                else:
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self." + self.model.state.name + "." + t.to_state
                    else:
                        to_state_label = self.model.parent.states[t.to_state].state.name + "." + t.to_state
                    to_outcome_label = None

                #print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]

                # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
    #                   name-color, to-state-color, transition-object, state-object, is_editable
                self.tree_store.append(None, [transition_id,  # id
                                              from_state_label,  # from-state
                                              from_outcome_label,  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              True,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])

    @Controller.observe("state", after=True)
    def assign_notification_parent_state(self, model, prop_name, info):
        print "transition_listViewCTRL call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        if info.method_name in ["add_outcome", "remove_outcome", "add_transition", "remove_transition",
                                "modify_outcome_name"]:
            self.update_stores()
            self.update_model()