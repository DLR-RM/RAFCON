
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
        self.method_list = ["add_transition", "remove_transition", "add_outcome", "remove_outcome",
                            "modify_outcome_name", "add_state", "remove_state"]

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
        #                   name-color, to-state-color, transition-object, state-model, is_editable
        self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
                                        str, str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT, bool)
        self.combo = {}
        self.combo['internal'] = {}
        self.combo['external'] = {}
        #
        self.to_state_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)
        #
        self.to_outcome_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)

        self.list_to_other_state = {}
        self.list_to_other_outcome = {}
        self.list_from_other_state = {}
        self.list_from_state_to_state = {}
        self.update_stores()
        self.new_version = False
        self.view_dict = {}
        if self.new_version:
            view.get_top_widget().set_model(self.tree_store)

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            if self.new_version:
                pass
            else:
                col = column.get_name()
                states_store = gtk.ListStore(str, str)
                states_store.append([container_model.state.state_id, container_model.state.name])
                transition = model.get_value(iter, 0)
                for state_model in container_model.states.itervalues():
                    #print "state: ", state_model.state
                    #print "state model: ", [state_model.state.state_id, state_model.state.name]
                    states_store.append([state_model.state.state_id, state_model.state.name])

                outcomes_store = gtk.ListStore(str, str)
                if col == 'from_state_col':
                    text = container_model.state.states[transition.from_state].name
                    cell_renderer.set_property('text', text)
                    cell_renderer.set_property('text-column', 1)
                    cell_renderer.set_property('model', states_store)
                elif col == 'to_state_col':
                    text = 'Sel. state' if transition.to_state is None else  \
                        container_model.states[transition.to_state].state.name
                    cell_renderer.set_property('text', text)
                    cell_renderer.set_property('text-column', 1)
                    cell_renderer.set_property('model', states_store)
                elif col == 'from_outcome_col':
                    from_state = container_model.state.states[transition.from_state]
                    self.get_outcome_combos(from_state, outcomes_store)
                    cell_renderer.set_property('text', from_state.outcomes[transition.from_outcome].name)
                    cell_renderer.set_property('text-column', 1)
                    cell_renderer.set_property('model', outcomes_store)
                elif col == 'to_outcome_col':
                    if transition.to_outcome is None:
                        cell_renderer.set_property('text', '')
                    else:
                        print "to state outcomes: ", container_model.state.outcomes
                        self.get_outcome_combos(container_model.state, outcomes_store)
                        cell_renderer.set_property('text', container_model.state.outcomes[transition.to_outcome].name)
                        cell_renderer.set_property('text-column', 1)
                        cell_renderer.set_property('model', outcomes_store)
                elif col == 'external_col':
                    # if transition.to_outcome is None:
                    #     cell_renderer.set_property('boolean', True)
                    # else:
                    #     print "to state outcomes: ", container_model.state.outcomes
                    cell_renderer.set_property('boolean', False)
                else:
                    logger.error("Unknown column '{col:s}' in TransitionListView".format(col=col))

        self.update_model()

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_outcome_col'].set_cell_data_func(view['from_outcome_combo'], cell_text, self.model)
        view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text, self.model)
        #view['external_col'].set_cell_data_func(view['external_toggle'], cell_text, self.model)

        view['from_state_combo'].connect("edited", self.on_combo_changed)
        view['from_outcome_combo'].connect("edited", self.on_combo_changed)
        view['to_state_combo'].connect("edited", self.on_combo_changed)
        view['to_outcome_combo'].connect("edited", self.on_combo_changed)
        #view['external_toggle'].connect("toggled", self.on_external_toggled)


    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    # def toggled_internal_view(self, button, name=None):
    #     if name in ['transitions_external', 'dataflows_external'] and self.model.parent is not None:
    #         self.view_dict[name] = button.get_active()
    #         print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
    #     elif not name in ['transitions_internal', 'dataflows_internal']:
    #         self.view_dict['transitions_external'] = False
    #         self.view_dict['dataflows_external'] = False
    #         button.set_active(False)
    #
    #     if name in ['transitions_internal', 'dataflows_internal'] and hasattr(self.model, 'states'):
    #         self.view_dict[name] = button.get_active()
    #         print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
    #     elif not name in ['transitions_external', 'dataflows_external']:
    #         self.view_dict['transitions_internal'] = False
    #         self.view_dict['dataflows_internal'] = False
    #         button.set_active(False)
    #
    # def toggled_external_view(self, button, name=None):

    def update_stores(self):

        model = self.model

        print "clean stores"
        self.to_state_combo_list.clear()
        self.to_state_combo_list.clear()
        self.to_outcome_combo_list.clear()
        self.to_state_combo_list.clear()

        self.list_to_other_state.clear()
        self.list_to_other_outcome.clear()
        self.list_from_other_state.clear()

        if hasattr(model, 'parent') and model.parent is not None:
            # check for "to state combos" -> so all states in parent
            parent_id = model.parent.state.state_id
            for smdl in model.parent.states.values():
                if not model.state.state_id == smdl.state.state_id:
                    self.to_state_combo_list.append([smdl.state.name + "." + smdl.state.state_id,
                                                     smdl.state.state_id, parent_id])
            # check for "to outcome combos" -> so all outcomes of parent
            for outcome in model.parent.state.outcomes.values():
                print "type outcome: ", outcome.name, type(outcome)
                self.to_outcome_combo_list.append(['parent.' + outcome.name + '.' + str(outcome.outcome_id),
                                                   outcome.outcome_id, parent_id])
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
                self.combo['internal'][transition_id] = {}
                from_state = gtk.ListStore(str)  # transition_id: Liststore
                from_outcome = gtk.ListStore(str)  # transition_id: Liststore
                to_state = gtk.ListStore(str)  # transition_id: Liststore
                to_outcome = gtk.ListStore(str)  # transition_id: Liststore
                print transition.from_state, transition.from_outcome, \
                        transition.to_state, transition.to_outcome, model.state.name, model.state.state_id, \
                    transition_id, transition.transition_id
                if transition.to_state is None:  # no to_state means self
                    if transition.to_outcome in self.list_from_other_state:
                        self.list_from_other_state[transition.to_outcome].append([transition.from_state, transition.from_outcome, transition.transition_id])
                    else:
                        self.list_from_other_state[transition.to_outcome] = [[transition.from_state, transition.from_outcome, transition.transition_id]]
                self.combo['internal'][transition_id]['from_state'] = from_state
                self.combo['internal'][transition_id]['from_outcome'] = from_outcome
                self.combo['internal'][transition_id]['to_state'] = to_state
                self.combo['internal'][transition_id]['to_outcome'] = to_outcome

        print "to_state: ", self.list_to_other_state
        print "to_outcome: ", self.list_to_other_outcome
        print "from state: ", self.list_from_other_state
        print "state.name: ", self.model.state.name

    def update_model(self):

        if hasattr(self.model, 'transitions'):
            print "CLEAN"
            from mvc.models import TransitionModel
            self.model.transitions = []
            self.model.transition_list_store.clear()
            for transition in self.model.state.transitions.itervalues():
                self.model.transitions.append(TransitionModel(transition, self))
                print "insert: ", transition
                self.model.transition_list_store.append([transition])

        if self.new_version:
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
        else:
            if isinstance(self.model.parent, StateModel):
                import gobject
                transition_list_store = gtk.ListStore(gobject.TYPE_PYOBJECT)
                # if isinstance(self.model, ContainerStateModel):
                #     transition_list_store.append(self.model.transition_list_store)
                # transition_list_store.append(self.model.parent.transition_list_store)
                # view.get_top_widget().set_model(transition_list_store)
                if isinstance(self.model, ContainerStateModel):
                    self.view.get_top_widget().set_model(self.model.transition_list_store)
            else:
                if isinstance(self.model, ContainerStateModel):
                    self.view.get_top_widget().set_model(self.model.transition_list_store)

    def on_add(self, button, info=None):
        print "add transition"
        if self.view_dict['transitions_internal']:
            if len(self.model.states) > 0:
                from_state_id = self.model.states.values()[0].state.state_id
                from_outcome = self.model.states.values()[0].state.outcomes.values()[0].outcome_id
                to_state_id = self.model.state.state_id
                to_outcome = self.model.state.outcomes.values()[0].outcome_id
                transition_id = self.model.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
                print "NEW TRANSITION INTERNAL IS: ", self.model.state.transitions[transition_id]
            else:
                print "NO INTERNAL STATE could not add transition"
        elif self.view_dict['transitions_external']:
            from_state_id = self.model.state.state_id
            from_outcome = self.model.state.outcomes.values()[0].outcome_id
            to_state_id = self.model.parent.state.state_id
            to_outcome = self.model.parent.state.outcomes.values()[0].outcome_id
            transition_id = self.model.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            print "NEW TRANSITION EXTERNAL IS: ", self.model.state.transitions[transition_id]
        else:
            print "NO OPTION TO ADD TRANSITION"

    def on_remove(self, button, info=None):
        print "remove transition"
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        print path, tree
        if path:
            if self.new_version:
                print "Transition: ", self.tree_store[path[0][0]][0]
            else:
                print "Transition old: ", self.model.transition_list_store[path[0][0]][0]
                transition_id = self.model.transition_list_store[path[0][0]][0].transition_id
                self.model.state.remove_transition(transition_id)

    def get_state_combos(self, state):
        states_store = gtk.ListStore(str, str)
        states_store.append([container_model.state.state_id, container_model.state.name])
        for state_model in container_model.states.itervalues():
            print "model: ", [state_model.state.state_id, state_model.state.name]
            states_store.append([state_model.state.state_id, state_model.state.name])

    def get_outcome_combos(self, state, outcomes_store):
        print "state name: ", state.name
        print "from state outcomes: ", state.outcomes
        for outcome in state.outcomes.itervalues():
            print "outcome: ", outcome
            print "model: ", [state.state_id, outcome.name]
            outcomes_store.append([state.state_id, outcome.name])
        print "final store: ", outcomes_store
        return outcomes_store

    def on_combo_changed(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        #liststore[path][2]

    @Controller.observe("state", after=True)
    def assign_notification_parent_state(self, model, prop_name, info):
        print "transition_listViewCTRL call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        if info.method_name in ["add_outcome", "remove_outcome", "add_transition", "remove_transition",
                                "modify_outcome_name"]:
            self.update_model()