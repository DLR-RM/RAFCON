
from gtkmvc import Controller, Observer
import gtk
import gobject

from utils import log
logger = log.get_logger(__name__)


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
        # logger.debug("parent call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
        #              (prop_name, info.instance, info.method_name, info.result))
        if info.method_name in self.method_list:
            for func_handle in self.func_handle_list:
                func_handle()


class StateTransitionsListController(Controller):
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

        # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
        #                   name-color, to-state-color, transition-object, state-object, is_editable
        self.view_dict = {'transitions_internal': True, 'transitions_external': True}
        self.tree_store = gtk.TreeStore(int, str, str, str, str, bool,
                                        str, str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT, bool)
        self.combo = {}

        if self.model.parent is not None:
            # OLD
            self.parent_observer = ParentObserver(model.parent, "state", [self.update_internal_data_base,
                                                                          self.update_tree_store])
            # NEW
            self.observe_model(self.model.parent)
            self.model.parent.register_observer(self)

        self.update_internal_data_base()
        self.update_tree_store()
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
            # print t_id, in_external, self.combo[in_external]
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
                # print "to_outcome: ", type(cell_renderer)
                # find to outcome by from_outcome == model.state.state_id and from_outcome == outcome.name
            else:
                logger.warning("Column has no cell_data_func %s %s" % (column.get_name(), column.get_title()))

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
        view.tree_view.connect("grab-focus", self.on_focus)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_focus(self, widget, data=None):
        logger.debug("TRANSITIONS_LIST get new FOCUS")
        path = self.view.tree_view.get_cursor()
        self.update_internal_data_base()
        self.update_tree_store()
        self.view.tree_view.set_cursor(path[0])

    # TODO mach es trotzdem es ist fuer faelschliche interne aber alls externe gewollte ganz nuetzlich
    def on_external_toggled(self, widget, path):
        """ Changes the transition from internal transition to a external ??? But does not make sense to do so or??
        :param widget:
        :param path:
        :return:
        """
        logger.debug("Widget: {widget:s} - Path: {path:s}".format(widget=widget, path=path))
    #
    #     # if widget.get_active():
    #     #     # print "change to INTERNAL ", path
    #     #     self.tree_store[path][5] = False
    #     # else:
    #     #     # print "change to EXTERNAL ", path
    #     #     self.tree_store[path][5] = True

    def on_add(self, button, info=None):
        # print "add transition"
        # type(self.combo['free_from_state_models']), self.combo['free_from_state_models'], '\n', self.combo['free_from_outcomes_dict']
        if self.view_dict['transitions_internal'] and self.combo['free_from_outcomes_dict']:
            from_state_id = self.combo['free_from_outcomes_dict'].keys()[0]
            from_outcome = self.combo['free_from_outcomes_dict'][from_state_id][0].outcome_id
            to_state_id = None  # self.model.state.state_id
            to_outcome = self.model.state.outcomes.values()[0].outcome_id
            # print "NEW TRANSITION INTERNAL IS: ", from_state_id, from_outcome, to_state_id, to_outcome
            transition_id = self.model.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            # print "NEW TRANSITION INTERNAL IS: ", self.model.state.transitions[transition_id]

        elif self.view_dict['transitions_external'] and self.combo['free_ext_from_outcomes_dict'] and \
                        self.model.state.state_id in self.combo['free_ext_from_outcomes_dict']:
            from_state_id = self.model.state.state_id
            from_outcome = self.combo['free_ext_from_outcomes_dict'][from_state_id][0].outcome_id
            to_state_id = None  # self.model.parent.state.state_id
            to_outcome = self.model.parent.state.outcomes.values()[0].outcome_id
            # print "NEW TRANSITION EXTERNAL IS: ", from_state_id, from_outcome, to_state_id, to_outcome
            transition_id = self.model.parent.state.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            # print "NEW TRANSITION EXTERNAL IS: ", self.model.parent.state.transitions[transition_id]
        else:
            logger.warning("NO OPTION TO ADD TRANSITION")

        # set focus on this new element
        # - at the moment every new element is the last -> easy work around :(
        self.view.tree_view.set_cursor(len(self.tree_store)-1)

    def on_remove(self, button, info=None):
        # print "remove transition"
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        # print path, tree
        if path:
            # print "Transition: ", self.tree_store[path[0][0]][0]
            transition_id = self.tree_store[path[0][0]][0]
            if self.tree_store[path[0][0]][5]:
                # print self.model.parent.state.transitions
                self.model.parent.state.remove_transition(int(transition_id))
            else:
                self.model.state.remove_transition(int(transition_id))

            # selection to next element
            row_number = path[0][0]
            if len(self.tree_store) > row_number:
                self.view.tree_view.set_cursor(path[0][0])
            elif len(self.tree_store) == row_number and not len(self.tree_store) == 0:
                self.view.tree_view.set_cursor(path[0][0]-1)

    def on_combo_changed_from_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

        # check if the state may has not changed or combo is empty
        if self.tree_store[path][1] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        t_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # is external
            self.model.parent.state.transitions[t_id].from_state = text[-1]
        else:
            self.model.state.transitions[t_id].from_state = text[-1]

    def on_combo_changed_from_outcome(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

        # check if the outcome may has not changed or combo is empty
        if self.tree_store[path][2] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        t_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # external
            self.model.parent.state.transitions[t_id].from_outcome = int(text[-1])
        else:
            self.model.state.transitions[t_id].from_outcome = int(text[-1])

    def on_combo_changed_to_state(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        # check if the state may has not changed or combo is empty
        if self.tree_store[path][3] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        t_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # is external
            self.model.parent.state.transitions[t_id].to_state = text[-1]
        else:
            self.model.state.transitions[t_id].to_state = text[-1]

    def on_combo_changed_to_outcome(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))

        # check if the outcome may has not changed or combo is empty
        if self.tree_store[path][4] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        t_id = self.tree_store[path][0]
        if self.tree_store[path][5]:  # is external
            self.model.parent.state.transitions[t_id].to_outcome = int(text[-1])
        else:
            self.model.state.transitions[t_id].to_outcome = int(text[-1])

    @staticmethod
    def get_possible_combos_for_transition(trans, model, self_model, is_external=False):
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
            # print from_o_combo

            # print [o.outcome_id for o in from_o_combo], state_model.state.state_id
            for transition in trans_dict.values():
                # print transition, [[o.outcome_id == transition.from_outcome, transition.from_state == state_model.state.state_id] for o in from_o_combo]
                from_o_combo = filter(lambda o: not (o.outcome_id == transition.from_outcome and
                                                transition.from_state == state_model.state.state_id), from_o_combo)
                #print [o.outcome_id for o in from_o_combo]

            if len(from_o_combo) > 0:
                free_from_outcomes_dict[state_model.state.state_id] = from_o_combo

        if from_state.state_id in free_from_outcomes_dict:
            for outcome in free_from_outcomes_dict[from_state.state_id]:
                state = model.states[from_state.state_id].state
                if from_state.state_id == self_model.state.state_id:
                    from_outcome_combo.append(["self." + outcome.name + "." + str(outcome.outcome_id)])
                else:
                    from_outcome_combo.append([state.name + "." + outcome.name + "." + str(outcome.outcome_id)])

        # for from-state-combo us all states with free outcomes and from_state
        # print "model.states: ", model.states, free_from_outcomes_dict
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

        # for to-state-combo filter from_state ... put it at the end
        to_state_models = filter(lambda s: not s.state.state_id == trans.from_state, model.states.values())
        for state_model in to_state_models:
            if state_model.state.state_id == self_model.state.state_id:
                to_state_combo.append(["self." + state_model.state.state_id])
            else:
                to_state_combo.append([state_model.state.name + '.' + state_model.state.state_id])
        from_state = model.states[trans.from_state].state
        to_state_combo.append([from_state.name + '.' + from_state.state_id])

        # for to-outcome-combo use parent combos
        to_outcome = model.state.outcomes.values()
        for outcome in to_outcome:
            if is_external:
                to_outcome_combo.append(['parent.' + outcome.name + "." + str(outcome.outcome_id)])
            else:
                to_outcome_combo.append(['self.' + outcome.name + "." + str(outcome.outcome_id)])

        return from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_state_models, free_from_outcomes_dict

    def update_internal_data_base(self):

        model = self.model

        # print "clean data base"

        ### FOR COMBOS
        # internal transitions
        # - take all internal states
        # - take all not used internal outcomes of this states

        # external transitions
        # - take all external states
        # - take all external outcomes
        # - take all not used own outcomes

        ### LINKING
        # internal  -> transition_id -> from_state = outcome combos
        #           -> ...
        # external -> state -> outcome combos
        self.combo['internal'] = {}
        self.combo['external'] = {}

        if hasattr(model.state, 'transitions'):
            # check for internal combos
            for transition_id, transition in model.state.transitions.items():
                self.combo['internal'][transition_id] = {}

                [from_state_combo, from_outcome_combo,
                 to_state_combo, to_outcome_combo,
                 free_from_state_models, free_from_outcomes_dict] = \
                    self.get_possible_combos_for_transition(transition, self.model, self.model)
                # print transition

                self.combo['internal'][transition_id]['from_state'] = from_state_combo
                self.combo['internal'][transition_id]['from_outcome'] = from_outcome_combo
                self.combo['internal'][transition_id]['to_state'] = to_state_combo
                self.combo['internal'][transition_id]['to_outcome'] = to_outcome_combo

                self.combo['free_from_state_models'] = free_from_state_models
                self.combo['free_from_outcomes_dict'] = free_from_outcomes_dict
                # print "FREE: ", self.combo['free_from_outcomes_dict'].keys(), self.combo['free_from_outcomes_dict']

        if hasattr(model, 'parent') and self.model.parent is not None:
            # check for external combos
            for transition_id, transition in model.parent.state.transitions.items():
                if transition.from_state == model.state.state_id or transition.to_state == model.state.state_id:
                    self.combo['external'][transition_id] = {}

                    [from_state_combo, from_outcome_combo,
                     to_state_combo, to_outcome_combo,
                     free_from_state_models, free_from_outcomes_dict] = \
                        self.get_possible_combos_for_transition(transition, self.model.parent, self.model, True)
                    # print transition

                    self.combo['external'][transition_id]['from_state'] = from_state_combo
                    self.combo['external'][transition_id]['from_outcome'] = from_outcome_combo
                    self.combo['external'][transition_id]['to_state'] = to_state_combo
                    self.combo['external'][transition_id]['to_outcome'] = to_outcome_combo

                    self.combo['free_ext_from_state_models'] = free_from_state_models
                    self.combo['free_ext_from_outcomes_dict'] = free_from_outcomes_dict

        # print "state.name: ", self.model.state.name

    def update_tree_store(self):

        self.tree_store.clear()
        if self.view_dict['transitions_internal'] and hasattr(self.model.state, 'transitions') and \
                        len(self.model.state.transitions) > 0:
            for transition_id in self.combo['internal'].keys():
                # print "TRANSITION_ID: ", transition_id, self.model.state.transitions
                t = self.model.state.transitions[transition_id]
                from_state = self.model.states[t.from_state].state

                if t.to_state is None:
                    to_state_label = None
                    to_outcome_label = 'self.' + self.model.state.outcomes[t.to_outcome].name + "." + str(t.to_outcome)
                else:
                    # print t.to_state, self.model.states
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self." + self.model.state.name + "." + t.to_state
                    else:
                        to_state_label = self.model.states[t.to_state].state.name + "." + t.to_state
                    to_outcome_label = None

                # print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]

                # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
    #                   name-color, to-state-color, transition-object, state-object, is_editable
                self.tree_store.append(None, [transition_id,  # id
                                              from_state.name + "." + from_state.state_id,  # from-state
                                              from_state.outcomes[t.from_outcome].name + "." + str(t.from_outcome),  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              False,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])

        if self.view_dict['transitions_external'] and self.model.parent and \
                        len(self.model.parent.state.transitions) > 0:
            for transition_id in self.combo['external'].keys():
                # print "TRANSITION_ID: ", transition_id, self.model.parent.state.transitions
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

                # print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]

                # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
    #                   name-color, to-state-color, transition-object, state-object, is_editable
                self.tree_store.append(None, [transition_id,  # id
                                              from_state_label,  # from-state
                                              from_outcome_label,  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              True,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])

    # NEW
    @Controller.observe("states", after=True)
    # @Controller.observe("outcomes", after=True)  # do not exist at the moment
    @Controller.observe("transitions", after=True)
    def transition_changed_parent_and_self_state(self, model, prop_name, info):
        # print "transition_listViewCTRL call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #       (prop_name, info.instance, info.method_name, info.result)
        self.update_internal_data_base()
        self.update_tree_store()

    # OLD
    # @Controller.observe("state", after=True)
    # def assign_notification_parent_and_self_state(self, model, prop_name, info):
    #     # print "transition_listViewCTRL call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
    #     #       (prop_name, info.instance, info.method_name, info.result)
    #     if info.method_name in ["add_outcome", "remove_outcome",
    #                             "modify_outcome_name"]:
    #         self.update_stores()
    #         self.update_model()


class StateTransitionsEditorController(Controller):

    def __init__(self, model, view):
        Controller.__init__(self, model, view)
        self.trans_list_ctrl = StateTransitionsListController(model, view.transitions_listView)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['add_t_button'].connect('clicked', self.trans_list_ctrl.on_add)
        #view['cancel_t_edit_button'].connect('clicked', self.on_cancel_transition_edit_clicked)
        view['remove_t_button'].connect('clicked', self.trans_list_ctrl.on_remove)
        view['connected_to_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_external')
        view['internal_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_internal')

        if self.model.parent is None:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            view['connected_to_t_checkbutton'].set_active(False)

        if not hasattr(self.model, 'states'):
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            view['internal_t_checkbutton'].set_active(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def toggled_button(self, button, name=None):

        if name in ['transitions_external'] and self.model.parent is not None:
            self.trans_list_ctrl.view_dict[name] = button.get_active()
            # print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
        elif not name in ['transitions_internal']:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            button.set_active(False)

        if name in ['transitions_internal'] and hasattr(self.model, 'states'):
            self.trans_list_ctrl.view_dict[name] = button.get_active()
            # print(name, "was turned", self.view_dict[name])  # , "\n", self.view_dict
        elif not name in ['transitions_external']:
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            button.set_active(False)

        self.trans_list_ctrl.update_internal_data_base()
        self.trans_list_ctrl.update_tree_store()
