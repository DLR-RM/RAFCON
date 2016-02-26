import gtk
import gobject

from rafcon.mvc.models.container_state import ContainerStateModel
from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.utils.notification_overview import NotificationOverview

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.utils import log

logger = log.get_logger(__name__)


class StateTransitionsListController(ExtendedController):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.transitions.TransitionListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param rafcon.mvc.models.ContainerStateModel model: The container state model containing the data
    :param rafcon.mvc.views.TransitionListView view: The GTK view showing the transitions as a table
    """

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        # TreeStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
        #                   name-color, to-state-color, transition-object, state-object, is_editable
        self.view_dict = {'transitions_internal': True, 'transitions_external': True}
        self.tree_store = gtk.TreeStore(int, str, str, str, str, bool,
                                        str, str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT, bool)
        self.combo = {}
        self.no_update = False  # used to reduce the update cost of the widget (e.g while no focus or complex changes)

        if not self.model.state.is_root_state:
            self.observe_model(self.model.parent)

        self.update()
        view.get_top_widget().set_model(self.tree_store)

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            t_id = model.get_value(iter, 0)
            # state = model.get_value(iter, 9)
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

        # view['external_toggle'].set_radio(True)
        view.tree_view.connect("grab-focus", self.on_focus)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_focus(self, widget, data=None):
        path = self.view.tree_view.get_cursor()
        self.update()
        if path[0]:
            self.view.tree_view.set_cursor(path[0])

    def on_add(self, button, info=None):

        free_outcomes = None

        if self.view_dict['transitions_internal'] and self.combo['free_from_outcomes_dict']:
            from_state_id = self.combo['free_from_outcomes_dict'].keys()[0]
            free_outcomes = self.combo['free_from_outcomes_dict'][from_state_id]
            responsible_parent = self.model.state
        elif self.view_dict['transitions_external'] and self.combo['free_ext_from_outcomes_dict'] and \
                self.model.state.state_id in self.combo['free_ext_from_outcomes_dict']:
            from_state_id = self.model.state.state_id
            free_outcomes = self.combo['free_ext_from_outcomes_dict'][from_state_id]
            responsible_parent = self.model.parent.state

        if free_outcomes is None or len(free_outcomes) == 0:
            logger.warning("No more options to add a transition")
            return

        from_outcome = None if free_outcomes[0] is None else free_outcomes[0].outcome_id
        to_state_id = responsible_parent.state_id
        to_outcomes = responsible_parent.outcomes.values()
        if len(to_outcomes) == 0:
            logger.warning("No more options to add a transition")
            return
        to_outcome = to_outcomes[0].outcome_id
        # print "NEW TRANSITION IS: ", from_state_id, from_outcome, to_state_id, to_outcome

        try:
            if from_state_id == responsible_parent.state_id:
                from_state_id = None
            t_id = responsible_parent.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            # Set the selection to the new transition
            ctr = 0
            for transition_entry in self.tree_store:
                # Compare transition ids
                if transition_entry[0] == t_id:
                    self.view.tree_view.set_cursor(ctr)
                    break
                ctr += 1
        except (AttributeError, ValueError) as e:
            logger.error("Transition couldn't be added: {0}".format(e))

    def on_remove(self, button, info=None):
        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        if path:
            row_number = path[0][0]
            transition_id = self.tree_store[row_number][0]
            try:
                if self.tree_store[row_number][5]:
                    self.model.parent.state.remove_transition(int(transition_id))
                else:
                    self.model.state.remove_transition(int(transition_id))
            except (AttributeError, ValueError) as e:
                logger.error("Transition couldn't be removed: {0}".format(e))

            # selection to next element
            if len(self.tree_store) > 0:
                self.view.tree_view.set_cursor(min(row_number, len(self.tree_store) - 1))
        else:
            logger.warning("Please select the data flow to be deleted")
            return

    def on_combo_changed_from_state(self, widget, path, text):
        # Check whether the from state was changed or the combo entry is empty
        if self.tree_store[path][1] == text or text is None:
            return

        # corresponding transition id
        transition_id = self.tree_store[path][0]
        is_external = self.tree_store[path][5]
        from_state_combo = self.combo['external' if is_external else 'internal'][transition_id]['from_state']
        # get selected combo entry by comparing the state names
        from_state_entry = reduce(lambda s1, s2: s1 if s1[0] == text else s2, from_state_combo)
        new_from_state_id = from_state_entry[1]

        if is_external:  # is external
            responsible_state_m = self.model.parent
            free_outcomes = self.combo['free_ext_from_outcomes_dict'][new_from_state_id]
        else:
            responsible_state_m = self.model
            free_outcomes = self.combo['free_from_outcomes_dict'][new_from_state_id]

        current_from_outcome = responsible_state_m.state.transitions[transition_id].from_outcome
        free_outcome_ids = map(lambda outcome_m: None if outcome_m is None else outcome_m.outcome_id, free_outcomes)
        if len(free_outcomes) == 0:
            logger.warning('There is no free outcome for the chosen state')
            return
        if current_from_outcome in free_outcome_ids:
            new_from_outcome_id = current_from_outcome
        else:
            new_from_outcome_id = None if free_outcomes[0] is None else free_outcomes[0].outcome_id
        if responsible_state_m.state.state_id == new_from_state_id:
            new_from_state_id = None
        try:
            responsible_state_m.state.transitions[transition_id].modify_origin(new_from_state_id, new_from_outcome_id)
        except ValueError as e:
            logger.error("Could not change transition from state: {0}".format(e))

    def on_combo_changed_from_outcome(self, widget, path, text):
        # check if the outcome may has not changed or combo is empty
        if self.tree_store[path][2] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        new_from_outcome_id = int(text[-1])
        transition_id = self.tree_store[path][0]

        if self.tree_store[path][5]:  # is external transition
            transition_parent_state = self.model.parent.state
        else:
            transition_parent_state = self.model.state

        try:
            transition_parent_state.transitions[transition_id].from_outcome = new_from_outcome_id
        except ValueError as e:
            logger.error("Could not change the origin outcome of the transition: {0}".format(e))

    def on_combo_changed_to_state(self, widget, path, text):
        # check if the state may has not changed or combo is empty
        if self.tree_store[path][3] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        new_to_state_id = text[-1]
        transition_id = self.tree_store[path][0]

        if self.tree_store[path][5]:  # is external transition
            transition_parent_state = self.model.parent.state
        else:
            transition_parent_state = self.model.state

        if self.model.parent is None or not new_to_state_id == self.model.parent.state.state_id:
            new_to_outcome = None
        else:
            new_to_outcome = self.model.parent.state.outcomes[0].outcome_id

        try:
            transition_parent_state.transitions[transition_id].modify_target(to_state=new_to_state_id,
                                                                             to_outcome=new_to_outcome)
        except ValueError as e:
            logger.error("Could not change the target state of the transition: {0}".format(e))

    def on_combo_changed_to_outcome(self, widget, path, text):
        # check if the outcome may has not changed or combo is empty
        if self.tree_store[path][4] == text or text is None:
            return

        # transition gets modified
        text = text.split('.')
        new_to_outcome_id = int(text[-1])
        transition_id = self.tree_store[path][0]

        if self.tree_store[path][5]:  # is external transition
            transition_parent_state = self.model.parent.state
            new_to_state_id = self.model.parent.state.state_id
        else:
            transition_parent_state = self.model.state
            new_to_state_id = self.model.state.state_id

        try:
            transition_parent_state.transitions[transition_id].modify_target(to_state=new_to_state_id,
                                                                             to_outcome=new_to_outcome_id)
        except ValueError as e:
            logger.error("Could not change the target outcome of the transition: {0}".format(e))

    @staticmethod
    def get_possible_combos_for_transition(trans, model, self_model, is_external=False):
        from_state_combo = gtk.ListStore(str, str)
        from_outcome_combo = gtk.ListStore(str)
        to_state_combo = gtk.ListStore(str)
        to_outcome_combo = gtk.ListStore(str)

        trans_dict = model.state.transitions

        # for from-outcome-combo filter all outcome already used
        from_state = None
        if trans is not None and trans.from_state is not None:
            from_state = model.state.states[trans.from_state]
        free_from_outcomes_dict = {}
        for state in model.state.states.values():
            from_o_combo = state.outcomes.values()
            # print from_o_combo

            # print [o.outcome_id for o in from_o_combo], state_model.state.state_id
            for transition in trans_dict.values():
                # print transition, [[o.outcome_id == transition.from_outcome, transition.from_state == state_model.state.state_id] for o in from_o_combo]
                from_o_combo = filter(lambda o: not (o.outcome_id == transition.from_outcome and
                                                     transition.from_state == state.state_id), from_o_combo)
                # print [o.outcome_id for o in from_o_combo]

            if len(from_o_combo) > 0:
                free_from_outcomes_dict[state.state_id] = from_o_combo
        # if transition.from_state == self_model.state.state_id:
        #     free_from_outcomes_dict[state_model.state.state_id].append(None)
        if model.state.start_state_id is None:
            free_from_outcomes_dict[model.state.state_id] = [None]

        if from_state is not None and from_state.state_id in free_from_outcomes_dict:
            for outcome in free_from_outcomes_dict[from_state.state_id]:
                state = model.state.states[from_state.state_id]
                if outcome is None:
                    from_outcome_combo.append(["None"])
                elif from_state.state_id == self_model.state.state_id:
                    from_outcome_combo.append(["self." + outcome.name + "." + str(outcome.outcome_id)])
                else:
                    from_outcome_combo.append([state.name + "." + outcome.name + "." + str(outcome.outcome_id)])

        # for from-state-combo us all states with free outcomes and from_state
        # print "model.states: ", model.states, free_from_outcomes_dict
        free_from_states = filter(lambda state: state.state_id in free_from_outcomes_dict.keys(),
                                        model.state.states.values())
        if trans is None:
            return None, None, None, None, free_from_states, free_from_outcomes_dict
        if from_state is not None and from_state.state_id not in free_from_outcomes_dict.keys():
            if from_state.state_id == self_model.state.state_id:
                from_state_combo.append(['self (' + from_state.name + ')', from_state.state_id])
            else:
                from_state_combo.append([from_state.name, from_state.state_id])
        for state in free_from_states:
            if state.state_id == self_model.state.state_id:
                from_state_combo.append(['self (' + state.name + ')', state.state_id])
            else:
                from_state_combo.append([state.name, state.state_id])
        if not is_external and model.state.start_state_id is None:
            from_state_combo.append(['self (' + model.state.name + ')', model.state.state_id])

        # for to-state-combo filter from_state ... put it at the end
        to_states = filter(lambda s: not s.state_id == trans.from_state, model.state.states.values())
        for state in to_states:
            if state.state_id == self_model.state.state_id:
                to_state_combo.append(["self." + state.state_id])
            else:
                to_state_combo.append([state.name + '.' + state.state_id])
        if from_state is not None:
            to_state_combo.append([from_state.name + '.' + from_state.state_id])

        # for to-outcome-combo use parent combos
        to_outcome = model.state.outcomes.values()
        for outcome in to_outcome:
            if is_external:
                to_outcome_combo.append(['parent.' + outcome.name + "." + str(outcome.outcome_id)])
            else:
                to_outcome_combo.append(['self.' + outcome.name + "." + str(outcome.outcome_id)])

        return from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_states, free_from_outcomes_dict

    def update(self):
        self._update_internal_data_base()
        self._update_tree_store()

    def _update_internal_data_base(self):
        """ Updates Internal combo knowledge for any actual transition by calling  get_possible_combos_for_transition-
        function for those.
        """

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
        self.combo['free_from_states'] = {}
        self.combo['free_from_outcomes_dict'] = {}
        self.combo['free_ext_from_outcomes_dict'] = {}
        self.combo['free_ext_from_outcomes_dict'] = {}

        if hasattr(model.state, 'transitions'):
            # check for internal combos
            for transition_id, transition in model.state.transitions.items():
                self.combo['internal'][transition_id] = {}

                [from_state_combo, from_outcome_combo,
                 to_state_combo, to_outcome_combo,
                 free_from_states, free_from_outcomes_dict] = \
                    self.get_possible_combos_for_transition(transition, self.model, self.model)

                self.combo['internal'][transition_id]['from_state'] = from_state_combo
                self.combo['internal'][transition_id]['from_outcome'] = from_outcome_combo
                self.combo['internal'][transition_id]['to_state'] = to_state_combo
                self.combo['internal'][transition_id]['to_outcome'] = to_outcome_combo

                self.combo['free_from_states'] = free_from_states
                self.combo['free_from_outcomes_dict'] = free_from_outcomes_dict

            if not model.state.transitions:
                [x, y, z, v, free_from_states, free_from_outcomes_dict] = \
                    self.get_possible_combos_for_transition(None, self.model, self.model)
                self.combo['free_from_states'] = free_from_states
                self.combo['free_from_outcomes_dict'] = free_from_outcomes_dict

        if not self.model.state.is_root_state:
            # check for external combos
            for transition_id, transition in model.parent.state.transitions.items():
                if transition.from_state == model.state.state_id or transition.to_state == model.state.state_id:
                    self.combo['external'][transition_id] = {}

                    [from_state_combo, from_outcome_combo,
                     to_state_combo, to_outcome_combo,
                     free_from_states, free_from_outcomes_dict] = \
                        self.get_possible_combos_for_transition(transition, self.model.parent, self.model, True)

                    self.combo['external'][transition_id]['from_state'] = from_state_combo
                    self.combo['external'][transition_id]['from_outcome'] = from_outcome_combo
                    self.combo['external'][transition_id]['to_state'] = to_state_combo
                    self.combo['external'][transition_id]['to_outcome'] = to_outcome_combo

                    self.combo['free_ext_from_states'] = free_from_states
                    self.combo['free_ext_from_outcomes_dict'] = free_from_outcomes_dict

            if not model.parent.state.transitions:
                [x, y, z, v, free_from_states, free_from_outcomes_dict] = \
                    self.get_possible_combos_for_transition(None, self.model.parent, self.model, True)
                self.combo['free_ext_from_states'] = free_from_states
                self.combo['free_ext_from_outcomes_dict'] = free_from_outcomes_dict

    def _update_tree_store(self):
        """ Updates TreeStore of the gtk.ListView according internal combo knowledge gained by
        _update_internal_data_base function call.
        """

        self.tree_store.clear()
        if self.view_dict['transitions_internal'] and hasattr(self.model.state, 'transitions') and \
                        len(self.model.state.transitions) > 0:
            for transition_id in self.combo['internal'].keys():
                # print "TRANSITION_ID: ", transition_id, self.model.state.transitions
                t = self.model.state.transitions[transition_id]

                if t.from_state is not None:
                    from_state = self.model.state.states[t.from_state]
                    from_state_label = from_state.name
                    from_outcome_label = from_state.outcomes[t.from_outcome].name
                else:
                    from_state_label = "self (" + self.model.state.name + ")"
                    from_outcome_label = ""

                if t.to_state is None:
                    to_state_label = "self (" + self.model.state.name + ")"
                    to_outcome = None if t.to_outcome is None else self.model.state.outcomes[t.to_outcome]
                    to_outcome_label = "None" if to_outcome is None else to_outcome.name
                else:
                    # print t.to_state, self.model.states
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self (" + self.model.state.name + ")"
                    else:
                        to_state_label = self.model.state.states[t.to_state].name
                    to_outcome_label = None

                self.tree_store.append(None, [transition_id,  # id
                                              from_state_label,  # from-state
                                              from_outcome_label,  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              False,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])

        if self.view_dict['transitions_external'] and self.model.parent and \
                len(self.model.parent.state.transitions) > 0:
            for transition_id in self.combo['external'].keys():
                # print "TRANSITION_ID: ", transition_id, self.model.parent.state.transitions
                t = self.model.parent.state.transitions[transition_id]
                from_state = None
                if t.from_state is not None:
                    from_state = self.model.parent.states[t.from_state].state

                if from_state is None:
                    from_state_label = "self (" + self.model.state.name + ")"
                    from_outcome_label = ""
                elif from_state.state_id == self.model.state.state_id:
                    from_state_label = "self (" + from_state.name + ")"
                    from_outcome_label = from_state.outcomes[t.from_outcome].name
                else:
                    from_state_label = from_state.name
                    from_outcome_label = from_state.outcomes[t.from_outcome].name

                if t.to_state == self.model.parent.state.state_id:
                    to_state_label = 'parent (' + self.model.parent.state.name + ")"
                    to_outcome_label = self.model.parent.state.outcomes[t.to_outcome].name
                else:
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self (" + self.model.state.name + ")"
                    else:
                        to_state_label = self.model.parent.state.states[t.to_state].name
                    to_outcome_label = None

                self.tree_store.append(None, [transition_id,  # id
                                              from_state_label,  # from-state
                                              from_outcome_label,  # from-outcome
                                              to_state_label,  # to-state
                                              to_outcome_label,  # to-outcome
                                              True,  # is_external
                                              '#f0E5C7', '#f0E5c7', t, self.model.state, True])

    @ExtendedController.observe("change_root_state_type", before=True)
    @ExtendedController.observe("change_state_type", before=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        """ Set the no update flag to avoid updates in between of a state-type-change.
        """
        self.no_update = True

    @ExtendedController.observe("state", after=True)
    def after_notification_state(self, model, prop_name, info):
        # The method causing the change raised an exception, thus nothing was changed
        overview = NotificationOverview(info)
        # if isinstance(overview['result'][-1], str) and "CRASH" in overview['result'][-1] or \
        #         isinstance(overview['result'][-1], Exception):
        #     return
        if overview['method_name'][-1] == 'parent' and overview['instance'][-1] is self.model.state:
            self.update()

    @ExtendedController.observe("states", after=True)
    @ExtendedController.observe("transitions", after=True)
    @ExtendedController.observe("outcomes", after=True)
    @ExtendedController.observe("change_root_state_type", after=True)
    @ExtendedController.observe("change_state_type", after=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        """ Activates the update after a state-type-change happend and triggers update if outcomes, transitions or
        states list has been changed.
        """
        # The method causing the change raised an exception, thus nothing was changed
        # overview = NotificationOverview(info)
        # if isinstance(overview['result'][-1], str) and "CRASH" in overview['result'][-1] or \
        #         isinstance(overview['result'][-1], Exception):
        #     return

        # self.notification_logs(model, prop_name, info)
        if self.no_update and info.method_name in ["change_state_type", "change_root_state_type"]:
            # print "DO_UNLOCK TRANSITION WIDGET"
            self.no_update = False

        if self.no_update:
            return

        try:
            self.update()
        except KeyError as e:
            logger.error("update of transition widget fails while detecting list change of state %s %s" %
                         (self.model.state.name, self.model.state.state_id))

    def notification_logs(self, model, prop_name, info):
        # logger.debug("IP OP SV or DF %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
        # (self.model.state.state_id, prop_name, info.instance, info.method_name, info))

        if model.state.state_id == self.model.state.state_id:
            relative_str = "SELF"
            from_state = self.model.state.state_id
        elif self.model.parent and model.state.state_id == self.model.parent.state.state_id:
            relative_str = "PARENT"
            from_state = self.model.parent.state.state_id
        else:
            relative_str = "OTHER"
            from_state = model.state.state_id

        if prop_name == 'states':
            logger.debug(
                "%s gets notified by states from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'transitions':
            logger.debug(
                "%s gets notified by transitions from %s %s" % (self.model.state.state_id, relative_str, from_state))
        elif prop_name == 'outcomes':
            logger.debug(
                "%s gets notified by outcomes from %s %s" % (self.model.state.state_id, relative_str, from_state))
        else:
            logger.debug("IP OP SV or DF !!! FAILURE !!! %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
                         (self.model.state.state_id, prop_name, info.instance, info.method_name, info.result))


class StateTransitionsEditorController(ExtendedController):
    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)
        self.trans_list_ctrl = StateTransitionsListController(model, view.transitions_listView)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['add_t_button'].connect('clicked', self.trans_list_ctrl.on_add)
        view['remove_t_button'].connect('clicked', self.trans_list_ctrl.on_remove)
        view['connected_to_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_external')
        view['internal_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_internal')

        if isinstance(self.model.state, LibraryState):
            view['internal_t_checkbutton'].set_sensitive(False)
            view['internal_t_checkbutton'].set_active(False)

        if self.model.parent is not None and isinstance(self.model.parent.state, LibraryState):
            view['add_t_button'].set_sensitive(False)
            view['remove_t_button'].set_sensitive(False)

        if self.model.state.is_root_state:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            view['connected_to_t_checkbutton'].set_active(False)

        if not isinstance(self.model, ContainerStateModel):
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            view['internal_t_checkbutton'].set_active(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        # self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_transition)
        shortcut_manager.add_callback_for_action("add", self.add_transition)

    def add_transition(self, *_):
        if self.view.transitions_listView.tree_view.is_focus():
            self.trans_list_ctrl.on_add(None)

    def remove_transition(self, *_):
        if self.view.transitions_listView.tree_view.is_focus():
            self.trans_list_ctrl.on_remove(None)

    def toggled_button(self, button, name=None):

        if name in ['transitions_external'] and not self.model.state.is_root_state:
            self.trans_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['transitions_internal']:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            button.set_active(False)

        if name in ['transitions_internal'] and hasattr(self.model, 'states'):
            self.trans_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['transitions_external']:
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            button.set_active(False)

        self.trans_list_ctrl.update()
