# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_transitions
   :synopsis: A module that holds the controller to list and edit all internal and related external transitions of a
     state.

"""

from gi.repository import GObject
from gi.repository import Gtk
from builtins import str

from rafcon.core.state_elements.transition import Transition
from rafcon.core.states.library_state import LibraryState
from rafcon.gui.controllers.state_editor.linkage_list import LinkageListController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.container_state import ContainerStateModel
from rafcon.gui.utils.notification_overview import NotificationOverview
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.utils import log
from functools import reduce

logger = log.get_logger(__name__)


class StateTransitionsListController(LinkageListController):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc3.Controller` class is the interface between the GTK widget view
    :class:`gui.views.transitions.TransitionListView` and the transitions of the
    :class:`gui.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param rafcon.gui.models.ContainerStateModel model: The container state model containing the data
    :param rafcon.gui.views.TransitionListView view: The GTK view showing the transitions as a table
    """
    ID_STORAGE_ID = 0
    FROM_STATE_STORAGE_ID = 1
    FROM_OUTCOME_STORAGE_ID = 2
    TO_STATE_STORAGE_ID = 3
    TO_OUTCOME_STORAGE_ID = 4
    IS_EXTERNAL_STORAGE_ID = 5
    MODEL_STORAGE_ID = 9
    CORE_ELEMENT_CLASS = Transition

    # TODO if a library with show content flag True is selected also the internal linkage should be shown and fit

    def __init__(self, model, view):
        # ListStore for: id, from-state, from-outcome, to-state, to-outcome, is_external,
        #                   name-color, to-state-color, transition-object, state-object, is_editable, transition-model
        list_store = Gtk.ListStore(int, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, bool,
                                   GObject.TYPE_PYOBJECT, GObject.TYPE_PYOBJECT, bool, GObject.TYPE_PYOBJECT)

        self.view_dict = {'transitions_internal': True, 'transitions_external': True}
        self.combo = {}
        self.debug_log = False
        super(StateTransitionsListController, self).__init__(model, view, view.get_top_widget(), list_store, logger)

    def destroy(self):
        self.view['from_state_col'].set_cell_data_func(self.view['from_state_combo'], None)
        self.view['to_state_col'].set_cell_data_func(self.view['to_state_combo'], None)
        self.view['from_outcome_col'].set_cell_data_func(self.view['from_outcome_combo'], None)
        self.view['to_outcome_col'].set_cell_data_func(self.view['to_outcome_combo'], None)
        super(StateTransitionsListController, self).destroy()

    def register_view(self, view):
        """Called when the View was registered
        """
        super(StateTransitionsListController, self).register_view(view)

        def cell_text(column, cell_renderer, model, iter, data):
            t_id = model.get_value(iter, self.ID_STORAGE_ID)
            in_external = 'external' if model.get_value(iter, self.IS_EXTERNAL_STORAGE_ID) else 'internal'
            # print(t_id, in_external, self.combo[in_external])
            if column.get_title() == 'Source State':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['from_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Source Outcome':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['from_outcome'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Target State':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['to_state'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'Target Outcome':
                cell_renderer.set_property("model", self.combo[in_external][t_id]['to_outcome'])
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            else:
                logger.warning("Column has no cell_data_func %s %s" % (column.get_name(), column.get_title()))

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text)
        view['from_outcome_col'].set_cell_data_func(view['from_outcome_combo'], cell_text)
        view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text)

        if self.model.state.get_next_upper_library_root_state():
            view['from_state_combo'].set_property("editable", False)
            view['from_outcome_combo'].set_property("editable", False)
            view['to_state_combo'].set_property("editable", False)
            view['to_outcome_combo'].set_property("editable", False)
        else:
            self.connect_signal(view['from_state_combo'], "edited", self.on_combo_changed_from_state)
            self.connect_signal(view['from_outcome_combo'], "edited", self.on_combo_changed_from_outcome)
            self.connect_signal(view['to_state_combo'], "edited", self.on_combo_changed_to_state)
            self.connect_signal(view['to_outcome_combo'], "edited", self.on_combo_changed_to_outcome)

        view.tree_view.connect("grab-focus", self.on_focus)
        self.update(initiator='"register view"')

    def on_focus(self, widget, data=None):
        path = self.get_path()
        self.update(initiator='"focus"')
        if path:
            self.tree_view.set_cursor(path)

    def on_add(self, button, info=None):

        if gui_helper_state_machine.is_selection_inside_of_library_state(selected_elements=[self.model]):
            logger.error("New transition is not added because target state is inside of library state.")
            return
        free_outcomes = None

        if self.view_dict['transitions_internal'] and self.combo['free_from_outcomes_dict']:
            from_state_id = list(self.combo['free_from_outcomes_dict'].keys())[0]
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
        to_outcomes = list(responsible_parent.outcomes.values())
        if len(to_outcomes) == 0:
            logger.warning("No more options to add a transition")
            return
        to_outcome = to_outcomes[0].outcome_id
        # print("NEW TRANSITION IS: ", from_state_id, from_outcome, to_state_id, to_outcome)

        try:
            if from_state_id == responsible_parent.state_id:
                from_state_id = None
            t_id = responsible_parent.add_transition(from_state_id, from_outcome, to_state_id, to_outcome)
            # Set the selection to the new transition
            self.select_entry(t_id)
            return True
        except (AttributeError, ValueError) as e:
            logger.error("Transition couldn't be added: {0}".format(e))

    def remove_core_element(self, model):
        """Remove respective core element of handed transition model

        :param TransitionModel model: Transition model which core element should be removed
        :return:
        """
        assert model.transition.parent is self.model.state or model.transition.parent is self.model.parent.state
        gui_helper_state_machine.delete_core_element_of_model(model)

    def on_combo_changed_from_state(self, widget, path, text):
        # Check whether the from state was changed or the combo entry is empty
        t_id = self.list_store[path][self.ID_STORAGE_ID]
        if text is None:
            return

        # corresponding transition id

        is_external = self.list_store[path][self.IS_EXTERNAL_STORAGE_ID]
        from_state_combo = self.combo['external' if is_external else 'internal'][t_id]['from_state']
        # get selected combo entry by comparing the state names or by state_id
        if text.split('.')[0] in ['self', 'parent']:
            new_from_state_id = self.model.state.state_id if text.split('.')[0] == 'self' else self.model.state.parent.state_id
        else:
            if is_external and text.split('.')[-1] in self.model.state.parent.states or \
                    isinstance(self.model, ContainerStateModel) and text.split('.')[-1] in self.model.state.states:
                new_from_state_id = text.split('.')[-1]
            else:
                from_state_entry = reduce(lambda s1, s2: s1 if s1[0] == text else s2, from_state_combo)
                new_from_state_id = from_state_entry[1]

        if self.list_store[path][self.IS_EXTERNAL_STORAGE_ID] and self.model.parent.state.transitions[t_id].from_state == text.split('.')[-1] or \
                isinstance(self.model, ContainerStateModel) and \
                (self.model.state.transitions[t_id].from_state == new_from_state_id or
                self.model.state.transitions[t_id].from_state is None and self.model.state.state_id == new_from_state_id):
            return

        if is_external:  # is external
            responsible_state_m = self.model.parent
            free_outcomes = self.combo['free_ext_from_outcomes_dict'][new_from_state_id]
        else:
            responsible_state_m = self.model
            free_outcomes = self.combo['free_from_outcomes_dict'][new_from_state_id]

        current_from_outcome = responsible_state_m.state.transitions[t_id].from_outcome
        free_outcome_ids = [None if outcome_m is None else outcome_m.outcome_id for outcome_m in free_outcomes]
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
            responsible_state_m.state.transitions[t_id].modify_origin(new_from_state_id, new_from_outcome_id)
        except ValueError as e:
            logger.error("Could not change transition from state: {0}".format(e))

    def on_combo_changed_from_outcome(self, widget, path, text):
        # check if the outcome may has not changed or combo is empty
        if text is None or text == "None" or self.list_store[path][self.FROM_OUTCOME_STORAGE_ID] == text.split('.')[0]:
            return

        # transition gets modified
        text = text.split('.')
        new_from_outcome_id = int(text[-1])
        transition_id = self.list_store[path][self.ID_STORAGE_ID]

        if self.list_store[path][self.IS_EXTERNAL_STORAGE_ID]:  # is external transition
            transition_parent_state = self.model.parent.state
        else:
            transition_parent_state = self.model.state

        try:
            transition_parent_state.transitions[transition_id].from_outcome = new_from_outcome_id
        except ValueError as e:
            logger.error("Could not change the origin outcome of the transition: {0}".format(e))

    def on_combo_changed_to_state(self, widget, path, text):
        # check if the state may has not changed or combo is empty
        if text is None:
            return

        # transition gets modified
        text = text.split('.')
        new_to_state_id = text[-1]
        transition_id = self.list_store[path][self.ID_STORAGE_ID]

        if self.list_store[path][self.IS_EXTERNAL_STORAGE_ID]:  # is external transition
            transition_parent_state = self.model.parent.state
        else:
            transition_parent_state = self.model.state

        if transition_parent_state.transitions[transition_id].to_state is None and \
                transition_parent_state.state_id == new_to_state_id or \
                transition_parent_state.transitions[transition_id].to_state == new_to_state_id:
            return

        if not new_to_state_id == transition_parent_state.state_id:
            new_to_outcome = None
        else:
            new_to_outcome = transition_parent_state.outcomes[0].outcome_id

        try:
            transition_parent_state.transitions[transition_id].modify_target(to_state=new_to_state_id,
                                                                             to_outcome=new_to_outcome)
        except ValueError as e:
            logger.error("Could not change the target state of the transition: {0}".format(e))

    def on_combo_changed_to_outcome(self, widget, path, text):
        # check if the outcome may has not changed or combo is empty
        if text is None or self.list_store[path][self.TO_OUTCOME_STORAGE_ID] == text.split('.')[1]:
            return

        # transition gets modified
        text = text.split('.')
        new_to_outcome_id = int(text[-1])
        transition_id = self.list_store[path][self.ID_STORAGE_ID]

        if self.list_store[path][self.IS_EXTERNAL_STORAGE_ID]:  # is external transition
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

    def on_right_click_menu(self):
        logger.debug("do right click menu")

    @staticmethod
    def get_possible_combos_for_transition(trans, model, self_model, is_external=False):
        """ The function provides combos for a transition and its respective

        :param trans:
        :param model:
        :param self_model:
        :param is_external:
        :return:
        """
        from_state_combo = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING)
        from_outcome_combo = Gtk.ListStore(GObject.TYPE_STRING)
        to_state_combo = Gtk.ListStore(GObject.TYPE_STRING)
        to_outcome_combo = Gtk.ListStore(GObject.TYPE_STRING)

        trans_dict = model.state.transitions

        # get from state
        if trans is None:
            from_state = None
        elif trans.from_state is not None:
            from_state = model.state.states[trans.from_state]
        else:
            from_state = model.state if is_external else self_model.state

        # collect all free from-outcome-combo and from_state which are still valid -> filter all outcome already in use
        free_from_outcomes_dict = {}
        for state in model.state.states.values():
            from_o_combo = state.outcomes.values()
            # print([o.outcome_id for o in from_o_combo], state_model.state.state_id)
            for transition in trans_dict.values():
                # print(transition, [[o.outcome_id == transition.from_outcome, transition.from_state == state_model.state.state_id] for o in from_o_combo])
                from_o_combo = [o for o in from_o_combo if not (o.outcome_id == transition.from_outcome and
                                                     transition.from_state == state.state_id)]
                # print([o.outcome_id for o in from_o_combo])
            if len(from_o_combo) > 0:
                free_from_outcomes_dict[state.state_id] = from_o_combo
        # check if parent has start_state
        if model.state.start_state_id is None:
            free_from_outcomes_dict[model.state.state_id] = [None]

        # for from-state-combo use all states with free outcomes and from_state
        combined_states = [model.state] if is_external else [self_model.state]
        combined_states.extend(model.state.states.values())
        free_from_states = [state for state in combined_states if state.state_id in free_from_outcomes_dict]

        if trans is None:
            return None, None, None, None, free_from_states, free_from_outcomes_dict

        def append_from_state_combo(possible_state):
            if possible_state.state_id == self_model.state.state_id:
                from_state_combo.append(['self.' + possible_state.state_id, possible_state.state_id])
            elif is_external and from_state.state_id == model.state.state_id:
                from_state_combo.append(['parent.' + possible_state.state_id, possible_state.state_id])
            else:
                from_state_combo.append([possible_state.name + '.' + possible_state.state_id, possible_state.state_id])

        append_from_state_combo(from_state)
        for state in free_from_states:
            if from_state is not state:
                append_from_state_combo(state)

        # for from-outcome-combo collect all combos for actual transition
        # -> actual outcome + free outcomes of actual from_state.state_id
        if trans is not None:
            if trans.from_outcome is None:
                from_outcome_combo.append(["None"])
            else:
                outcome = from_state.outcomes[trans.from_outcome]
                from_outcome_combo.append([outcome.name + "." + str(outcome.outcome_id)])
            for outcome in free_from_outcomes_dict.get(from_state.state_id, []):
                if outcome is None:
                    from_outcome_combo.append(["None"])
                else:
                    from_outcome_combo.append([outcome.name + "." + str(outcome.outcome_id)])

        # get to state
        if trans.to_state == model.state.state_id:
            to_state = model.state if is_external else self_model.state
        else:
            to_state = model.state.states[trans.to_state]

        # for to-state-combo filter from_state -> first actual to_state + other optional states
        def generate_to_state_combo(possible_state):
            if possible_state.state_id == self_model.state.state_id:
                to_state_combo.append(["self." + possible_state.state_id])
            elif is_external and possible_state.state_id == model.state.state_id:
                to_state_combo.append(['parent.' + possible_state.state_id])
            else:
                to_state_combo.append([possible_state.name + '.' + possible_state.state_id])

        to_states = [model.state] if is_external else [self_model.state]
        to_states.extend(model.state.states.values())
        generate_to_state_combo(to_state)
        for state in to_states:
            if not to_state.state_id == state.state_id:
                generate_to_state_combo(state)

        # for to-outcome-combo use parent combos -> first actual outcome + other outcome
        def append_to_outcome_combo(possible_outcome):
            if is_external:
                to_outcome_combo.append(['parent.' + possible_outcome.name + "." + str(possible_outcome.outcome_id)])
            else:
                to_outcome_combo.append(['self.' + possible_outcome.name + "." + str(possible_outcome.outcome_id)])

        if trans.to_outcome is not None:
            append_to_outcome_combo(model.state.outcomes[trans.to_outcome])
        for outcome in model.state.outcomes.values():
            if not (trans.to_outcome == outcome.outcome_id and trans.to_state == model.state.state_id):
                append_to_outcome_combo(outcome)

        return from_state_combo, from_outcome_combo, to_state_combo, to_outcome_combo, free_from_states, free_from_outcomes_dict

    def _update_internal_data_base(self):
        """ Updates Internal combo knowledge for any actual transition by calling  get_possible_combos_for_transition-
        function for those.
        """

        model = self.model

        # print("clean data base")

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

        if isinstance(model, ContainerStateModel):
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
        # TODO check why the can happen should not be handed always the LibraryStateModel
        if not (self.model.state.is_root_state or self.model.state.is_root_state_of_library):
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
        """ Updates TreeStore of the Gtk.ListView according internal combo knowledge gained by
        _update_internal_data_base function call.
        """

        self.list_store.clear()
        if self.view_dict['transitions_internal'] and isinstance(self.model, ContainerStateModel) and \
                len(self.model.state.transitions) > 0:
            for transition_id in self.combo['internal'].keys():
                # print("TRANSITION_ID: ", transition_id, self.model.state.transitions)
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
                    # print(t.to_state, self.model.states)
                    if t.to_state == self.model.state.state_id:
                        to_state_label = "self (" + self.model.state.name + ")"
                        to_outcome_label = self.model.state.outcomes[t.to_outcome].name
                    else:
                        to_state_label = self.model.state.states[t.to_state].name
                        to_outcome_label = None

                self.list_store.append([transition_id,  # id
                                        from_state_label,  # from-state
                                        from_outcome_label,  # from-outcome
                                        to_state_label,  # to-state
                                        to_outcome_label,  # to-outcome
                                        False,  # is_external
                                        t,
                                        self.model.state,
                                        True,
                                        self.model.get_transition_m(transition_id)])

        if self.view_dict['transitions_external'] and self.model.parent and \
                len(self.model.parent.state.transitions) > 0:
            for transition_id in self.combo['external'].keys():
                # print("TRANSITION_ID: ", transition_id, self.model.parent.state.transitions)
                try:
                    t = self.model.parent.state.transitions[transition_id]
                    # logger.info(str(t))
                    from_state = None
                    if t.from_state is not None:
                        from_state = self.model.parent.states[t.from_state].state

                    if from_state is None:
                        from_state_label = "parent (" + self.model.state.parent.name + ")"
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

                    self.list_store.append([transition_id,  # id
                                            from_state_label,  # from-state
                                            from_outcome_label,  # from-outcome
                                            to_state_label,  # to-state
                                            to_outcome_label,  # to-outcome
                                            True,  # is_external
                                            t,
                                            self.model.state,
                                            True,
                                            self.model.parent.get_transition_m(transition_id)])
                except Exception as e:
                    logger.warning("There was a problem while updating the data-flow widget TreeStore. {0}".format(e))

    def update(self, initiator='Unknown'):
        try:
            self._update_internal_data_base()
            self._update_tree_store()
            self.update_selection_sm_prior()
        except Exception as e:
            logger.exception("Unexpected failure while update of transitions related to {0} with path {1} "
                             "with initiator {2}".format(self.model.state, self.model.state.get_path(), initiator))

    @LinkageListController.observe("state", before=True)
    def before_notification_of_parent_or_state(self, model, prop_name, info):
        """ Set the no update flag to avoid updates in between of a state removal. """
        # logger.info("before state notification: {0}".format(NotificationOverview(info)))
        self.check_no_update_flags_and_return_combined_flag(prop_name, info)

    @LinkageListController.observe("state", after=True)
    def after_notification_state(self, model, prop_name, info):

        # avoid updates because of execution status updates or while multi-actions
        if self.check_no_update_flags_and_return_combined_flag(prop_name, info):
            return

        overview = NotificationOverview(info)

        if overview.get_cause() == 'parent' and overview.get_affected_core_element() is self.model.state or \
                overview.get_affected_core_element() in [self.model.state, self.model.state.parent] and \
                overview.get_cause() in ['name', 'group_states', 'ungroup_state', 'change_data_type',
                                                "remove_outcome", "remove_transition"]:
            self.update(initiator=str(overview))

    @LinkageListController.observe("states", after=True)
    @LinkageListController.observe("transitions", after=True)
    @LinkageListController.observe("outcomes", after=True)
    def after_notification_of_parent_or_state_from_lists(self, model, prop_name, info):
        """ Activates the update after update if outcomes, transitions or states list has been changed.
        """
        # avoid updates because of execution status updates or while multi-actions
        if self.check_no_update_flags_and_return_combined_flag(prop_name, info):
            return

        overview = NotificationOverview(info)
        if overview.get_cause() not in ['name', 'append', '__setitem__',  # '__delitem__', 'remove'
                                        'from_outcome', 'to_outcome', 'from_state', 'to_state',
                                        'modify_origin', 'modify_target']:
                if self.model.parent:
                    # check for sibling port change
                    if prop_name == 'states' and overview.get_affected_core_element() is self.model.parent.state and \
                            (overview.get_affected_core_element() in self.model.parent.state.states and
                             overview.get_cause() in ['add_outcome'] or
                             overview.get_affected_property() in ['outcome'] and
                             overview.get_cause() in ['name']):
                        pass
                    else:
                        return
                else:
                    return

        try:
            self.update(initiator=str(overview))
        except KeyError as e:
            if self.debug_log:
                import traceback
                self.store_debug_log_file(str(overview))
                self.store_debug_log_file(str(traceback.format_exc()))
            logger.error("update of transition widget fails while detecting list change of state %s %s %s\n%s" %
                         (self.model.state.name, self.model.state.state_id, e, self))


class StateTransitionsEditorController(ExtendedController):
    def __init__(self, model, view):
        super(StateTransitionsEditorController, self).__init__(model, view)
        self.trans_list_ctrl = StateTransitionsListController(model, view.transitions_listView)
        self.add_controller('trans_list_ctrl', self.trans_list_ctrl)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        super(StateTransitionsEditorController, self).register_view(view)
        view['add_t_button'].connect('clicked', self.trans_list_ctrl.on_add)
        view['remove_t_button'].connect('clicked', self.trans_list_ctrl.on_remove)
        view['connected_to_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_external')
        view['internal_t_checkbutton'].connect('toggled', self.toggled_button, 'transitions_internal')

        if isinstance(self.model.state, LibraryState):
            view['internal_t_checkbutton'].set_sensitive(False)
            view['internal_t_checkbutton'].set_active(False)

        if self.model.parent is not None and isinstance(self.model.parent.state, LibraryState) or \
                self.model.state.get_next_upper_library_root_state():
            view['add_t_button'].set_sensitive(False)
            view['remove_t_button'].set_sensitive(False)

        # TODO check why the can happen should not be handed always the LibraryStateModel
        if self.model.state.is_root_state or self.model.state.is_root_state_of_library:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            view['connected_to_t_checkbutton'].set_active(False)

        if not isinstance(self.model, ContainerStateModel):
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            view['internal_t_checkbutton'].set_active(False)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.trans_list_ctrl.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.trans_list_ctrl.add_action_callback)

    def toggled_button(self, button, name=None):
        # TODO check why the can happen should not be handed always the LibraryStateModel
        if name in ['transitions_external'] and \
                not (self.model.state.is_root_state or self.model.state.is_root_state_of_library):
            self.trans_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['transitions_internal']:
            self.trans_list_ctrl.view_dict['transitions_external'] = False
            button.set_active(False)

        if name in ['transitions_internal'] and isinstance(self.model, ContainerStateModel):
            self.trans_list_ctrl.view_dict[name] = button.get_active()
        elif name not in ['transitions_external']:
            self.trans_list_ctrl.view_dict['transitions_internal'] = False
            button.set_active(False)

        self.trans_list_ctrl.update(initiator='"button toggled"')
