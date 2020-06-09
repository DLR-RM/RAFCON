# Copyright (C) 2015-2018 DLR
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
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_outcomes
   :synopsis: A module that holds the controller to list and edit outcomes of a state and to add-remove or edit
     outcome-related transitions.

"""

from gi.repository import GObject
from gi.repository import Gtk
from builtins import str

from rafcon.gui.helpers.meta_data import insert_self_transition_meta_data
from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.states.library_state import LibraryState
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.controllers.utils.tree_view_controller import ListViewController, react_to_event
from rafcon.gui.views.state_editor.outcomes import StateOutcomesTreeView, StateOutcomesEditorView
from rafcon.gui.models.container_state import ContainerStateModel, AbstractStateModel
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.utils import log

logger = log.get_logger(__name__)

# TODO find out why not editable ID-column cause segfault ->  if sometimes move into widget by tab and enter is pressed


class StateOutcomesListController(ListViewController):
    """The controller handles the outcomes of one respective state

    The controller allows to add and remove outcomes as well as to add, remove and to modify the related transition.

    The related transition can be set to a sibling-state, to the state it self or to a outcome of the parent.
    Hereby the transition also can switch from pointing to an outcome or to a state. It react to changes in the
    state's respective outcomes-list, transitions-list or change of parent-state and use additionally the focus change
    to update after a modification (e.g. the focus change updates if not observed state-names change).
    """
    ID_STORAGE_ID = 0
    NAME_STORAGE_ID = 1
    CORE_STORAGE_ID = 4
    CORE_PARENT_STORAGE_ID = 5
    MODEL_STORAGE_ID = 6
    CORE_ELEMENT_CLASS = Outcome

    def __init__(self, model, view):
        assert isinstance(model, AbstractStateModel)
        # initiate data base and tree
        # id, name, to-state, to-outcome, name-color, to-state-color, outcome, state, outcome_model
        list_store = Gtk.ListStore(int, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_PYOBJECT, GObject.TYPE_PYOBJECT,
                                   GObject.TYPE_PYOBJECT)
        super(StateOutcomesListController, self).__init__(model, view, view['tree_view'], list_store, logger)

        self.to_state_combo_list = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING)
        self.to_outcome_combo_list = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING)
        # key-outcome_id -> label,  to_state_id,  transition_id
        self.dict_to_other_state = {}
        # key-outcome_id ->  label,  to_outcome_id,  transition_id
        self.dict_to_other_outcome = {}
        # not used at the moment key-outcome_id -> label,  from_state_id,  transition_id
        self.dict_from_other_state = {}  # if widget gets extended
        # TODO check why the can happen should not be handed always the LibraryStateModel
        if not (model.state.is_root_state or model.state.is_root_state_of_library):
            self.observe_model(model.parent)

        if self.model.get_state_machine_m() is not None:
            self.observe_model(self.model.get_state_machine_m())
        else:
            logger.warning("State model has no state machine model -> state model: {0}".format(self.model))

    def destroy(self):
        self.relieve_all_models()
        super(StateOutcomesListController, self).destroy()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        super(StateOutcomesListController, self).register_view(view)
        if isinstance(view, StateOutcomesTreeView):
            self.connect_signal(view['to_state_combo'], "edited", self.on_to_state_edited)
            self.connect_signal(view['to_outcome_combo'], "edited", self.on_to_outcome_edited)

        if isinstance(self.model.state, LibraryState) or self.model.state.get_next_upper_library_root_state():
            view['id_cell'].set_property('editable', False)
            view['name_cell'].set_property('editable', False)

        self._apply_value_on_edited_and_focus_out(view['name_cell'], self.apply_new_outcome_name)

        self.update(initiator='"register view"')

    def apply_new_outcome_name(self, path, new_name):
        """Apply the newly entered outcome name it is was changed

        :param str path: The path string of the renderer
        :param str new_name: Newly entered outcome name
        """
        # Don't do anything if outcome name didn't change
        if new_name == self.list_store[path][self.NAME_STORAGE_ID]:
            return

        outcome = self.list_store[path][self.CORE_STORAGE_ID]
        try:
            outcome.name = new_name
            logger.debug("Outcome name changed to '{0}'".format(outcome.name))
        except (ValueError, TypeError) as e:
            logger.warning("The name of the outcome could not be changed: {0}".format(e))
        self.list_store[path][self.NAME_STORAGE_ID] = outcome.name

    def on_to_state_edited(self, renderer, path, new_state_identifier):
        """Connects the outcome with a transition to the newly set state

        :param Gtk.CellRendererText renderer: The cell renderer that was edited
        :param str path: The path string of the renderer
        :param str new_state_identifier: An identifier for the new state that was selected
        """
        def do_self_transition_check(t_id, new_state_identifier):
            # add self transition meta data
            if 'self' in new_state_identifier.split('.'):
                insert_self_transition_meta_data(self.model, t_id, 'outcomes_widget', combined_action=True)

        outcome_id = self.list_store[path][self.ID_STORAGE_ID]
        if outcome_id in self.dict_to_other_state or outcome_id in self.dict_to_other_outcome:
            transition_parent_state = self.model.parent.state
            if outcome_id in self.dict_to_other_state:
                t_id = self.dict_to_other_state[outcome_id][2]
            else:
                t_id = self.dict_to_other_outcome[outcome_id][2]
            if new_state_identifier is not None:
                to_state_id = new_state_identifier.split('.')[1]
                if not transition_parent_state.transitions[t_id].to_state == to_state_id:
                    try:
                        transition_parent_state.transitions[t_id].modify_target(to_state=to_state_id)
                        do_self_transition_check(t_id, new_state_identifier)
                    except ValueError as e:
                        logger.warning("The target of transition couldn't be modified: {0}".format(e))
            else:
                try:
                    transition_parent_state.remove_transition(t_id)
                except AttributeError as e:
                    logger.warning("The transition couldn't be removed: {0}".format(e))
        else:  # there is no transition till now
            if new_state_identifier is not None and not self.model.state.is_root_state:
                transition_parent_state = self.model.parent.state
                to_state_id = new_state_identifier.split('.')[1]
                try:
                    t_id = transition_parent_state.add_transition(from_state_id=self.model.state.state_id,
                                                                  from_outcome=outcome_id,
                                                                  to_state_id=to_state_id,
                                                                  to_outcome=None, transition_id=None)
                    do_self_transition_check(t_id, new_state_identifier)
                except (ValueError, TypeError) as e:
                    logger.warning("The transition couldn't be added: {0}".format(e))
                    return
            else:
                logger.debug("outcome-editor got None in to_state-combo-change no transition is added")

    def on_to_outcome_edited(self, renderer, path, new_outcome_identifier):
        """Connects the outcome with a transition to the newly set outcome

        :param Gtk.CellRendererText renderer: The cell renderer that was edited
        :param str path: The path string of the renderer
        :param str new_outcome_identifier: An identifier for the new outcome that was selected
        """
        if self.model.parent is None:
            return
        outcome_id = self.list_store[path][self.ID_STORAGE_ID]
        transition_parent_state = self.model.parent.state
        if outcome_id in self.dict_to_other_state or outcome_id in self.dict_to_other_outcome:
            if outcome_id in self.dict_to_other_state:
                t_id = self.dict_to_other_state[outcome_id][2]
            else:
                t_id = self.dict_to_other_outcome[outcome_id][2]
            if new_outcome_identifier is not None:
                new_to_outcome_id = int(new_outcome_identifier.split('.')[2])
                if not transition_parent_state.transitions[t_id].to_outcome == new_to_outcome_id:
                    to_state_id = self.model.parent.state.state_id
                    try:
                        transition_parent_state.transitions[t_id].modify_target(to_state=to_state_id,
                                                                                to_outcome=new_to_outcome_id)
                    except ValueError as e:
                        logger.warning("The target of transition couldn't be modified: {0}".format(e))
            else:

                transition_parent_state.remove_transition(t_id)
        else:  # there is no transition till now
            if new_outcome_identifier is not None:
                to_outcome = int(new_outcome_identifier.split('.')[2])

                try:
                    self.model.parent.state.add_transition(from_state_id=self.model.state.state_id,
                                                           from_outcome=outcome_id,
                                                           to_state_id=self.model.parent.state.state_id,
                                                           to_outcome=to_outcome, transition_id=None)
                except (ValueError, TypeError) as e:
                    logger.warning("The transition couldn't be added: {0}".format(e))
            else:
                logger.debug("outcome-editor got None in to_outcome-combo-change no transition is added")

    def on_add(self, button, info=None):
        try:
            outcome_ids = gui_helper_state_machine.add_outcome_to_selected_states(selected_states=[self.model])
            if outcome_ids:
                self.select_entry(outcome_ids[self.model.state])
        except ValueError as e:
            logger.warning("The outcome couldn't be added: {0}".format(e))
            return False
        return True

    def remove_core_element(self, model):
        """Remove respective core element of handed outcome model

        :param OutcomeModel model: Outcome model which core element should be removed
        :return:
        """
        assert model.outcome.parent is self.model.state
        gui_helper_state_machine.delete_core_element_of_model(model)

    def on_right_click_menu(self):
        pass

    def update_internal_data_base(self):

        model = self.model

        self.to_state_combo_list.clear()
        self.to_state_combo_list.append([None, None, None])
        self.to_outcome_combo_list.clear()
        self.to_outcome_combo_list.append([None, None, None])
        self.dict_to_other_state.clear()
        self.dict_to_other_outcome.clear()
        self.dict_from_other_state.clear()

        # TODO check this work around -> it could be avoided by observation of remove-before-notifications
        if not (model.state.is_root_state or model.state.is_root_state_of_library) and self.model.parent.state:  # if parent model is not already destroyed
            # check for "to state combos" -> so all states in parent
            parent_id = model.parent.state.state_id
            for parent_child_state_m in model.parent.states.values():
                if not model.state.state_id == parent_child_state_m.state.state_id:
                    self.to_state_combo_list.append([parent_child_state_m.state.name + "." + parent_child_state_m.state.state_id,
                                                     parent_child_state_m.state.state_id, parent_id])
                else:
                    self.to_state_combo_list.append(["self." + parent_child_state_m.state.state_id,
                                                     parent_child_state_m.state.state_id, parent_id])
            # check for "to outcome combos" -> so all outcomes of parent
            for outcome in model.parent.state.outcomes.values():
                self.to_outcome_combo_list.append(['parent.' + outcome.name + '.' + str(outcome.outcome_id),
                                                   str(outcome.outcome_id), parent_id])
            for transition_id, transition in model.parent.state.transitions.items():
                # check for "to other state" connections -> so from self-state and self-outcome "external" transitions
                if transition.from_state == model.state.state_id and transition.from_outcome in model.state.outcomes:
                    # check for "to other outcomes" connections -> so to parent-state and parent-outcome "ext" transitions
                    if transition.to_state == model.parent.state.state_id:
                        to_state_id = model.parent.state.state_id
                        to_outcome_name = model.parent.state.outcomes[transition.to_outcome].name
                        self.dict_to_other_outcome[transition.from_outcome] = [to_outcome_name + '.' + str(transition.to_outcome),
                                                                               to_state_id,
                                                                               transition.transition_id]
                    else:  # or to other state
                        if model.parent.states[transition.to_state].state.state_id == self.model.state.state_id:
                            to_state_name = 'self'
                        else:
                            to_state_name = model.parent.states[transition.to_state].state.name
                        self.dict_to_other_state[transition.from_outcome] = [to_state_name + '.' + transition.to_state,
                                                                             '',
                                                                             transition.transition_id]
        if isinstance(model, ContainerStateModel):
            # check for "from other state" connections -> so to self-state and self-outcome "internal" transitions
            for transition_id, transition in model.state.transitions.items():
                if transition.to_state is None:  # no to_state means self
                    if transition.to_outcome in self.dict_from_other_state:
                        self.dict_from_other_state[transition.to_outcome].append([transition.from_state, transition.from_outcome, transition.transition_id])
                    else:
                        self.dict_from_other_state[transition.to_outcome] = [[transition.from_state, transition.from_outcome, transition.transition_id]]

    def update_list_store(self):

        self.list_store.clear()
        for outcome in self.model.state.outcomes.values():
            to_state = None
            if outcome.outcome_id in self.dict_to_other_state:
                to_state = self.dict_to_other_state[outcome.outcome_id][0]
            to_outcome = None
            if outcome.outcome_id in self.dict_to_other_outcome:
                to_outcome = self.dict_to_other_outcome[outcome.outcome_id][0]
                to_state = 'parent'
            self.list_store.append([outcome.outcome_id, outcome.name, to_state, to_outcome,
                                    outcome, self.model.state, self.model.get_outcome_m(outcome.outcome_id)])

        if isinstance(self.view, StateOutcomesTreeView):
            for cell_renderer in self.view['to_state_col'].get_cells():
                if self.model.state.get_next_upper_library_root_state() is None:
                    cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_state_combo_list)
                cell_renderer.set_property("text-column", self.ID_STORAGE_ID)
                cell_renderer.set_property("has-entry", False)
        if self.view and isinstance(self.view, StateOutcomesTreeView):
            for cell_renderer in self.view['to_outcome_col'].get_cells():
                if self.model.state.get_next_upper_library_root_state() is None:
                    cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_outcome_combo_list)
                cell_renderer.set_property("text-column", self.ID_STORAGE_ID)
                cell_renderer.set_property("has-entry", False)

    def update(self, initiator='Unknown'):
        try:
            self.update_internal_data_base()
            self.update_list_store()
        except Exception as e:
            # TODO this is connected to the destruction_signal TODO some lines below
            # normally this is the case when some of the sibling states are already destroyed i.e. sibling = None
            # and the code line to access the state_id of a sibling state cannot be done any more
            # actually this should be solved by connecting to the destruction_signal and relieve all models upon
            # receiving it, but the TODO below prevents this
            logger.debug("Error: Unexpected failure while update of outcomes of {0} with path {1} "
                             "with initiator {2}".format(self.model.state, self.model.state.get_path(), initiator))

    @ListViewController.observe("parent", after=True)
    @ListViewController.observe("outcomes", after=True)
    @ListViewController.observe("transitions", after=True)
    def outcomes_changed(self, model, prop_name, info):
        self.update(initiator=str(info))

    # TODO Find out why the observation of the destruction_signal cause threading problems
    # @ExtendedController.observe("destruction_signal", signal=True)
    # def get_destruction_signal(self, model, prop_name, info):
    #     """ Relieve models if the parent state model is destroyed"""
    #     # this is necessary because the controller use data of its parent model and would try to adapt to
    #     # transition changes before the self.model is destroyed, too
    #     if not self.model.state.is_root_state and self.model.parent is model:
    #         # as long as a relieve of models before the after will cause threading issues the controller is suspended
    #         self.__suspended = True


class StateOutcomesEditorController(ExtendedController):

    def __init__(self, model, view):
        """Constructor
        """
        super(StateOutcomesEditorController, self).__init__(model, view)
        self.oc_list_ctrl = StateOutcomesListController(model, view.treeView)
        self.add_controller('oc_list_ctrl', self.oc_list_ctrl)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        super(StateOutcomesEditorController, self).register_view(view)
        if isinstance(view, StateOutcomesEditorView):
            self.connect_signal(view['add_button'], "clicked", self.oc_list_ctrl.on_add)
            self.connect_signal(view['remove_button'], "clicked", self.oc_list_ctrl.on_remove)

            if isinstance(self.model.state, LibraryState) or self.model.state.get_next_upper_library_root_state():
                view['add_button'].set_sensitive(False)
                view['remove_button'].set_sensitive(False)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("copy", self.oc_list_ctrl.copy_action_callback)
        shortcut_manager.add_callback_for_action("delete", self.oc_list_ctrl.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.oc_list_ctrl.add_action_callback)
        shortcut_manager.add_callback_for_action("cut", self.oc_list_ctrl.cut_action_callback)
        shortcut_manager.add_callback_for_action("paste", self.paste_action_callback)

    def paste_action_callback(self, *event, **kwargs):
        """Callback method for paste action"""
        if react_to_event(self.view, self.oc_list_ctrl.tree_view, event) and self.oc_list_ctrl.active_entry_widget is None:
            global_clipboard.paste(self.model, limited=['outcomes'])
            return True
