import gtk
import gobject

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.statemachine_helper import insert_self_transition_meta_data
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateOutcomesListController(ExtendedController):

    """The controller handles the outcomes of one respective state

    The controller allows to add and remove outcomes as well as to add, remove and to modify the related transition.

    The related transition can be set to a sibling-state, to the state it self or to a outcome of the parent.
    Hereby the transition also can switch from pointing to an outcome or to a state. It react to changes in the
    state's respective outcomes-list, transitions-list or change of parent-state and use additionally the focus change
    to update after a modification (e.g. the focus change updates if not observed state-names change).
    """

    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)

        self.tree_store = view.tree_store

        self.to_state_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)
        self.to_outcome_combo_list = gtk.ListStore(gobject.TYPE_STRING, gobject.TYPE_STRING, gobject.TYPE_STRING)
        # key-outcome_id -> label,  to_state_id,  transition_id
        self.dict_to_other_state = {}
        # key-outcome_id ->  label,  to_outcome_id,  transition_id
        self.dict_to_other_outcome = {}
        # not used at the moment key-outcome_id -> label,  from_state_id,  transition_id
        self.dict_from_other_state = {}  # if widget gets extended

        if not model.state.is_root_state:
            self.observe_model(model.parent)

        # in case of weired notification calls
        self.update_runs = False  # secures that on_focus and outcomes_changed do not run at the same time updates

        # initiate data base and tree
        self.update_internal_data_base()
        self.update_tree_store()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """

        def cell_text(column, cell_renderer, model, iter, container_model):

            outcome = model.get_value(iter, 6)
            if column.get_title() == 'ID':
                if int(outcome.outcome_id) < 0:
                    cell_renderer.set_alignment(0.9, 0.5)
                    # this is False per default thus in the else case "editable" will be False as well
                    cell_renderer.set_property('editable', False)
            elif column.get_title() == 'Name':
                if int(outcome.outcome_id) < 0:
                    cell_renderer.set_property('editable', False)
                else:
                    if not isinstance(self.model.state, LibraryState):
                        cell_renderer.set_property('editable', True)
            elif column.get_title() == 'To-State':
                cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_state_combo_list)
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            elif column.get_title() == 'To-Outcome':
                cell_renderer.set_property("editable", True)
                cell_renderer.set_property("model", self.to_outcome_combo_list)
                cell_renderer.set_property("text-column", 0)
                cell_renderer.set_property("has-entry", False)
            else:
                logger.warning("Column does not have cell_data_func %s %s" % (column.get_name(), column.get_title()))

        view['tree_view'].set_model(self.tree_store)
        view['id_col'].set_cell_data_func(view['id_cell'], cell_text, self.model)
        view['name_col'].set_cell_data_func(view['name_cell'], cell_text, self.model)
        if view['to_state_col'] and view['to_outcome_col'] and view['to_state_combo'] and view['to_outcome_combo']:
            view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
            view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text, self.model)
            view['to_state_combo'].connect("edited", self.on_to_state_modification)
            view['to_outcome_combo'].connect("edited", self.on_to_outcome_modification)

        view['name_cell'].connect('edited', self.on_name_modification)
        view.tree_view.connect("grab-focus", self.on_focus)

    def on_focus(self, widget, data=None):
        # logger.debug("OUTCOMES_LIST get new FOCUS")
        path = self.view.tree_view.get_cursor()
        try:
            self.update_internal_data_base()
            self.update_tree_store()
        except Exception:
            logger.warning("update failed")
        if path[0]:  # if valid -> is possible as long as set_cursor does not trigger on_focus again
            self.view.tree_view.set_cursor(path[0])

    def on_name_modification(self, widget, path, text):
        outcome_id = self.tree_store[path][6].outcome_id
        outcome = self.model.state.outcomes[outcome_id]
        try:
            outcome.name = text
            logger.debug("Outcome name changed to '{0}'".format(outcome.name))
        except (ValueError, TypeError) as e:
            logger.warning("The name of the outcome could not be changed: {0}".format(e))
        self.tree_store[path][1] = outcome.name

    def on_to_state_modification(self, widget, path, text):
        # logger.debug("on_to_state_modification %s, %s, %s" % (widget, path, text))
        outcome_id = int(self.tree_store[path][0])
        if outcome_id in self.dict_to_other_state.keys() or outcome_id in self.dict_to_other_outcome.keys():
            transition_parent_state = self.model.parent.state
            if outcome_id in self.dict_to_other_state.keys():
                # logger.debug("1")
                t_id = int(self.dict_to_other_state[outcome_id][2])
            else:
                # logger.debug("2")
                t_id = int(self.dict_to_other_outcome[outcome_id][2])
            if text is not None:
                to_state_id = text.split('.')[1]
                if not transition_parent_state.transitions[t_id].to_state == to_state_id:
                    try:
                        transition_parent_state.transitions[t_id].modify_target(to_state=to_state_id)
                    except ValueError as e:
                        logger.warn("The target of transition couldn't be modified: {0}".format(e))
            else:
                try:
                    transition_parent_state.remove_transition(t_id)
                except AttributeError as e:
                    logger.warn("The transition couldn't be removed: {0}".format(e))
        else:  # there is no transition till now
            if text is not None and not self.model.state.is_root_state:
                # logger.debug("s31")
                transition_parent_state = self.model.parent.state
                to_state_id = text.split('.')[1]
                try:
                    t_id = transition_parent_state.add_transition(from_state_id=self.model.state.state_id,
                                                                  from_outcome=outcome_id,
                                                                  to_state_id=to_state_id,
                                                                  to_outcome=None, transition_id=None)
                except (ValueError, TypeError) as e:
                    logger.warn("The transition couldn't be added: {0}".format(e))
                    return

                # add self transition meta data
                if 'self' in text.split('.'):
                    insert_self_transition_meta_data(self.model, t_id, 'outcomes_widget', 'append_to_last_change')

            else:
                # logger.debug("s32")
                logger.debug("outcome-editor got None in to_state-combo-change no transition is added")

    def on_to_outcome_modification(self, widget, path, text):
        # logger.debug("on_to_outcome_modification %s, %s, %s" % (widget, path, text))
        outcome_id = int(self.tree_store[path][0])
        transition_parent_state = self.model.parent.state
        if outcome_id in self.dict_to_other_state.keys() or outcome_id in self.dict_to_other_outcome.keys():
            if outcome_id in self.dict_to_other_state.keys():
                # logger.debug("o1")
                t_id = int(self.dict_to_other_state[outcome_id][2])
            else:
                # logger.debug("o2")
                t_id = int(self.dict_to_other_outcome[outcome_id][2])
            if text is not None:
                new_to_outcome_id = int(text.split('.')[2])
                if not transition_parent_state.transitions[t_id].to_outcome == new_to_outcome_id:
                    to_state_id = self.model.parent.state.state_id
                    try:
                        transition_parent_state.transitions[t_id].modify_target(to_state=to_state_id,
                                                                                to_outcome=new_to_outcome_id)
                    except ValueError as e:
                        logger.warn("The target of transition couldn't be modified: {0}".format(e))
            else:
                transition_parent_state.remove_transition(t_id)
        else:  # there is no transition till now
            if text is not None:
                # logger.debug("o31")
                to_outcome = int(text.split('.')[2])

                try:
                    self.model.parent.state.add_transition(from_state_id=self.model.state.state_id,
                                                           from_outcome=outcome_id,
                                                           to_state_id=self.model.parent.state.state_id,
                                                           to_outcome=to_outcome, transition_id=None)
                except (ValueError, TypeError) as e:
                    logger.warn("The transition couldn't be added: {0}".format(e))
            else:
                # logger.debug("o32")
                logger.debug("outcome-editor got None in to_outcome-combo-change no transition is added")

    def on_add(self, button, info=None):
        # logger.debug("add outcome")
        outcome_id = None
        for run_id in range(5):
            try:
                outcome_id = self.model.state.add_outcome('success' + str(len(self.model.state.outcomes)-1+run_id))
                break
            except ValueError as e:
                logger.debug("The outcome couldn't be added: {0}".format(e))
                if run_id == 4:
                    logger.warn("The outcome couldn't be added: {0}".format(e))
                    return
        # Search for new entry and select it
        ctr = 0
        for outcome_entry in self.tree_store:
            # Compare outcome ids
            if outcome_entry[6].outcome_id == outcome_id:
                self.view.tree_view.set_cursor(ctr)
                break
            ctr += 1

    def on_remove(self, button, info=None):

        tree, path = self.view.tree_view.get_selection().get_selected_rows()
        if path:  # and not self.tree_store[path[0][0]][6].outcome_id < 0 leave this check for the state
            outcome_id = self.tree_store[path[0][0]][6].outcome_id
            try:
                self.model.state.remove_outcome(outcome_id)
                row_number = path[0][0]
                if len(self.tree_store) > 0:
                    self.view.tree_view.set_cursor(min(row_number, len(self.tree_store)-1))
            except AttributeError as e:
                logger.warning("Error while removing outcome: {0}".format(e))

    def update_internal_data_base(self):

        if self.update_runs:
            return
        self.update_runs = True

        model = self.model

        # print "clean data base"
        self.to_state_combo_list.clear()
        self.to_state_combo_list.append([None, None, None])
        self.to_outcome_combo_list.clear()
        self.to_outcome_combo_list.append([None, None, None])
        self.dict_to_other_state.clear()
        self.dict_to_other_outcome.clear()
        self.dict_from_other_state.clear()

        if not model.state.is_root_state:
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
                # print "type outcome: ", outcome.name, type(outcome)
                self.to_outcome_combo_list.append(['parent.' + outcome.name + '.' + str(outcome.outcome_id),
                                                   outcome.outcome_id, parent_id])
            for transition_id, transition in model.parent.state.transitions.items():
                # print transition.from_state, transition.from_outcome, \
                #         transition.to_state, transition.to_outcome, model.parent.state.name, \
                #         model.parent.state.state_id, \
                #         transition_id, transition.transition_id, model.parent.state.transitions[transition_id].transition_id
                # check for "to other state" connections -> so from self-state and self-outcome "external" transitions
                if transition.from_state == model.state.state_id and transition.from_outcome in model.state.outcomes.keys():
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
        if hasattr(model.state, 'transitions'):
            # check for "from other state" connections -> so to self-state and self-outcome "internal" transitions
            for transition_id, transition in model.state.transitions.items():
                # print transition.from_state, transition.from_outcome, \
                #         transition.to_state, transition.to_outcome, model.state.name, model.state.state_id, \
                #         transition_id, transition.transition_id
                if transition.to_state is None:  # no to_state means self
                    if transition.to_outcome in self.dict_from_other_state:
                        self.dict_from_other_state[transition.to_outcome].append([transition.from_state, transition.from_outcome, transition.transition_id])
                    else:
                        self.dict_from_other_state[transition.to_outcome] = [[transition.from_state, transition.from_outcome, transition.transition_id]]

        # print "to_state: ", self.list_to_other_state
        # print "to_outcome: ", self.list_to_other_outcome
        # print "from state: ", self.list_from_other_state
        # print "state.name: ", self.model.state.name
        self.update_runs = False

    def update_tree_store(self):

        if self.update_runs:
            return
        self.update_runs = True
        self.tree_store.clear()
        for outcome in self.model.state.outcomes.values():
            to_state = None
            if outcome.outcome_id in self.dict_to_other_state.keys():
                to_state = self.dict_to_other_state[outcome.outcome_id][0]
            to_outcome = None
            if outcome.outcome_id in self.dict_to_other_outcome.keys():
                to_outcome = self.dict_to_other_outcome[outcome.outcome_id][0]
                to_state = 'parent'
            from_state = None
            if outcome.outcome_id in self.dict_from_other_state.keys():
                from_state = self.dict_from_other_state[outcome.outcome_id][0]
            # print "treestore: ", [outcome.outcome_id, outcome.name, to_state, to_outcome]
            self.tree_store.append(None, [outcome.outcome_id, outcome.name, to_state, to_outcome,
                                          '#f0E5C7', '#f0E5c7', outcome, self.model.state])
        self.update_runs = False

    @ExtendedController.observe("parent", after=True)
    @ExtendedController.observe("outcomes", after=True)
    @ExtendedController.observe("transitions", after=True)
    def outcomes_changed(self, model, prop_name, info):
        # logger.debug("call_notification - AFTER:prop-%s instance-%s method-%s result-%s" %
        #              (prop_name, info.instance, info.method_name, info.result))
        self.update_internal_data_base()
        self.update_tree_store()


class StateOutcomesEditorController(ExtendedController):

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.oc_list_ctrl = StateOutcomesListController(model, view.treeView)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """

        if view['add_button'] and view['remove_button']:
            view['add_button'].connect("clicked", self.oc_list_ctrl.on_add)
            view['remove_button'].connect("clicked", self.oc_list_ctrl.on_remove)

            if isinstance(self.model.state, LibraryState):
                view['add_button'].set_sensitive(False)
                view['remove_button'].set_sensitive(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_outcome)
        shortcut_manager.add_callback_for_action("add", self.add_outcome)

    def add_outcome(self, *_):
        if self.view.tree.is_focus():
            self.oc_list_ctrl.on_add(None)

    def remove_outcome(self, *_):
        if self.view.tree.is_focus():
            self.oc_list_ctrl.on_remove(None)
