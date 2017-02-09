"""
.. module:: state_machine_tree
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to access a state machine overview by a TreeView.

.. moduleauthor:: Rico Belder


"""

from functools import partial

import gobject
import gtk
from rafcon.gui.helpers import state_machine
from rafcon.core.states.state import State
from rafcon.core.states.state import StateType
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.right_click_menu.state import StateMachineTreeRightClickMenuController
from rafcon.gui.controllers.utils.tree_view_controller import TreeViewController
from rafcon.gui.gui_helper import react_to_event
from rafcon.gui.models import ContainerStateModel
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.utils.notification_overview import NotificationOverview, \
    is_execution_status_update_notification_from_state_machine_model
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineTreeController(TreeViewController):
    """Controller handling the state machine tree.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel model: The state machine manager model,
        holding data regarding state machines. Should be exchangeable.
    :param rafcon.gui.views.state_machine_tree.StateMachineTreeView view: The GTK view showing the state machine tree.
    """

    # TODO hold expansion if refresh all is performed -> minor feature (use storage_path instate of state_machine_id)
    # TODO hold expansion type changes are re- and undone -> minor feature which also depends on modification-history
    NAME_STORAGE_ID = 0
    ID_STORAGE_ID = 1
    TYPE_NAME_STORAGE_ID = 2
    MODEL_STORAGE_ID = 3
    STATE_PATH_STORAGE_ID = 4
    CORE_ELEMENT_CLASS = State

    def __init__(self, model, view):
        assert isinstance(model, StateMachineManagerModel)
        tree_store = gtk.TreeStore(str, str, str, gobject.TYPE_PYOBJECT, str)
        super(StateMachineTreeController, self).__init__(model, view, view, tree_store)

        self.state_right_click_ctrl = StateMachineTreeRightClickMenuController(model, view)

        self.view_is_registered = False

        # view.set_hover_expand(True)
        self.state_row_iter_dict_by_state_path = {}
        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.__expansion_state = {}

        self.register()

    def register_view(self, view):
        """Called when the view was registered"""
        self.view.connect('button_press_event', self.mouse_click)
        self.view_is_registered = True
        self.update(with_expand=True)
        super(StateMachineTreeController, self).register_view(view)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self._delete_selection)
        shortcut_manager.add_callback_for_action("add", partial(self._add_new_state, state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add2", partial(self._add_new_state, state_type=StateType.HIERARCHY))
        shortcut_manager.add_callback_for_action("copy", self.copy_action_callback)
        shortcut_manager.add_callback_for_action("cut", self.cut_action_callback)
        shortcut_manager.add_callback_for_action("paste", self.paste_action_callback)

    def register(self):
        """Change the state machine that is observed for new selected states to the selected state machine."""
        # relieve old model
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self._selected_sm_model)
        # set own selected state machine id
        self.__my_selected_sm_id = self.model.selected_state_machine_id
        if self.__my_selected_sm_id is not None:
            # observe new model
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            self.observe_model(self._selected_sm_model)  # for selection
            self.update()
        else:
            self._selected_sm_model = None
            self.tree_store.clear()

    def paste_action_callback(self, *event):
        """Callback method for paste action"""
        if react_to_event(self.view, self.tree_view, event):
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are cut by widget
            if len(sm_selection.states) == 1:
                global_clipboard.paste(sm_selection.get_selected_state(), limited=['states', 'transitions', 'data_flows'])
            else:
                logger.warning("Please select only one state to paste.")
            return True

    def _add_new_state(self, *event, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the the state machine tree is in focus.
        """
        if react_to_event(self.view, self.view['state_machine_tree_view'], event):
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            state_machine.add_new_state(self._selected_sm_model, state_type)
            return True

    def _delete_selection(self, *event):
        if react_to_event(self.view, self.view['state_machine_tree_view'], event):
            return state_machine.delete_selected_elements(self._selected_sm_model)

    @TreeViewController.observe("state_machine", after=True)
    def states_update(self, model, prop_name, info):

        if is_execution_status_update_notification_from_state_machine_model(prop_name, info):
            return

        overview = NotificationOverview(info, False, self.__class__.__name__)

        if overview['prop_name'][-1] == 'state' and \
                overview['method_name'][-1] in ["name"]:  # , "add_state", "remove_state"]:
            self.update_tree_store_row(overview['model'][-1])
        elif overview['prop_name'][-1] == 'state' and \
                overview['method_name'][-1] in ["add_state", "remove_state"]:
            self.update(overview['model'][-1])

    @TreeViewController.observe("state_machine", before=True)
    def states_update_before(self, model, prop_name, info):

        if is_execution_status_update_notification_from_state_machine_model(prop_name, info):
            return

        overview = NotificationOverview(info, False, self.__class__.__name__)

        if overview['prop_name'][-1] == 'state' and \
                overview['method_name'][-1] in ["change_state_type"]:
            changed_model = self._selected_sm_model.get_state_model_by_path(overview['args'][-1][1].get_path())
            self.observe_model(changed_model)

    @TreeViewController.observe("state_type_changed_signal", signal=True)
    def notification_state_type_changed(self, model, prop_name, info):
        self.relieve_model(model)
        self.update() if model.state.is_root_state else self.update(model.parent)

    @TreeViewController.observe("root_state", assign=True)
    def state_machine_notification(self, model, property, info):
        self.update(model.root_state)

    @TreeViewController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        # store expansion state
        self.store_expansion_state()
        # register new state machine
        self.register()
        self.assign_notification_selection(None, None, None)
        # redo expansion state
        self.redo_expansion_state(info)

    def store_expansion_state(self):
        # print "\n\n store of state machine {0} \n\n".format(self.__my_selected_sm_id)
        try:
            act_expansion_state = {}
            for state_path, state_row_iter in self.state_row_iter_dict_by_state_path.iteritems():
                state_row_path = self.tree_store.get_path(state_row_iter)
                if state_row_path is not None:
                    act_expansion_state[state_path] = self.view.row_expanded(state_row_path)
                else:
                    if self._selected_sm_model and \
                            self._selected_sm_model.state_machine.get_state_by_path(state_path, as_check=True):
                        # happens if refresh all is performed -> otherwise it is a error
                        logger.debug("State not in StateMachineTree but in StateMachine, {0}. {1}, {2}".format(state_path,
                                                                                                            state_row_path,
                                                                                                            state_row_iter))
            self.__expansion_state[self.__my_selected_sm_id] = act_expansion_state
        except TypeError:
            logger.error("expansion state of state machine {0} could not be stored".format(self.__my_selected_sm_id))

    def redo_expansion_state(self, info):
        if self.__my_selected_sm_id in self.__expansion_state:
            try:
                for state_path, state_row_expanded in self.__expansion_state[self.__my_selected_sm_id].iteritems():
                    if state_path in self.state_row_iter_dict_by_state_path:
                        state_row_iter = self.state_row_iter_dict_by_state_path[state_path]
                        if state_row_iter:  # may elements are missing afterwards
                            state_row_path = self.tree_store.get_path(state_row_iter)
                            if state_row_expanded:
                                self.view.expand_to_path(state_row_path)
                    else:
                        if self._selected_sm_model and \
                                self._selected_sm_model.state_machine.get_state_by_path(state_path, as_check=True):
                            logger.error("State not in StateMachineTree but in StateMachine, {0}.".format(state_path))

            except (TypeError, KeyError):
                logger.error(
                    "expansion state of state machine {0} could not be re-done".format(self.__my_selected_sm_id))

    def update(self, changed_state_model=None, with_expand=False):
        """Checks if all states are in tree and if tree has states which were deleted

        :param changed_state_model: Model that row has to be updated
        :param with_expand: The expand flag for the tree
        """
        if not self.view_is_registered:
            return

        # define initial state-model for update
        if changed_state_model is None:
            # reset all
            parent_row_iter = None
            self.state_row_iter_dict_by_state_path.clear()
            self.tree_store.clear()
            if self._selected_sm_model:
                changed_state_model = self._selected_sm_model.root_state
            else:
                return
        else:  # pick
            if changed_state_model.state.is_root_state:
                parent_row_iter = self.state_row_iter_dict_by_state_path[changed_state_model.state.get_path()]
            else:
                parent_row_iter = self.state_row_iter_dict_by_state_path[changed_state_model.parent.state.get_path()]

        # do recursive update
        self.insert_and_update_rec(parent_row_iter, changed_state_model, with_expand)

    def update_tree_store_row(self, state_model):
        state_row_iter = self.state_row_iter_dict_by_state_path[state_model.state.get_path()]
        # print "check for update row of state: ", state_model.state.get_path()
        state_row_path = self.tree_store.get_path(state_row_iter)

        if not type(state_model.state).__name__ == self.tree_store[state_row_path][self.TYPE_NAME_STORAGE_ID] or \
                not state_model.state.name == self.tree_store[state_row_path][self.NAME_STORAGE_ID]:
            # print "update row of state: ", state_model.state.get_path()
            self.tree_store[state_row_path][self.NAME_STORAGE_ID] = state_model.state.name
            self.tree_store[state_row_path][self.TYPE_NAME_STORAGE_ID] = type(state_model.state).__name__
            self.tree_store[state_row_path][self.MODEL_STORAGE_ID] = state_model
            self.tree_store.row_changed(state_row_path, state_row_iter)

    def insert_and_update_rec(self, parent_iter, state_model, with_expand=False):
        # check if in
        state_path = state_model.state.get_path()
        if state_path not in self.state_row_iter_dict_by_state_path:
            # if not in -> insert it
            state_row_iter = self.tree_store.insert_before(parent=parent_iter, sibling=None,
                                                           row=(state_model.state.name,
                                                                state_model.state.state_id,
                                                                type(state_model.state).__name__,
                                                                state_model,
                                                                state_model.state.get_path()))
            self.state_row_iter_dict_by_state_path[state_path] = state_row_iter
            if with_expand:
                parent_path = self.tree_store.get_path(state_row_iter)
                self.view.expand_to_path(parent_path)
        else:
            # if in -> check if up to date
            state_row_iter = self.state_row_iter_dict_by_state_path[state_model.state.get_path()]
            self.update_tree_store_row(state_model)

        # check children
        # - check if ALL children are in
        if type(state_model) is ContainerStateModel:
            for child_state_id, child_state_model in state_model.states.items():
                self.insert_and_update_rec(state_row_iter, child_state_model, with_expand=False)

        # - check if TOO MUCH children are in
        for n in reversed(range(self.tree_store.iter_n_children(state_row_iter))):
            child_iter = self.tree_store.iter_nth_child(state_row_iter, n)
            # check if there are left over rows of old states (switch from HS or CS to S and so on)
            if not type(state_model) is ContainerStateModel or \
                    not self.tree_store.get_value(child_iter, self.ID_STORAGE_ID) in state_model.states:
                self.remove_tree_children(child_iter)
                del self.state_row_iter_dict_by_state_path[self.tree_store.get_value(child_iter, self.STATE_PATH_STORAGE_ID)]
                self.tree_store.remove(child_iter)

    def remove_tree_children(self, child_tree_iter):
        for n in reversed(range(self.tree_store.iter_n_children(child_tree_iter))):
            child_iter = self.tree_store.iter_nth_child(child_tree_iter, n)
            if self.tree_store.iter_n_children(child_iter):
                self.remove_tree_children(child_iter)
            del self.state_row_iter_dict_by_state_path[self.tree_store.get_value(child_iter, self.STATE_PATH_STORAGE_ID)]
            # self.tree_store.remove(child_iter)

    def get_state_machine_selection(self):
        if self._selected_sm_model:
            return self._selected_sm_model.selection, self._selected_sm_model.selection.states
        else:
            return None, []

    def mouse_click(self, widget, event=None):
        # logger.info("press id: {0}, type: {1} goal: {2} {3} {4}"
        #             "".format(event.button, gtk.gdk.BUTTON_PRESS, event.type == gtk.gdk._2BUTTON_PRESS,
        #                       event.type == gtk.gdk.BUTTON_PRESS, event.button == 1))
        (model, paths) = self.view.get_selection().get_selected_rows()
        if paths and event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
            state_model = model[paths[0]][self.MODEL_STORAGE_ID]
            # logger.info("left double click event detected -> unfold state tree: {0}/{1}".format(model[row][self.TYPE_NAME_STORAGE_ID],
            #                                                                                     model[row][self.NAME_STORAGE_ID]))
            if self.view.row_expanded(paths[0]):
                self.view.collapse_row(paths[0])
            else:
                if isinstance(state_model, ContainerStateModel):
                    self.view.expand_to_path(paths[0])

            return True

    @TreeViewController.observe("sm_selection_changed_signal", signal=True)
    def assign_notification_selection(self, model, prop_name, info):
        if model is None and self._selected_sm_model and self._selected_sm_model.selection.get_selected_state() or \
                info and self.tree_store.get_iter_root() and self.CORE_ELEMENT_CLASS in info.arg.core_element_types:
            assert self._selected_sm_model is model or model is None
            # logger.info("selection state {0}".format(info))
            self.update_selection_sm_prior()
