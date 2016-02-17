import gtk
import gobject

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.utils import log

from rafcon.mvc import singleton as mvc_singleton

logger = log.get_logger(__name__)

# TODO Comment


class StateMachineHistoryController(ExtendedController):
    def __init__(self, model, view):
        """Constructor
        :param model StateMachineModel should be exchangeable
        """
        assert isinstance(model, StateMachineManagerModel)

        ExtendedController.__init__(self, model, view)
        self.view_is_registered = False

        self._mode = 'branch'
        self.with_tree = True
        self.tree_folded = False
        # self._mode = 'trail'
        assert self._mode in ['trail', 'branch']
        self.history_tree_store = gtk.TreeStore(str, str, str, str, str, str, gobject.TYPE_PYOBJECT, str)

        if view is not None:
            view['history_tree'].set_model(self.history_tree_store)

        # view.set_hover_expand(True)

        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.doing_update = False
        self.no_cursor_observation = False

        self.register()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        self.register()

    def register(self):
        """
        Change the state machine that is observed for new selected states to the selected state machine.
        :return:
        """
        # logger.debug("StateMachineEditionChangeHistory register state_machine old/new sm_id %s/%s" %
        #              (self.__my_selected_sm_id, self.model.selected_state_machine_id))

        # relieve old models
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self._selected_sm_model.history)

        if self.model.selected_state_machine_id is not None:

            # set own selected state machine id
            self.__my_selected_sm_id = self.model.selected_state_machine_id

            # observe new models
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            self.observe_model(self._selected_sm_model.history)
            self.update(None, None, None)
        else:
            if self.__my_selected_sm_id is not None:
                self.history_tree_store.clear()
            self.__my_selected_sm_id = None
            self._selected_sm_model = None

    def register_view(self, view):
        view['history_tree'].connect('cursor-changed', self.on_cursor_changed)
        view['undo_button'].connect('clicked', self.on_undo_button_clicked)
        view['redo_button'].connect('clicked', self.on_redo_button_clicked)
        view['reset_button'].connect('clicked', self.on_reset_button_clicked)
        if self._mode == 'branch':
            view['branch_checkbox'].set_active(True)
        view['branch_checkbox'].connect('toggled', self.on_toggle_mode)
        if self.with_tree:
            view['tree_checkbox'].set_active(True)
        view['tree_checkbox'].connect('toggled', self.on_toggle_with_tree)
        if self.tree_folded:
            view['folded_checkbox'].set_active(True)
        view['folded_checkbox'].connect('toggled', self.on_toggle_tree_folded)
        self.view_is_registered = True

    def register_adapters(self):
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("undo", self.undo)
        shortcut_manager.add_callback_for_action("redo", self.redo)

    def on_undo_button_clicked(self, widget, event=None):
        self.undo(None, None)

    def on_redo_button_clicked(self, widget, event=None):
        self.redo(None, None)

    def on_reset_button_clicked(self, widget, event=None):
        # logger.debug("do reset")
        self._selected_sm_model.history.changes.reset()

    def on_toggle_mode(self, widget, event=None):
        if self.view['branch_checkbox'].get_active():
            self._mode = 'branch'
        else:
            self._mode = 'trail'
        logger.info("modification history mode: {0}".format(self._mode))
        self.update(None, None, None)

    def on_toggle_with_tree(self, widget, event=None):
        self.with_tree = self.view['tree_checkbox'].get_active()
        logger.info("modification history with tree: {0}".format(self.with_tree))
        self.update(None, None, None)

    def on_toggle_tree_folded(self, widget, event=None):
        self.tree_folded = self.view['folded_checkbox'].get_active()
        logger.info("modification history tree by default folded: {0}".format(self.tree_folded))
        self.update(None, None, None)

    def on_cursor_changed(self, widget):
        if self.no_cursor_observation:
            return
        (model, row) = self.view['history_tree'].get_selection().get_selected()
        self.no_cursor_observation = True
        if not self.doing_update:
            logger.debug(
                "The view jumps to the selected history element that would be situated on a right click menu in future")
            if len(self.history_tree_store) > 1 and model[row][1] is not None:
                version_id = int(model[row][1].split('.')[-1])

                # do recovery
                self.doing_update = True
                self._selected_sm_model.history.recover_specific_version(version_id)
                self.doing_update = False
                self.update(None, None, None)
        self.no_cursor_observation = False

    def undo(self, key_value, modifier_mask):
        for key, tab in mvc_singleton.main_window_controller.get_controller('states_editor_ctrl').tabs.iteritems():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    tab['controller'].get_controller('source_ctrl').view.textview.is_focus():
                # print tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_undo(), key
                if tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_undo():
                    return False
        self._selected_sm_model.history.undo()
        return True

    def redo(self, key_value, modifier_mask):
        for key, tab in mvc_singleton.main_window_controller.get_controller('states_editor_ctrl').tabs.iteritems():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    tab['controller'].get_controller('source_ctrl').view.textview.is_focus():
                # print tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_redo(), key
                if tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_redo():
                    return False
        self._selected_sm_model.history.redo()
        return True

    @ExtendedController.observe("changes", after=True)
    def update(self, model, prop_name, info):
        """ The method updates the history a gtk.TreeStore which is the model of respective TreeView.
        It functionality is strongly depends on a consistent history-tree hold by a ChangeHistory-Class.
        """
        # logger.debug("History changed %s\n%s\n%s" % (model, prop_name, info))
        if self._selected_sm_model.history.fake or \
                info is not None and info.method_name not in ["insert_action", "undo", "redo", "reset"]:
            return
        self.doing_update = True
        self.history_tree_store.clear()
        self.list_tree_iter = {}
        trail_actions = [action.version_id for action in self._selected_sm_model.history.changes.single_trail_history()]

        def insert_this_action(action, parent_tree_item, init_branch=False):
            """ The function insert a action with respective arguments (e.g. method_name, instance) into a TreeStore.
                - final trail or tree specific gtk-tree-levels -> 'trail' has no levels as well as 'branch' without tree
                - defines which element is marked as active
                - generate branch labels with version-id
            """

            model = action.before_overview['model'][-1]
            method_name = action.before_overview['method_name'][-1]
            instance = action.before_overview['instance'][-1]
            if hasattr(action.before_overview, 'new_overview'):
                parameters = [str(elem) for i1, elem in enumerate(action.before_overview.new_overview['args'][-1]) if not i1 == 0]
                for name, value in action.before_overview.new_overview['kwargs'][-1].iteritems(): # .iteritems():
                    parameters.append("{0}: {1}".format(name, value))
            else:
                parameters = action.before_overview['args']

            # find active actions in to be marked in view
            if self._mode == 'trail':
                active = len(self.history_tree_store) <= self._selected_sm_model.history.changes.trail_pointer
            else:
                all_active = self._selected_sm_model.history.changes.get_all_active_actions()
                active = action.version_id in all_active

            # generate label to mark branches
            version_label = action.version_id
            if init_branch:
                version_label = 'b.' + str(action.version_id)

            tree_row_iter = self.new_change(model, method_name, instance, info, version_label, active, parent_tree_item,
                                            parameters)
            self.list_tree_iter[action.version_id] = (tree_row_iter, parent_tree_item)
            return tree_row_iter

        def insert_all_next_actions(version_id, parent_tree_item=None):
            """ The function defines linkage of history-tree-elements in a gtk-tree-view to create a optimal overview.
            """
            if len(self._selected_sm_model.history.changes.all_time_history) == 0:
                return
            next_id = version_id
            while next_id is not None:
                # print next_id, len(self._selected_sm_model.history.changes.all_time_history)
                version_id = next_id
                history_tree_elem = self._selected_sm_model.history.changes.all_time_history[next_id]
                next_id = history_tree_elem.next_id
                prev_tree_elem = None
                if history_tree_elem.prev_id is not None:
                    prev_tree_elem = self._selected_sm_model.history.changes.all_time_history[history_tree_elem.prev_id]
                action = history_tree_elem.action
                # logger.info("prev branch #{0} <-> {1}, trail_actions are {2}".format(history_tree_elem.action.version_id, prev_tree_elem, trail_actions))
                in_trail = history_tree_elem.action.version_id in trail_actions
                if in_trail:
                    # in trail parent is always None
                    if prev_tree_elem is not None and prev_tree_elem.old_next_ids:
                        child_tree_item = insert_this_action(action, None, init_branch=True)
                    else:
                        child_tree_item = insert_this_action(action, None)
                elif prev_tree_elem is not None and prev_tree_elem.old_next_ids:
                    # branch and not trail -> level + 1 ... child of prev_id -> parent_iter is prev_id_iter
                    prev_id_iter = self.list_tree_iter[prev_tree_elem.action.version_id][0]
                    child_tree_item = insert_this_action(action, prev_id_iter, init_branch=True)
                elif prev_tree_elem is not None and not prev_tree_elem.old_next_ids:
                    # no branch and not trail
                    prev_prev_tree_elem = self._selected_sm_model.history.changes.all_time_history[prev_tree_elem.prev_id]
                    branch_limit_for_extra_ident_level = 1 if prev_tree_elem.prev_id in trail_actions else 0
                    if len(prev_prev_tree_elem.old_next_ids) > branch_limit_for_extra_ident_level:
                        # -> level + 1 as previous element, because to many branches -> so prev_id iter as parent_iter
                        iter_of_prev_id = self.list_tree_iter[prev_tree_elem.action.version_id][0]
                        child_tree_item = insert_this_action(action, iter_of_prev_id)
                    else:
                        # -> same level as previous element -> so same parent_iter as prev_id
                        parent_iter_of_prev_id = self.list_tree_iter[prev_tree_elem.action.version_id][1]
                        child_tree_item = insert_this_action(action, parent_iter_of_prev_id)
                else:
                    logger.warning("relative level could not be found -> this should not happen")
                    child_tree_item = insert_this_action(action, parent_tree_item)

                if history_tree_elem.old_next_ids and self._mode == 'branch':
                    old_next_ids = history_tree_elem.old_next_ids
                    for old_next_id in old_next_ids:
                        insert_all_next_actions(old_next_id, child_tree_item)

        insert_all_next_actions(version_id=0)

        # set selection of Tree
        if self._selected_sm_model.history.changes.trail_pointer is not None and len(self.history_tree_store) > 1:
            searched_row_version_id = self._selected_sm_model.history.changes.single_trail_history()[self._selected_sm_model.history.changes.trail_pointer].version_id
            row_number = 0
            for action_entry in self.history_tree_store:
                # compare action.version_id
                if int(action_entry[1].split('.')[-1]) == searched_row_version_id:
                    self.view['history_tree'].set_cursor(row_number)
                    break
                row_number += 1

        # set colors of Tree
        # - is state full and all element which are open to be re-done gray
        self.doing_update = False
        # for history_tree_elem in self._selected_sm_model.history.changes.all_time_history:
        #     logger.info("ActionVersionId: {0} and {1}".format(history_tree_elem.action.version_id,
        #                                                       str(history_tree_elem)))

    @staticmethod
    def get_color_active(active):
        if active:
            return "white"
        else:
            # foreground = "gray"
            return "#707070"

    def new_change(self, model, method_name, instance, info, version_id, active, parent_tree_item, parameters):
        # Nr, Instance, Method, Details, model

        # TODO may useful tooltip
        trail_tip = "any modification-state of state machine can be reached by clicking on respective action"
        branch_tip = "-> actions of old branches can be reached by clicking on respective action"

        # active-state based coloring
        foreground = self.get_color_active(active)

        # force tree to list if 'trail' or without tree enabled
        if self._mode == 'trail' or (self._mode == 'branch' and not self.with_tree):
            parent_tree_item = None

        # different ordering of elements if branch -> seems more natural
        used_func = self.history_tree_store.prepend
        if parent_tree_item is not None:
            used_func = self.history_tree_store.append
        history_row_iter = used_func(parent_tree_item, ('',
                                                        version_id,  # '',  # version
                                                        method_name,
                                                        instance,
                                                        info,
                                                        foreground,
                                                        model,
                                                        parameters))

        # handle expand-mode
        if parent_tree_item is not None:
            if not self.tree_folded:
                history_row_path = self.history_tree_store.get_path(history_row_iter)
                self.view['history_tree'].expand_to_path(history_row_path)
        return history_row_iter


