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
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>

"""
.. module:: execution_history
   :synopsis: A module holding a controller for the ExecutionHistoryView holding information about the
     execution history in a execution tree

"""

from builtins import range
from builtins import next
from builtins import str
from os import path
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
from threading import RLock

import rafcon

from rafcon.core.state_machine_manager import StateMachineManager
from rafcon.core.execution.execution_history import ConcurrencyItem, CallItem, ScopedDataItem, HistoryItem
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.core.execution.execution_history import CallType, StateMachineStartItem

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.views.execution_history import ExecutionHistoryView
from rafcon.gui.singleton import state_machine_execution_model
from rafcon.gui.config import global_gui_config

from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionHistoryTreeController(ExtendedController):
    """Controller handling the execution history.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel model: The state machine manager model,
        holding data regarding state machines.
    :param rafcon.gui.views.execution_history.ExecutionHistoryTreeView view: The GTK View showing the execution history
        tree.
    :param rafcon.core.state_machine_manager.StateMachineManager state_machine_manager:
    """
    LABEL_NAME_STORAGE_ID = 0
    HISTORY_ITEM_STORAGE_ID = 1
    TOOL_TIP_STORAGE_ID = 2
    TOOL_TIP_TEXT = "Right click for more details\n" \
                    "Middle click for external more detailed viewer\n" \
                    "Double click to select corresponding state"

    def __init__(self, model=None, view=None):
        assert isinstance(model, StateMachineManagerModel)
        assert isinstance(view, ExecutionHistoryView)

        super(ExecutionHistoryTreeController, self).__init__(model, view)
        self.history_tree_store = Gtk.TreeStore(GObject.TYPE_STRING, GObject.TYPE_PYOBJECT, GObject.TYPE_STRING)
        # a TreeView
        self.history_tree = view['history_tree']
        self.history_tree.set_model(self.history_tree_store)
        view['history_tree'].set_tooltip_column(self.TOOL_TIP_STORAGE_ID)

        self.observe_model(state_machine_execution_model)
        self._expansion_state = {}
        self._update_lock = RLock()

        self.update()

    def destroy(self):
        self.clean_history(None, None)
        super(ExecutionHistoryTreeController, self).destroy()

    def register_view(self, view):
        super(ExecutionHistoryTreeController, self).register_view(view)
        self.history_tree.connect('button_press_event', self.mouse_click)
        view['reload_button'].connect('clicked', self.reload_history)
        view['clean_button'].connect('clicked', self.clean_history)
        view['open_separately_button'].connect('clicked', self.open_selected_history_separately)

    def open_selected_history_separately(self, widget, event=None):
        model, row = self.history_tree.get_selection().get_selected()
        item_path = self.history_tree_store.get_path(row)
        selected_history_item = model[row][self.HISTORY_ITEM_STORAGE_ID]

        # check if valid history item (in case of concurrency not all tree items has a history item in the tree store
        if selected_history_item is None and model.iter_has_child(row):
            child_iter = model.iter_nth_child(row, 0)
            selected_history_item = model.get_value(child_iter, self.HISTORY_ITEM_STORAGE_ID)
            if selected_history_item is None:
                logger.info("The selected element could not be connected to a run-id. Therefore, no run-id is handed "\
                            "to the external execution log viewer.")
                return
        run_id = selected_history_item.run_id if selected_history_item is not None else None

        selected_state_machine = self.model.get_selected_state_machine_model().state_machine

        history_id = len(selected_state_machine.execution_histories) - 1 - item_path[0]
        execution_history = selected_state_machine.execution_histories[history_id]

        from rafcon.core.states.state import StateExecutionStatus
        if execution_history is selected_state_machine.execution_histories[-1] \
                and selected_state_machine.root_state.state_execution_status is not StateExecutionStatus.INACTIVE:
            logger.warning("Stop the execution or wait until it is finished. "
                           "The external execution history viewer can only open finished executions.")
            return

        if execution_history.execution_history_storage and execution_history.execution_history_storage.filename:
            from rafcon.gui.utils.shell_execution import execute_command_in_process
            gui_path = path.dirname(path.dirname(path.realpath(__file__)))
            source_path = path.dirname(path.dirname(gui_path))
            viewer_path = path.join(gui_path, "execution_log_viewer.py")
            # TODO run in fully separate process but from here to use the option for selection synchronization via dict
            cmd = "{path} {filename} {run_id}" \
                  "".format(path=viewer_path, filename=execution_history.execution_history_storage.filename,
                            run_id=run_id)
            execute_command_in_process(cmd, shell=True, cwd=source_path, logger=logger)
        else:
            logger.info("Set EXECUTION_LOG_ENABLE to True in your config to activate execution file logging and to use "
                        "the external execution history viewer.")

    def append_string_to_menu(self, popup_menu, menu_item_string):
        final_string = menu_item_string
        if len(menu_item_string) > 2000:
            final_string = menu_item_string[:1000] + "\n...\n" + menu_item_string[-1000:]
        menu_item = Gtk.MenuItem(final_string)
        menu_item.set_sensitive(False)
        menu_item.show()
        popup_menu.append(menu_item)

    def mouse_click(self, widget, event=None):
        """Triggered when mouse click is pressed in the history tree. The method shows all scoped data for an execution
        step as tooltip or fold and unfold the tree by double-click and select respective state for double clicked
        element.
        """
        if event.type == Gdk.EventType._2BUTTON_PRESS and event.get_button()[1] == 1:

            (model, row) = self.history_tree.get_selection().get_selected()
            if row is not None:
                histroy_item_path = self.history_tree_store.get_path(row)
                histroy_item_iter = self.history_tree_store.get_iter(histroy_item_path)
                # logger.info(history_item.state_reference)
                # TODO generalize double-click folding and unfolding -> also used in states tree of state machine
                if histroy_item_path is not None and self.history_tree_store.iter_n_children(histroy_item_iter):
                    if self.history_tree.row_expanded(histroy_item_path):
                        self.history_tree.collapse_row(histroy_item_path)
                    else:
                        self.history_tree.expand_to_path(histroy_item_path)
                sm = self.get_history_item_for_tree_iter(histroy_item_iter).state_reference.get_state_machine()
                if sm:
                    if sm.state_machine_id != self.model.selected_state_machine_id:
                        self.model.selected_state_machine_id = sm.state_machine_id
                else:
                    logger.info("No state machine could be found for selected item's state reference and "
                                "therefore no selection is performed.")
                    return
                active_sm_m = self.model.get_selected_state_machine_model()
                assert active_sm_m.state_machine is sm
                state_path = self.get_history_item_for_tree_iter(histroy_item_iter).state_reference.get_path()
                ref_state_m = active_sm_m.get_state_model_by_path(state_path)
                if ref_state_m and active_sm_m:
                    active_sm_m.selection.set(ref_state_m)

            return True

        if event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 2:
            x = int(event.x)
            y = int(event.y)
            pthinfo = self.history_tree.get_path_at_pos(x, y)
            if pthinfo is not None:
                path, col, cellx, celly = pthinfo
                self.history_tree.grab_focus()
                self.history_tree.set_cursor(path, col, 0)
                self.open_selected_history_separately(None)

        if event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 3:
            x = int(event.x)
            y = int(event.y)
            time = event.time
            pthinfo = self.history_tree.get_path_at_pos(x, y)
            if pthinfo is not None:
                path, col, cellx, celly = pthinfo
                self.history_tree.grab_focus()
                self.history_tree.set_cursor(path, col, 0)

                popup_menu = Gtk.Menu()

                model, row = self.history_tree.get_selection().get_selected()
                history_item = model[row][self.HISTORY_ITEM_STORAGE_ID]
                if not isinstance(history_item, ScopedDataItem) or history_item.scoped_data is None:
                    return
                scoped_data = history_item.scoped_data
                input_output_data = history_item.child_state_input_output_data
                state_reference = history_item.state_reference

                self.append_string_to_menu(popup_menu, "------------------------")
                self.append_string_to_menu(popup_menu, "Scoped Data: ")
                self.append_string_to_menu(popup_menu, "------------------------")
                for key, data in scoped_data.items():
                    menu_item_string = "    %s (%s - %s):\t%s" % (
                        data.name.replace("_", "__"), key, data.value_type, data.value)
                    self.append_string_to_menu(popup_menu, menu_item_string)

                if input_output_data:
                    if isinstance(history_item, CallItem):
                        self.append_string_to_menu(popup_menu, "------------------------")
                        self.append_string_to_menu(popup_menu, "Input Data:")
                        self.append_string_to_menu(popup_menu, "------------------------")
                    else:
                        self.append_string_to_menu(popup_menu, "------------------------")
                        self.append_string_to_menu(popup_menu, "Output Data:")
                        self.append_string_to_menu(popup_menu, "------------------------")

                    for key, data in input_output_data.items():
                        menu_item_string = "    %s :\t%s" % (key.replace("_", "__"), data)
                        self.append_string_to_menu(popup_menu, menu_item_string)

                if state_reference:
                    if history_item.outcome:
                        self.append_string_to_menu(popup_menu, "------------------------")
                        final_outcome_menu_item_string = "Final outcome: " + str(history_item.outcome)
                        self.append_string_to_menu(popup_menu, final_outcome_menu_item_string)
                        self.append_string_to_menu(popup_menu, "------------------------")

                popup_menu.show()
                popup_menu.popup(None, None, None, None, event.get_button()[1], time)

            return True

    # TODO: implement! To do this efficiently a mechanism is needed that does not regenerate the whole tree view
    # TODO: the appropriate state machine would have to be observed as well
    # @ExtendedController.observe("execution_history_container", after=True)
    # def model_changed(self, model, prop_name, info):
    #     #self.update()

    def get_history_item_for_tree_iter(self, child_tree_iter):
        """Hands history item for tree iter and compensate if tree item is a dummy item

        :param Gtk.TreeIter child_tree_iter: Tree iter of row
        :rtype rafcon.core.execution.execution_history.HistoryItem:
        :return history tree item:
        """
        history_item = self.history_tree_store[child_tree_iter][self.HISTORY_ITEM_STORAGE_ID]
        if history_item is None:  # is dummy item
            if self.history_tree_store.iter_n_children(child_tree_iter) > 0:
                child_iter = self.history_tree_store.iter_nth_child(child_tree_iter, 0)
                history_item = self.history_tree_store[child_iter][self.HISTORY_ITEM_STORAGE_ID]
            else:
                logger.debug("In a dummy history should be respective real call element.")
        return history_item

    def _store_expansion_state(self):
        """Iter recursively all tree items and store expansion state"""

        def store_tree_expansion(child_tree_iter, expansion_state):

            tree_item_path = self.history_tree_store.get_path(child_tree_iter)
            history_item = self.get_history_item_for_tree_iter(child_tree_iter)

            # store expansion state if tree item path is valid and expansion state was not stored already
            if tree_item_path is not None:
                # if first element of sub-tree has same history_item as the parent ignore it's expansion state
                if history_item not in expansion_state:
                    expansion_state[history_item] = self.history_tree.row_expanded(tree_item_path)

            for n in range(self.history_tree_store.iter_n_children(child_tree_iter)):
                child_iter = self.history_tree_store.iter_nth_child(child_tree_iter, n)
                store_tree_expansion(child_iter, expansion_state)

        root_iter = self.history_tree_store.get_iter_first()
        if not root_iter:
            return
        current_expansion_state = {}
        # this can be the case when the execution history tree is currently being deleted
        if not self.get_history_item_for_tree_iter(root_iter).state_reference:
            return
        state_machine = self.get_history_item_for_tree_iter(root_iter).state_reference.get_state_machine()
        self._expansion_state[state_machine.state_machine_id] = current_expansion_state
        while root_iter:
            store_tree_expansion(root_iter, current_expansion_state)
            root_iter = self.history_tree_store.iter_next(root_iter)

    def _restore_expansion_state(self):
        """Iter recursively all tree items and restore expansion state"""

        def restore_tree_expansion(child_tree_iter, expansion_state):
            tree_item_path = self.history_tree_store.get_path(child_tree_iter)
            history_item = self.get_history_item_for_tree_iter(child_tree_iter)

            # restore expansion state if tree item path is valid and expansion state was not stored already
            if tree_item_path and history_item in expansion_state:
                if expansion_state[history_item]:
                    self.history_tree.expand_to_path(tree_item_path)

            for n in range(self.history_tree_store.iter_n_children(child_tree_iter)):
                child_iter = self.history_tree_store.iter_nth_child(child_tree_iter, n)
                restore_tree_expansion(child_iter, expansion_state)

        root_iter = self.history_tree_store.get_iter_first()
        if not root_iter:
            return
        state_machine = self.get_history_item_for_tree_iter(root_iter).state_reference.get_state_machine()
        if state_machine.state_machine_id not in self._expansion_state:
            return
        while root_iter:
            restore_tree_expansion(root_iter, self._expansion_state[state_machine.state_machine_id])
            root_iter = self.history_tree_store.iter_next(root_iter)

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def notification_selected_sm_changed(self, model, prop_name, info):
        """If a new state machine is selected, make sure expansion state is stored and tree updated"""
        selected_state_machine_id = self.model.selected_state_machine_id
        if selected_state_machine_id is None:
            return
        self.update()

    @ExtendedController.observe("state_machines", after=True)
    def notification_sm_changed(self, model, prop_name, info):
        """Remove references to non-existing state machines"""
        for state_machine_id in list(self._expansion_state.keys()):
            if state_machine_id not in self.model.state_machines:
                del self._expansion_state[state_machine_id]

    @ExtendedController.observe("execution_engine", after=True)
    def execution_history_focus(self, model, prop_name, info):
        """ Arranges to put execution-history widget page to become top page in notebook when execution starts and stops
        and resets the boolean of modification_history_was_focused to False each time this notification are observed.
        """
        if state_machine_execution_engine.status.execution_mode in \
                [StateMachineExecutionStatus.STARTED, StateMachineExecutionStatus.STOPPED,
                 StateMachineExecutionStatus.FINISHED]:
            if self.parent is not None and hasattr(self.parent, "focus_notebook_page_of_controller"):
                # request focus -> which has not have to be satisfied
                self.parent.focus_notebook_page_of_controller(self)

        if state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
            if not self.model.selected_state_machine_id == self.model.state_machine_manager.active_state_machine_id:
                pass
            else:
                self.update()

    def clean_history(self, widget, event=None):
        """Triggered when the 'Clean History' button is clicked.

        Empties the execution history tree by adjusting the start index and updates tree store and view.
        """
        self.history_tree_store.clear()
        selected_sm_m = self.model.get_selected_state_machine_model()
        if selected_sm_m:
            # the core may continue running without the GUI and for this it needs its execution histories
            if state_machine_execution_engine.finished_or_stopped():
                selected_sm_m.state_machine.destroy_execution_histories()
                self.update()

    def reload_history(self, widget, event=None):
        """Triggered when the 'Reload History' button is clicked."""
        self.update()

    def update(self):
        """
        rebuild the tree view of the history item tree store
        :return:
        """
        # with self._update_lock:
        with self._update_lock:
            self._store_expansion_state()
            self.history_tree_store.clear()
            selected_sm_m = self.model.get_selected_state_machine_model()
            if not selected_sm_m:
                return

            for execution_number, execution_history in enumerate(selected_sm_m.state_machine.execution_histories):
                if len(execution_history) > 0:
                    first_history_item = execution_history[0]
                    # the next lines filter out the StateMachineStartItem, which is not intended to
                    # be displayed, but merely as convenient entry point in the saved log file
                    if isinstance(first_history_item, StateMachineStartItem):
                        if len(execution_history) > 1:
                            first_history_item = execution_history[1]
                            tree_item = self.history_tree_store.insert_after(
                                None,
                                None,
                                (first_history_item.state_reference.name + " - Run " + str(execution_number + 1),
                                 first_history_item, self.TOOL_TIP_TEXT))
                            self.insert_execution_history(tree_item, execution_history[1:], is_root=True)
                        else:
                            pass  # there was only the Start item in the history
                    else:
                        tree_item = self.history_tree_store.insert_after(
                            None,
                            None,
                            (first_history_item.state_reference.name + " - Run " + str(execution_number + 1),
                             first_history_item, self.TOOL_TIP_TEXT))
                        self.insert_execution_history(tree_item, execution_history, is_root=True)

            self._restore_expansion_state()

    def insert_history_item(self, parent, history_item, description, dummy=False):
        """Enters a single history item into the tree store

        :param Gtk.TreeItem parent: Parent tree item
        :param HistoryItem history_item: History item to be inserted
        :param str description: A description to be added to the entry
        :param None dummy: Whether this is just a dummy entry (wrapper for concurrency items)
        :return: Inserted tree item
        :rtype: Gtk.TreeItem
        """
        if not history_item.state_reference:
            logger.error("This must never happen! Current history_item is {}".format(history_item))
            return None
        content = None

        if global_gui_config.get_config_value("SHOW_PATH_NAMES_IN_EXECUTION_HISTORY", False):
            content = (history_item.state_reference.name + " - " +
                           history_item.state_reference.get_path() + " - " +
                           description, None if dummy else history_item,
                           None if dummy else self.TOOL_TIP_TEXT)
        else:
            content = (history_item.state_reference.name + " - " +
                           description, None if dummy else history_item,
                           None if dummy else self.TOOL_TIP_TEXT)

        tree_item = self.history_tree_store.insert_before(
            parent, None, content)
        return tree_item

    def insert_execution_history(self, parent, execution_history, is_root=False):
        """Insert a list of history items into a the tree store

        If there are concurrency history items, the method is called recursively.

        :param Gtk.TreeItem parent: the parent to add the next history item to
        :param ExecutionHistory execution_history: all history items of a certain state machine execution
        :param bool is_root: Whether this is the root execution history
        """
        current_parent = parent
        execution_history_iterator = iter(execution_history)
        for history_item in execution_history_iterator:
            if isinstance(history_item, ConcurrencyItem):
                self.insert_concurrent_execution_histories(current_parent, history_item.execution_histories)

            elif isinstance(history_item, CallItem):
                tree_item = self.insert_history_item(current_parent, history_item, "Enter" if is_root else "Call")
                if not tree_item:
                    return
                if history_item.call_type is CallType.EXECUTE:
                    # this is necessary that already the CallType.EXECUTE item opens a new hierarchy in the
                    # tree view and not the CallType.CONTAINER item
                    next_history_item = history_item.next
                    if next_history_item and next_history_item.call_type is CallType.CONTAINER:
                        current_parent = tree_item
                        self.insert_history_item(current_parent, next_history_item, "Enter")
                        try:
                            next(execution_history_iterator)  # skips the next history item in the iterator
                        except StopIteration as e:
                            # the execution engine does not have another item
                            return

            else:  # history_item is ReturnItem
                if current_parent is None:
                    # The reasons here can be: missing history items, items in the wrong order etc.
                    # Does not happen when using RAFCON without plugins
                    logger.error("Invalid execution history: current_parent is None")
                    return
                if history_item.call_type is CallType.EXECUTE:
                    self.insert_history_item(current_parent, history_item, "Return")
                else:  # CONTAINER
                    self.insert_history_item(current_parent, history_item, "Exit")
                    current_parent = self.history_tree_store.iter_parent(current_parent)

            is_root = False

    def insert_concurrent_execution_histories(self, parent, concurrent_execution_histories):
        """Adds the child execution histories of a concurrency state.

        :param Gtk.TreeItem parent: the parent to add the next history item to
        :param list[ExecutionHistory] concurrent_execution_histories: a list of all child execution histories
        :return:
        """
        for execution_history in concurrent_execution_histories:
            if len(execution_history) >= 1:
                first_history_item = execution_history[0]
                # this is just a dummy item to have an extra parent for each branch
                # gives better overview in case that one of the child state is a simple execution state
                tree_item = self.insert_history_item(parent, first_history_item, "Concurrency Branch", dummy=True)
                self.insert_execution_history(tree_item, execution_history)
