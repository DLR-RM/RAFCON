"""
.. module:: execution_history
   :platform: Unix, Windows
   :synopsis: A module holding a controller for the ExecutionHistoryView holding information about the
     execution history in a execution tree

.. moduleauthor:: Sebastian Brunner


"""

import gtk
import gobject

import rafcon

from rafcon.statemachine.state_machine_manager import StateMachineManager
from rafcon.statemachine.execution.execution_history import ConcurrencyItem, CallItem
from rafcon.statemachine.singleton import state_machine_execution_engine
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.statemachine.enums import CallType

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.views.execution_history import ExecutionHistoryView

from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionHistoryTreeController(ExtendedController):
    """Controller handling the execution history.

    :param rafcon.mvc.models.state_machine_manager.StateMachineManagerModel model: The state machine manager model,
        holding data regarding state machines.
    :param rafcon.mvc.views.execution_history.ExecutionHistoryTreeView view: The GTK View showing the execution history
        tree.
    :param rafcon.statemachine.state_machine_manager.StateMachineManager state_machine_manager:
    """

    def __init__(self, model=None, view=None, state_machine_manager=None):

        assert isinstance(model, StateMachineManagerModel)
        assert isinstance(view, ExecutionHistoryView)
        assert isinstance(state_machine_manager, StateMachineManager)
        self.state_machine_manager = state_machine_manager

        ExtendedController.__init__(self, model, view)
        self.history_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        # a TreeView
        self.history_tree = view['history_tree']
        self.history_tree.set_model(self.history_tree_store)

        view['reload_button'].connect('clicked', self.reload_history)
        view['clean_button'].connect('clicked', self.clean_history)
        self._start_idx = 0

        self.state_machine_execution_model = rafcon.mvc.singleton.state_machine_execution_model
        self.observe_model(self.state_machine_execution_model)

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.history_tree.connect('button_press_event', self.right_click)

    # TODO: unused
    def switch_state_machine_execution_manager_model(self, new_state_machine_execution_engine):
        """
        Switch the state machine execution engine model to observe.
        :param new_state_machine_execution_engine: the new sm execution engine manager model
        :return:
        """
        self.relieve_model(self.state_machine_execution_model)
        self.state_machine_execution_model = new_state_machine_execution_engine
        self.observe_model(self.state_machine_execution_model)

    def right_click(self, widget, event=None):
        """Triggered when right click is pressed in the history tree. The method shows all scoped data for an execution
        step as tooltip.
        """
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            x = int(event.x)
            y = int(event.y)
            time = event.time
            pthinfo = self.history_tree.get_path_at_pos(x, y)
            if pthinfo is not None:
                path, col, cellx, celly = pthinfo
                self.history_tree.grab_focus()
                self.history_tree.set_cursor(path, col, 0)

                popup_menu = gtk.Menu()

                model, row = self.history_tree.get_selection().get_selected()
                scoped_data = model[row][1]
                state_reference = model[row][2]
                if scoped_data is None:
                    return
                for key, data in scoped_data.iteritems():
                    menu_item_string = "%s (%s - %s):\t%s" % (data.name, key, data.value_type, data.value)
                    menu_item = gtk.MenuItem(menu_item_string)
                    menu_item.set_sensitive(False)
                    menu_item.show()
                    popup_menu.append(menu_item)

                if state_reference:
                    if state_reference.final_outcome:
                        final_outcome_menu_item = gtk.MenuItem("Final outcome: " + str(state_reference.final_outcome))
                        final_outcome_menu_item.set_sensitive(False)
                        final_outcome_menu_item.show()
                        popup_menu.append(final_outcome_menu_item)

                popup_menu.show()

                popup_menu.popup(None, None, None, event.button, time)
            return True

    # TODO: implement! To do this efficiently a mechanism is needed that does not regenerate the whole tree view
    # TODO: the appropriate statemachine would have to be observed as well
    # @ExtendedController.observe("execution_history_container", after=True)
    # def model_changed(self, model, prop_name, info):
    #     #self.update()

    @ExtendedController.observe("execution_engine", after=True)
    def execution_history_focus(self, model, prop_name, info):
        """ Arranges to put execution-history widget page to become top page in notebook when execution starts and stops
        and resets the boolean of modification_history_was_focused to False each time this notification are observed.
        """
        if state_machine_execution_engine.status.execution_mode in \
                [StateMachineExecutionStatus.STARTED, StateMachineExecutionStatus.STOPPED]:
            if self.parent is not None and hasattr(self.parent, "focus_notebook_page_of_controller"):
                # request focus -> which has not have to be satisfied
                self.parent.focus_notebook_page_of_controller(self)

        if state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
            self.update()

    def clean_history(self, widget, event=None):
        """Triggered when the 'Clean History' button is clicked.

        Empties the execution history tree by adjusting the start index and updates tree store and view.
        """
        self.state_machine_manager.get_active_state_machine().execution_history_container.clean_execution_histories()
        self.update()

    def reload_history(self, widget, event=None):
        """Triggered when the 'Reload History' button is clicked."""
        self.update()

    def update(self):
        """
        rebuild the tree view of the history item tree store
        :return:
        """
        self.history_tree_store.clear()
        active_sm = self.state_machine_manager.get_active_state_machine()
        if not active_sm:
            return

        execution_history_container = active_sm.execution_history_container

        for execution_history in execution_history_container.execution_histories:
            if len(execution_history.history_items) > 0:
                history_item = execution_history.history_items[0]
                tree_item = self.history_tree_store.insert_after(
                    None, None, (history_item.state_reference.name,
                                 history_item.scoped_data,
                                 history_item.state_reference))
                self.insert_recursively(tree_item, execution_history.history_items, 1)

    def insert_recursively(self, parent, history_items, index):
        """
        Recursively insert a list of history items into a the tree store
        :param parent: the parent to add the next history item to
        :param history_items: all history items of a certain state machine execution
        :param index: the index of the current history item
        :return:
        """
        if index >= len(history_items):
            return
        history_item = history_items[index]
        new_index = index + 1
        if isinstance(history_item, ConcurrencyItem):
            # do not create tree item to avoid duplicate hierarchies
            # tree_item = self.history_tree_store.insert_before(parent, None, (history_item.state_reference.name, None))
            self.insert_concurrency(parent, history_item.execution_histories)
        else:
            if isinstance(history_item, CallItem):
                tree_item = self.history_tree_store.insert_before(
                    parent, None, (history_item.state_reference.name + " - Call",
                                   history_item.scoped_data,
                                   history_item.state_reference))
                if history_item.call_type is CallType.EXECUTE:
                    # jump over the next call history item with call type CONTAINER to avoid duplicate tree entries
                    if len(history_items) > index + 1:
                        next_history_item = history_items[index + 1]
                        if next_history_item.call_type is CallType.CONTAINER:
                            self.insert_recursively(tree_item, history_items, new_index + 1)
                        else:
                            self.insert_recursively(parent, history_items, new_index)
                    else:
                        self.insert_recursively(parent, history_items, new_index)

                else:
                    self.insert_recursively(tree_item, history_items, new_index)
            else:  # history_item is ReturnItem
                tree_item = self.history_tree_store.insert_before(
                    parent, None, (history_item.state_reference.name + " - Return",
                                   history_item.scoped_data,
                                   history_item.state_reference))
                if history_item.call_type is CallType.EXECUTE:
                    self.insert_recursively(parent, history_items, new_index)
                else:
                    self.insert_recursively(self.history_tree_store.iter_parent(parent), history_items, new_index)

    def insert_concurrency(self, parent, children_execution_histories):
        """
        Recursively add the child execution histories of an concurrency state.
        :param parent: the parent to add the next history item to
        :param children_execution_histories: a list of all child execution histories
        :return:
        """
        assert isinstance(children_execution_histories, dict)
        for child_history_number, child_history in children_execution_histories.iteritems():
            if len(child_history.history_items) >= 1:
                first_history_item = child_history.history_items[0]
                # comment this item out to avoid duplicate hierarchies
                # tree_item = self.history_tree_store.insert_before(
                #     parent, None, (first_history_item.state_reference.name + " - Concurrency Branch",
                #                    first_history_item.scoped_data,
                #                    first_history_item.state_reference))
                self.insert_recursively(parent, child_history.history_items, 1)

