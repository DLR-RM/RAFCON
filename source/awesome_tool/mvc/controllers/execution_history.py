import gtk
import gobject

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.statemachine.state_machine_manager import StateMachineManager
from awesome_tool.statemachine.execution.execution_history import ConcurrencyItem, CallItem
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class ExecutionHistoryTreeController(ExtendedController):  # (Controller):

    """

    """
    def __init__(self, model=None, view=None, state_machine_manager=None):
        ExtendedController.__init__(self, model, view)
        self.history_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT)
        self.history_tree = view['history_tree']
        self.history_tree.set_model(self.history_tree_store)

        assert isinstance(state_machine_manager, StateMachineManager)
        self.state_machine_manager = state_machine_manager

        view['reload_button'].connect('clicked', self.reload_history)

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.history_tree.connect('button_press_event', self.right_click)

    def right_click(self, widget, event=None):
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
                if scoped_data is None:
                    return
                for key, data in scoped_data.iteritems():
                    menu_item_string = "%s (%s - %s):\t%s" % (data.name, key, data.value_type, data.value)
                    menu_item = gtk.MenuItem(menu_item_string)
                    menu_item.set_sensitive(False)
                    menu_item.show()
                    popup_menu.append(menu_item)

                popup_menu.show()

                popup_menu.popup(None, None, None, event.button, time)
            return True

    # @ExtendedController.observe("execution_history", after=True)
    # def model_changed(self, model, prop_name, info):
    #     logger.warning("execution_history changed")
    #     print info
    #     #self.update()  # TODO: only update when execution mode is not RUNNING (while running history not interesting)
    #                     # TODO: update when finished RUNNING all states or other state activated

    def reload_history(self, widget, event=None):
        self.update()

    def update(self):
        self.history_tree_store.clear()
        execution_history_items = self.state_machine_manager.get_active_state_machine().execution_history.history_items
        for item in execution_history_items:
            if isinstance(item, ConcurrencyItem):
                self.insert_rec(None, item.state_reference.name, item.execution_histories, None)
            elif isinstance(item, CallItem):
                self.insert_rec(None, item.state_reference.name + " - Call", None, item.scoped_data)
            else:
                self.insert_rec(None, item.state_reference.name + " - Return", None, item.scoped_data)

    def insert_rec(self, parent, history_item_name, history_item_children, history_item_scoped_data):
        tree_item = self.history_tree_store.insert_after(parent, None, (history_item_name, history_item_scoped_data))
        if isinstance(history_item_children, dict):
            for child_history_number, child_history in history_item_children.iteritems():
                for item in child_history.history_items:
                    if isinstance(item, ConcurrencyItem):
                        self.insert_rec(tree_item, item.state_reference.name, item.execution_histories, None)
                    elif isinstance(item, CallItem):
                        self.insert_rec(tree_item, item.state_reference.name + " - Call", None, item.scoped_data)
                    else:
                        self.insert_rec(tree_item, item.state_reference.name + " - Return", None, item.scoped_data)
                self.insert_rec(tree_item, "--------------", None, None)