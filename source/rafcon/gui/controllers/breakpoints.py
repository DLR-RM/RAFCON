from gi.repository import Gtk
from gi.repository import GObject

from rafcon.core.singleton import state_machine_execution_engine
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.views.breakpoints import BreakpointsView
from rafcon.utils import log

logger = log.get_logger(__name__)


class BreakpointsController(ExtendedController):
    """Controller for the breakpoints panel.

    Manages the list of breakpoints, allowing users to enable/disable
    or remove them.
    """

    # TreeStore column indices
    COL_ENABLED = 0     # checkbox
    COL_NAME = 1        # state name
    COL_PATH = 2        # display path
    COL_STATE_ID = 3    # state ID (hidden, used as key)

    def __init__(self, model=None, view=None):
        assert isinstance(model, StateMachineManagerModel)
        assert isinstance(view, BreakpointsView)

        super(BreakpointsController, self).__init__(model, view)

        # Create list store: [enabled (bool), name (str), path (str), state_id (str)]
        self.breakpoints_store = Gtk.ListStore(GObject.TYPE_BOOLEAN, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING)

        # Get tree view from view
        self.breakpoints_tree = view['breakpoints_tree']
        self.breakpoints_tree.set_model(self.breakpoints_store)

        # Setup columns
        self._setup_tree_columns()

        # Initial update
        self.update()

    def _setup_tree_columns(self):
        """Setup the tree view columns"""
        # Column 1: Enabled checkbox
        renderer_toggle = Gtk.CellRendererToggle()
        renderer_toggle.connect("toggled", self.on_breakpoint_toggled)
        column_enabled = Gtk.TreeViewColumn("Enabled", renderer_toggle, active=self.COL_ENABLED)
        self.breakpoints_tree.append_column(column_enabled)

        # Column 2: State name
        renderer_text = Gtk.CellRendererText()
        column_name = Gtk.TreeViewColumn("State", renderer_text, text=self.COL_NAME)
        column_name.set_expand(True)
        self.breakpoints_tree.append_column(column_name)

        # Column 3: Path (displayed)
        renderer_path = Gtk.CellRendererText()
        column_path = Gtk.TreeViewColumn("Path", renderer_path, text=self.COL_PATH)
        column_path.set_expand(True)
        self.breakpoints_tree.append_column(column_path)

        # Note: COL_STATE_ID (column 4) is hidden, used only as key for lookups

    def register_view(self, view):
        """Connect button signals"""
        super(BreakpointsController, self).register_view(view)
        view['remove_button'].connect('clicked', self.on_remove_selected)
        view['remove_all_button'].connect('clicked', self.on_remove_all)
        view['refresh_button'].connect('clicked', self.on_refresh)
        view['toggle_all_button'].connect('toggled', self.on_toggle_all)

    def update(self):
        """Refresh the breakpoints list from the breakpoint manager"""
        self.breakpoints_store.clear()

        # Get all breakpoints from the execution engine
        breakpoints = state_machine_execution_engine.breakpoint_manager.get_all_breakpoints()

        # Add each breakpoint to the list
        for state_id, info in breakpoints.items():
            self.breakpoints_store.append([
                info['enabled'],
                info['name'],
                info.get('display_path', ''),
                state_id
            ])

    def on_breakpoint_toggled(self, widget, path):
        """Toggle breakpoint enabled/disabled"""
        # Get the row
        tree_iter = self.breakpoints_store.get_iter(path)
        state_id = self.breakpoints_store.get_value(tree_iter, self.COL_STATE_ID)

        # Toggle in breakpoint manager
        state_machine_execution_engine.breakpoint_manager.toggle_breakpoint(state_id)

        # Update display
        self.update()

    def on_remove_selected(self, widget):
        """Remove selected breakpoint"""
        selection = self.breakpoints_tree.get_selection()
        model, tree_iter = selection.get_selected()

        if tree_iter is None:
            logger.info("No breakpoint selected to remove")
            return

        # Get state ID
        state_id = model.get_value(tree_iter, self.COL_STATE_ID)

        # Remove from breakpoint manager (triggers _notify to update graphical editor)
        state_machine_execution_engine.breakpoint_manager.remove_breakpoint_by_id(state_id)

        # Update display
        self.update()

    def on_remove_all(self, widget):
        """Remove all breakpoints"""
        state_machine_execution_engine.breakpoint_manager.clear_all()
        self.update()
        logger.info("All breakpoints removed")

    def on_refresh(self, widget):
        """Refresh the breakpoints list"""
        self.update()

    def on_toggle_all(self, toggle_button):
        """Toggle all breakpoints on/off"""
        if toggle_button.get_active():
            state_machine_execution_engine.breakpoint_manager.disable_all()
            toggle_button.set_label("Enable All")
        else:
            state_machine_execution_engine.breakpoint_manager.enable_all()
            toggle_button.set_label("Disable All")
        self.update()
