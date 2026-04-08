from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View
from rafcon.gui.utils import constants
from rafcon.gui.helpers import label


class BreakpointsTreeView(View, Gtk.TreeView):
    """Tree view for displaying breakpoints list"""
    def __init__(self):
        View.__init__(self, parent='breakpoints_treeview')
        Gtk.TreeView.__init__(self)
        self.set_name("breakpoints_tree")
        self['breakpoints_treeview'] = self


class BreakpointsView(View, Gtk.ScrolledWindow):
    """View for breakpoints management"""
    def __init__(self):
        View.__init__(self, parent='breakpoints_vbox')
        Gtk.ScrolledWindow.__init__(self)

        # Create tree view
        breakpoints_tree = BreakpointsTreeView()

        # Create buttons
        refresh_button = Gtk.Button.new_with_label("Refresh")
        refresh_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        remove_button = Gtk.Button.new_with_label("Remove")
        remove_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        remove_all_button = Gtk.Button.new_with_label("Remove All")
        remove_all_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        toggle_all_button = Gtk.ToggleButton.new_with_label("Disable All")
        toggle_all_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        # Button box
        button_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        button_box.get_style_context().add_class("widget-toolbar")
        button_box.pack_end(refresh_button, False, True, 0)
        button_box.pack_end(toggle_all_button, False, True, 0)
        button_box.pack_end(remove_button, False, True, 0)
        button_box.pack_end(remove_all_button, False, True, 0)

        label.ellipsize_labels_recursively(button_box)

        # Main vbox
        breakpoints_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        breakpoints_vbox.pack_end(button_box, False, True, 0)
        breakpoints_vbox.pack_end(self, True, True, 0)

        self.add(breakpoints_tree)
        self.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.show_all()

        # Store references
        self['breakpoints_vbox'] = breakpoints_vbox
        self['breakpoints_view'] = self
        self['breakpoints_tree'] = breakpoints_tree
        self['refresh_button'] = refresh_button
        self['remove_button'] = remove_button
        self['remove_all_button'] = remove_all_button
        self['toggle_all_button'] = toggle_all_button
