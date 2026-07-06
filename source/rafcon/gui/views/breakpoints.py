from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View
from rafcon.gui.utils import constants
from rafcon.gui.utils.gtk_utils import set_all_margins
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
        set_all_margins(refresh_button, constants.BUTTON_BORDER_WIDTH)

        remove_button = Gtk.Button.new_with_label("Remove")
        set_all_margins(remove_button, constants.BUTTON_BORDER_WIDTH)

        remove_all_button = Gtk.Button.new_with_label("Remove All")
        set_all_margins(remove_all_button, constants.BUTTON_BORDER_WIDTH)

        toggle_all_button = Gtk.ToggleButton.new_with_label("Disable All")
        set_all_margins(toggle_all_button, constants.BUTTON_BORDER_WIDTH)

        # Button box
        button_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        button_box.add_css_class("widget-toolbar")
        button_box.set_halign(Gtk.Align.END)
        button_box.append(remove_all_button)
        button_box.append(remove_button)
        button_box.append(toggle_all_button)
        button_box.append(refresh_button)

        label.ellipsize_labels_recursively(button_box)

        # Main vbox
        breakpoints_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.set_vexpand(True)
        breakpoints_vbox.append(self)
        breakpoints_vbox.append(button_box)

        self.set_child(breakpoints_tree)
        self.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        # Store references
        self['breakpoints_vbox'] = breakpoints_vbox
        self['breakpoints_view'] = self
        self['breakpoints_tree'] = breakpoints_tree
        self['refresh_button'] = refresh_button
        self['remove_button'] = remove_button
        self['remove_all_button'] = remove_all_button
        self['toggle_all_button'] = toggle_all_button
