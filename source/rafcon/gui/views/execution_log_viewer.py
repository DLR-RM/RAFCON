# Copyright

# example basictreeview.py

from gi.repository import Gtk
from gtkmvc3.view import View


class ExecutionLogTreeView(View):

    def __init__(self):

        View.__init__(self)
        # Create a new window
        # self.get_window() = Gtk.Window(Gtk.WindowType.TOPLEVEL)

        # self.get_window().set_title("Execution Log Viewer")

        # self.get_window().set_default_size(1024, 786)

        # self.get_window().connect("delete_event", self.delete_event)

        # Setting up the self.grid in which the elements are to be positioned
        self.paned = Gtk.HPaned()
        # self.get_window().add(self.paned)

        # setting up the layout, putting the tree_view in a scrollwindow, and the buttons in a row
        self.scrollable_treelist = Gtk.ScrolledWindow()
        # self.scrollable_treelist.set_expand(True)
        self.paned.add1(self.scrollable_treelist)

        # setting up text view
        self.scrollable_textview = Gtk.ScrolledWindow()
        self.paned.add2(self.scrollable_textview)

        self.paned.set_position(300)

        self.text_view = Gtk.TextView(buffer=None)
        self.scrollable_textview.add(self.text_view)

        self.tree_view = Gtk.TreeView()
        self.selection = self.tree_view.get_selection()
        self.scrollable_treelist.add(self.tree_view)

        # create the TreeViewColumn to display the data
        self.tvcolumn = Gtk.TreeViewColumn('Execution History')

        # add tvcolumn to tree_view
        self.tree_view.append_column(self.tvcolumn)

        # create a CellRendererText to render the data
        self.cell = Gtk.CellRendererText()

        # add the cell to the tvcolumn and allow it to expand
        self.tvcolumn.pack_start(self.cell, True)

        # set the cell "text" attribute to column 0 - retrieve text
        # from that column in treestore
        self.tvcolumn.add_attribute(self.cell, 'text', 0)

        # make it searchable
        self.tree_view.set_search_column(0)

        # Allow sorting on the column
        self.tvcolumn.set_sort_column_id(0)

        # Allow drag and drop reordering of rows
        self.tree_view.set_reorderable(True)

        self.tree_view.show_all()
        self.paned.show_all()

        self['execution_log_paned'] = self.paned
        self['execution_log_tree_view'] = self.tree_view

        self.top = 'execution_log_tree_view'
