from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View


class ExecutionLogTreeView(View):
    def __init__(self):
        super().__init__(parent='execution_log_paned')

        # Setting up the self.grid in which the elements are to be positioned
        self.paned = Gtk.HPaned()

        # setting up the layout, putting the tree_view in a scrollwindow, and the buttons in a row
        self.scrollable_treelist = Gtk.ScrolledWindow()
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
