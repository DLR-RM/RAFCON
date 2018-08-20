#!/usr/bin/env python

# Copyright

# example basictreeview.py

import pygtk
import gtk
import shelve
import argparse

import rafcon.utils.execution_log as log_helper

pygtk.require('2.0')

parser = argparse.ArgumentParser()
parser.add_argument("file", help="path to the log file")
args = parser.parse_args()


class BasicTreeViewExample:

    # close the window and quit
    def delete_event(self, widget, event, data=None):
        gtk.main_quit()
        return False

    def add_collapsed_key(self, parent, key):
        piter = self.treestore.append(parent, ["%s (%s)" % (self.items[key]['state_name'],self.items[key]['state_type']), str(key)])

        returns = []
        if key in self.next_:
            # self.add_collapsed_key(parent, self.next_[key])
            returns.append((parent, self.next_[key]))

        if key in self.hierarchy:
            # self.add_collapsed_key(piter, self.hierarchy[key])
            returns.append((piter, self.hierarchy[key]))

        if key in self.concurrent:
            for i, next_key in enumerate(self.concurrent[key]):
                tmppiter = self.treestore.append(piter, [str(i), None])
                # self.add_collapsed_key(tmppiter, next_key)
                returns.append((tmppiter, next_key))

        return returns

    def add_key(self, parent, key):
        piter = self.treestore.append(parent, [str(key)])

        if key in self.next_ and self.items[key]['call_type'] == 'EXECUTE':
            self.add_key(parent, self.next_[key])
        elif key in self.next_ and self.items[key]['call_type'] == 'CONTAINER' and (self.items[key]['item_type'] == 'CallItem'):
            self.add_key(piter, self.next_[key])
        if key in self.next_ and self.items[key]['call_type'] == 'CONTAINER' and (self.items[key]['item_type'] == 'ConcurrencyItem'):
            self.add_key(parent, self.next_[key])
        elif key in self.next_ and self.items[key]['call_type'] == 'CONTAINER' and self.items[key]['item_type'] == 'ReturnItem':
            new_parent = self.treestore.iter_parent(parent)
            self.add_key(new_parent, self.next_[key])

        if key in self.concurrent:
            for i, next_key in enumerate(self.concurrent[key]):
                tmppiter = self.treestore.append(piter, [str(i)])
                self.add_key(tmppiter, next_key)

    def __init__(self, filename):
        self.hist_items = shelve.open(filename, 'r')
        self.start, self.next_, self.concurrent, self.hierarchy, self.items = \
            log_helper.log_to_collapsed_structure(self.hist_items,
                                                  throw_on_pickle_error=False,
                                                  include_erroneous_data_ports=True)

        # Create a new window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)

        self.window.set_title("Execution Log Viewer")

        self.window.set_default_size(1024, 786)

        self.window.connect("delete_event", self.delete_event)

        # Setting up the self.grid in which the elements are to be positioned
        self.paned = gtk.HPaned()
        self.window.add(self.paned)

        # setting up the layout, putting the treeview in a scrollwindow, and the buttons in a row
        self.scrollable_treelist = gtk.ScrolledWindow()
        # self.scrollable_treelist.set_vexpand(True)
        self.paned.add1(self.scrollable_treelist)

        # setting up text view
        self.scrollable_textview = gtk.ScrolledWindow()
        self.paned.add2(self.scrollable_textview)

        self.paned.set_position(300)

        self.textview = gtk.TextView(buffer=None)
        self.scrollable_textview.add(self.textview)

        # create a TreeStore with one string column to use as the model
        self.treestore = gtk.TreeStore(str, str)

        # we'll add some data now - 4 rows with 3 child rows each
        if not self.start:
            print 'WARNING: no start item found, just listing all items'
            elements = [(None, run_id) for run_id in self.items.keys()]
        else:
            elements = [(None, self.start['run_id'])]
        while True:
            new_elements = []
            for e in elements:
                new_elements.extend(self.add_collapsed_key(e[0], e[1]))
            if len(new_elements) == 0:
                break
            else:
                elements = new_elements
#        for parent in range(4):
#            piter = self.treestore.append(None, ['parent %i' % parent])
#            for child in range(3):
#                self.treestore.append(piter, ['child %i of parent %i' %
#                                              (child, parent)])

        # create the TreeView using treestore
        self.treeview = gtk.TreeView(self.treestore)
        self.selection = self.treeview.get_selection()
        self.selection.connect('changed', self.on_treeview_selection_changed)
        self.scrollable_treelist.add(self.treeview)

        # create the TreeViewColumn to display the data
        self.tvcolumn = gtk.TreeViewColumn('Execution History')

        # add tvcolumn to treeview
        self.treeview.append_column(self.tvcolumn)

        # create a CellRendererText to render the data
        self.cell = gtk.CellRendererText()

        # add the cell to the tvcolumn and allow it to expand
        self.tvcolumn.pack_start(self.cell, True)

        # set the cell "text" attribute to column 0 - retrieve text
        # from that column in treestore
        self.tvcolumn.add_attribute(self.cell, 'text', 0)

        # make it searchable
        self.treeview.set_search_column(0)

        # Allow sorting on the column
        self.tvcolumn.set_sort_column_id(0)

        # Allow drag and drop reordering of rows
        self.treeview.set_reorderable(True)

        self.window.show_all()

    def on_treeview_selection_changed(self, tree_selection):
        m, iter = tree_selection.get_selected()
        hist_item_id = m.get_value(iter, 1)
        item = self.items.get(hist_item_id)
        import pprint as pp
        self.textview.get_buffer().set_text(pp.pformat(item))


def main():
    gtk.main()


if __name__ == "__main__":
    tvexample = BasicTreeViewExample(args.file)
    main()
