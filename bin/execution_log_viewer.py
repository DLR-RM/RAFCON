#!/usr/bin/env python

# example basictreeview.py

import pygtk
pygtk.require('2.0')
import gtk
import shelve
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file", help="path to the log file")
args = parser.parse_args()

class BasicTreeViewExample:

    # close the window and quit
    def delete_event(self, widget, event, data=None):
        gtk.main_quit()
        return False

    def parse_log_file(self, filename):
        previous = {}
        next_ = {}
        concurrent = {}
        grouped_by_run_id = {}
        start_item = None

        hist_items = shelve.open(filename, 'r')
        for k,v in hist_items.items():
            if v['item_type'] == 'StateMachineStartItem':
                start_item = v
            else:
                prev_item_id = v['prev_hist_item_id']
                previous[k] = prev_item_id

                if hist_items[prev_item_id]['item_type'] == 'ConcurrencyItem' and hist_items[k]['item_type'] == 'ReturnItem':
                    # this prev relationship is a next relationship
                    next_[prev_item_id] = k
                elif hist_items[prev_item_id]['item_type'] == 'ConcurrencyItem':
                    # this prev relationship is a concurrent relationship
                    if prev_item_id in concurrent:
                        concurrent[prev_item_id].append(k)
                    else:
                        concurrent[prev_item_id] = [k]
                else:
                    next_[prev_item_id] = k

            rid = v['run_id']
            if rid in grouped_by_run_id:
                grouped_by_run_id[rid].append(v)
            else:
                grouped_by_run_id[rid] = [v]

        return start_item, hist_items, previous, next_, concurrent, grouped_by_run_id

    def collapse_log_file(self, filename):
        start_item, hist_items, previous, next_, concurrent, grouped = self.parse_log_file(filename)
        # build collapsed items


    def add_key(self, parent, key):
        piter = self.treestore.append(parent, [str(key)])
        if key in self.next_:
            self.add_key(parent, self.next_[key])

        if key in self.concurrent:
            for i, next_key in enumerate(self.concurrent[key]):
                tmppiter = self.treestore.append(piter, [str(i)])
                self.add_key(tmppiter, next_key)

    def __init__(self, filename):
        self.start, self.items, self.previous, self.next_, self.concurrent, _ = self.parse_log_file(filename)

        # Create a new window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)

        self.window.set_title("Execution Log Viewer")

        self.window.set_size_request(1024, 786)

        self.window.connect("delete_event", self.delete_event)

        #Setting up the self.grid in which the elements are to be positionned
        self.paned = gtk.HPaned()
        self.window.add(self.paned)

        #setting up the layout, putting the treeview in a scrollwindow, and the buttons in a row
        self.scrollable_treelist = gtk.ScrolledWindow()
#        self.scrollable_treelist.set_vexpand(True)
        self.paned.add1(self.scrollable_treelist)

        #setting up text view
        self.scrollable_textview = gtk.ScrolledWindow()
        self.paned.add2(self.scrollable_textview)

        self.paned.set_position(300)

        self.textview = gtk.TextView(buffer=None)
        self.scrollable_textview.add(self.textview)

        # create a TreeStore with one string column to use as the model
        self.treestore = gtk.TreeStore(str)

        # we'll aidd some data now - 4 rows with 3 child rows each
        self.add_key(None, self.start['hist_item_id'])
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
        self.tvcolumn = gtk.TreeViewColumn('History Element')

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
        hist_item_id = m.get_value(iter, 0)
        item = self.items.get(hist_item_id)
        import pprint as pp
        self.textview.get_buffer().set_text(pp.pformat(item))


def main():
    gtk.main()

if __name__ == "__main__":
    tvexample = BasicTreeViewExample(args.file)
    main()
