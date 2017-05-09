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

import json

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

        start_item = None
        collapsed_next = {}
#        collapsed_previous = {}
        collapsed_concurrent ={}
        collapsed_hierarchy = {}
        collapsed_items = {}
        # build collapsed items
        for rid, gitems in grouped.items():
            print rid
            if gitems[0]['item_type'] == 'StateMachineStartItem':
                execution_item = {}
                execution_item['item_type'] = 'StateMachineStartItem'
                execution_item['state_type'] = 'StateMachineStartState'
                execution_item['state_name'] = 'Start'
                execution_item['run_id'] = gitems[0]['run_id']
                start_item = execution_item
                collapsed_next[rid] = hist_items[next_[gitems[0]['hist_item_id']]]['run_id']
                collapsed_items[rid] = execution_item
            elif gitems[0]['state_type'] == 'ExecutionState' or gitems[0]['state_type'] == 'HierarchyState' or 'Concurrency' in gitems[0]['state_type']:
                try:
                    call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and gitems[i]['call_type'] == 'EXECUTE' for i in range(len(gitems))].index(True)]
                except ValueError:
                    # fall back to container call
                    call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and gitems[i]['call_type'] == 'CONTAINER' for i in range(len(gitems))].index(True)]
                try:
                    return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and gitems[i]['call_type'] == 'EXECUTE' for i in range(len(gitems))].index(True)]
                except ValueError:
                    return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and gitems[i]['call_type'] == 'CONTAINER' for i in range(len(gitems))].index(True)]

                # next item (on same hierarchy level) is always after return item
                if return_item['hist_item_id'] in next_:
                    if hist_items[next_[return_item['hist_item_id']]]['state_type'] == 'HierarchyState' and hist_items[next_[return_item['hist_item_id']]]['item_type'] == 'ReturnItem':
                        pass
                    else:
                        collapsed_next[rid] = hist_items[next_[return_item['hist_item_id']]]['run_id']

                if hist_items[previous[call_item['hist_item_id']]]['state_type'] == 'HierarchyState' and hist_items[previous[call_item['hist_item_id']]]['item_type'] == 'CallItem':
                    prev_rid = hist_items[previous[call_item['hist_item_id']]]['run_id']
                    collapsed_hierarchy[prev_rid] = rid

                if hist_items[previous[call_item['hist_item_id']]]['item_type'] == 'ConcurrencyItem':
                    prev_rid = hist_items[previous[call_item['hist_item_id']]]['run_id']
                    if prev_rid in collapsed_concurrent:
                        collapsed_concurrent[prev_rid].append(rid)
                    else:
                        collapsed_concurrent[prev_rid] = [rid]

                execution_item = {}
                for l in ['description', 'path_by_name', 'state_name', 'run_id', 'state_type', 'path']:
                    execution_item[l] = call_item[l]
                for l in ['outcome_name', 'outcome_id']:
                    execution_item[l] = return_item[l]
                for l in ['timestamp']:
                    execution_item[l+'_call'] = call_item[l]
                    execution_item[l+'_return'] = return_item[l]


                execution_item['data_ins'] = json.loads(call_item['input_output_data'])
                execution_item['data_outs'] = json.loads(return_item['input_output_data'])

                execution_item['scoped_data_ins'] = {}
                for k, v in json.loads(call_item['scoped_data']).items():
                    if k.startswith('error'):
                        pass
                    execution_item['scoped_data_ins'][v['name']] = v['value']
                execution_item['scoped_data_outs'] = {}
                for k, v in json.loads(return_item['scoped_data']).items():
                    if k.startswith('error'):
                        pass
                    execution_item['scoped_data_outs'][v['name']] = v['value']

                collapsed_items[rid] = execution_item

        return start_item, collapsed_next, collapsed_concurrent, collapsed_hierarchy, collapsed_items

    def add_collapsed_key(self, parent, key):
        piter = self.treestore.append(parent, ["%s (%s)" % (self.items[key]['state_name'],self.items[key]['state_type']), str(key)])

        if key in self.next_:
            self.add_collapsed_key(parent, self.next_[key])

        if key in self.hierarchy:
            self.add_collapsed_key(piter, self.hierarchy[key])

        if key in self.concurrent:
            for i, next_key in enumerate(self.concurrent[key]):
                tmppiter = self.treestore.append(piter, [str(i), None])
                self.add_collapsed_key(tmppiter, next_key)


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

            #raise Exception()

    def __init__(self, filename):
#        self.start, self.items, self.previous, self.next_, self.concurrent, _ = self.parse_log_file(filename)
        self.start, self.next_, self.concurrent, self.hierarchy, self.items = self.collapse_log_file(filename)
        print self.items.keys()
        import pprint as pp
        pp.pprint(self.hierarchy)

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
        self.treestore = gtk.TreeStore(str, str)

        # we'll aidd some data now - 4 rows with 3 child rows each
        self.add_collapsed_key(None, self.start['run_id'])
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
        hist_item_id = m.get_value(iter, 1)
        item = self.items.get(hist_item_id)
        import pprint as pp
        self.textview.get_buffer().set_text(pp.pformat(item))


def main():
    gtk.main()

if __name__ == "__main__":
    tvexample = BasicTreeViewExample(args.file)
    main()
