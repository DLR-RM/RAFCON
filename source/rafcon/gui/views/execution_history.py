# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from gtkmvc import View
from rafcon.gui.utils import constants


class ExecutionHistoryTreeView(View, gtk.TreeView):

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)
        self.set_name("history_tree")

        tvcolumn = gtk.TreeViewColumn('History', gtk.CellRendererText(), text=0)
        tvcolumn.set_property("sizing", "autosize")
        self.append_column(tvcolumn)

        self['history_treeview'] = self
        self.top = 'history_treeview'


class ExecutionHistoryView(View, gtk.ScrolledWindow):

    def __init__(self):
        View.__init__(self)
        gtk.ScrolledWindow.__init__(self)

        history_tree = ExecutionHistoryTreeView()

        reload_button = gtk.Button("Reload history")
        reload_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        clean_button = gtk.Button("Clean history")
        clean_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        button_box = gtk.HBox()
        button_box.pack_end(reload_button, False, True, 0)
        button_box.pack_end(clean_button, False, True, 0)

        history_vbox = gtk.VBox()
        history_vbox.pack_end(button_box, False, True, 0)
        history_vbox.pack_end(self, True, True, 0)

        self.add(history_tree)
        self.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.show_all()

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['reload_button'] = reload_button
        self['clean_button'] = clean_button
        self.top = 'history_view'
