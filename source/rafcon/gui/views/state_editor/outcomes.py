# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from gtkmvc import View

from rafcon.gui import glade
from rafcon.gui.views.utils.tree import TreeView
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class StateOutcomesTreeView(TreeView):
    builder = glade.get_glade_path("outcome_list_widget.glade")
    top = 'tree_view'

    def __init__(self):
        super(StateOutcomesTreeView, self).__init__()


class StateOutcomesEditorView(View):

    def __init__(self):
        super(StateOutcomesEditorView, self).__init__()

        self.vbox = gtk.VBox()
        self.treeView = StateOutcomesTreeView()

        add_button = gtk.Button('Add')
        add_button.set_focus_on_click(False)
        add_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        add_button.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)

        remove_button = gtk.Button('Remove')
        remove_button.set_focus_on_click(False)
        remove_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        remove_button.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)

        self['add_button'] = add_button
        self['remove_button'] = remove_button

        self.Hbox = gtk.HBox()
        self.Hbox.pack_end(self['remove_button'], expand=False, fill=True)
        self.Hbox.pack_end(self['add_button'], expand=False, fill=True)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.treeView.get_top_widget())
        self.treeView.scrollbar_widget = scrollable

        outcomes_label = gui_helper_label.create_label_with_text_and_spacing("OUTCOMES",
                                                                             letter_spacing=constants.LETTER_SPACING_1PT)
        outcomes_label.set_alignment(0.0, 0.5)
        eventbox = gtk.EventBox()
        eventbox.set_border_width(constants.BORDER_WIDTH_TEXTVIEW)
        eventbox.set_name('label_wrapper')
        eventbox.add(outcomes_label)
        title_viewport = gtk.Viewport()
        title_viewport.set_name("outcomes_title_wrapper")
        title_viewport.add(eventbox)

        self.vbox.pack_start(title_viewport, False, True, 0)
        self.vbox.pack_start(scrollable, True, True, 0)
        self.vbox.pack_start(self.Hbox, expand=False, fill=True)
        self.vbox.show_all()

        self['main_frame'] = self.vbox
        self.top = 'main_frame'

    def get_top_widget(self):
        return self.vbox
