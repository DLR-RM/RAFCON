# Copyright (C) 2014-2017 DLR
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

from rafcon.design_patterns.mvc.view import View

from rafcon.gui import glade
from rafcon.gui.views.utils.tree import TreeView
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class StateTransitionsListView(TreeView):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('transition_list_widget.glade'), parent='tree_view')
        self.tree_view = self['tree_view']


class StateTransitionsEditorView(View):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('state_transitions_widget.glade'), parent='vbox2')
        self.transitions_listView = StateTransitionsListView()
        self['transitions_scroller'].add(self.transitions_listView.get_parent_widget())
        self.transitions_listView.scrollbar_widget = self['transitions_scroller']
        self['internal_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['connected_to_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        gui_helper_label.ellipsize_labels_recursively(self['transitions_toolbar'])
