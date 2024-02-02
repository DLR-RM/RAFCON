# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from rafcon.gui import glade
from rafcon.design_patterns.mvc.view import View
import rafcon.gui.helpers.label as gui_helper_label

from rafcon.gui.utils import constants


class SemanticDataEditorView(View):

    """ The view of the Semantic Data Editor widget

    """

    KEY_STORAGE_ID = 0
    VALUE_STORAGE_ID = 1
    IS_DICT_STORAGE_ID = 2
    KEY_COLUMN_ID = 0
    VALUE_COLUMN_ID = 1

    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('semantic_data_editor.glade'), parent='semantic_data_vbox')
        self.scrollbar_widget = self['semantic_data_scroller']
        self['delete_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_dict_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        tree_view = self["semantic_data_tree_view"]

        # prepare tree view columns
        key_renderer = Gtk.CellRendererText()
        key_renderer.set_property('editable', True)
        col = Gtk.TreeViewColumn('Key', key_renderer, text=self.KEY_STORAGE_ID)
        tree_view.append_column(col)

        value_renderer = Gtk.CellRendererText()
        value_renderer.set_property('editable', True)
        col = Gtk.TreeViewColumn('Value', value_renderer, text=self.VALUE_STORAGE_ID)
        tree_view.append_column(col)

        is_dict_renderer = Gtk.CellRendererText()
        col = Gtk.TreeViewColumn('Is Dict', is_dict_renderer, text=self.IS_DICT_STORAGE_ID)
        tree_view.append_column(col)

        gui_helper_label.ellipsize_labels_recursively(self['semantic_data_toolbar'])
