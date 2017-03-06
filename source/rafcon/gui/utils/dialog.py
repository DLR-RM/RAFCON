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
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import gobject
from enum import Enum

from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class ButtonDialog(Enum):
    __order__ = "OPTION_1 OPTION_2 OPTION_3 OPTION_4 OPTION_5"
    OPTION_1 = 1
    OPTION_2 = 2
    OPTION_3 = 3
    OPTION_4 = 4
    OPTION_5 = 5


class RAFCONDialog(gtk.MessageDialog):

    def __init__(self, type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL, parent=None):

        super(RAFCONDialog, self).__init__(type=type, buttons=buttons, flags=flags)

        # Prevent dialog window from being hidden by the main window
        if parent:
            self.set_transient_for(parent)

    def set_markup(self, markup_text):
        from cgi import escape
        super(RAFCONDialog, self).set_markup(escape(markup_text))

    def finalize(self, callback, *args):

        # If no special callback function was defined, don't connect one
        if callback:
            self.connect('response', callback, *args)

        # Use HBox instead of ButtonBox to align buttons within the action area
        button_box = self.get_action_area()
        vbox = button_box.get_parent()
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        buttons = [button for button in button_box.get_children()]
        for button in buttons:
            button_box.remove(button)
            button.set_relief(gtk.RELIEF_NORMAL)
            hbox.pack_end(button)
        vbox.remove(button_box)
        align_action_area = gtk.Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)
        align_action_area.add(hbox)
        vbox.pack_end(align_action_area)
        align_action_area.show()
        hbox.show()
        self.show()


class RAFCONButtonDialog(RAFCONDialog):
    def __init__(self, markup_text, button_texts, callback, callback_args=(), standalone=True, type=gtk.MESSAGE_INFO,
                 parent=None, width=None):

        super(RAFCONButtonDialog, self).__init__(type, gtk.BUTTONS_NONE, gtk.DIALOG_MODAL, parent)

        if isinstance(width, int):
            hbox = self.get_action_area()
            vbox = hbox.parent
            msg_ctr = vbox.get_children()[0]
            text_ctr = msg_ctr.get_children()[1]
            text_ctr.get_children()[0].set_size_request(width, -1)
            text_ctr.get_children()[1].set_size_request(width, -1)
        self.set_markup(markup_text)
        for button_text, option in zip(button_texts, ButtonDialog):
            self.add_button(button_text, option.value)

        # This assures that if a ButtonInputDialog is created, the init of this class doesnt already run the window.
        # Maybe it is better to run this window from the caller not from its own init, this also gives the possibility
        # to get the return of the run method or otherwise make a new class which only runs on init
        if standalone:
            self.finalize(callback, *callback_args)
            self.grab_focus()
            self.run()


class RAFCONButtonInputDialog(RAFCONButtonDialog):

    def __init__(self, markup_text, button_texts, callback=None, callback_args=(), checkbox=False,
                 checkbox_text='check',
                 button_box_flag=False, type=gtk.MESSAGE_QUESTION, parent=None):

        super(RAFCONButtonInputDialog, self).__init__(markup_text, button_texts, callback, callback_args,
                                                      button_box_flag, type, parent)

        # Setup new text entry line
        self.entry = gtk.Entry()
        self.entry.set_max_length(120)
        self.entry.set_editable(1)
        self.entry.set_activates_default(True)
        self.entry.set_width_chars(10)

        self.entry.connect('activate', lambda w: self.response(1))

        self.entry.show()

        vbox = self.get_content_area()
        # create a new gtk.Hbox to put in the checkbox and the entry
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        # add the hbox to the content area
        vbox.add(hbox)

        if checkbox:
            # If a checkbox is asked for by the caller, create one.
            self.check = gtk.CheckButton(checkbox_text)
            self.check.show()
            hbox.pack_end(self.check, True, True, 1)

        hbox.pack_end(self.entry, True, True, 1)

        vbox.show()
        hbox.show()

        self.show()
        self.finalize(callback, *callback_args)
        self.grab_focus()

    def return_text(self):
        return self.entry.get_text()

    def return_check(self):
        return self.check.get_state()


class RAFCONCheckBoxTableDialog(RAFCONButtonDialog):
    """ The window creates a table with multiple rows of check box- and text field-columns.

    The header is defined by a list of strings. The table data is defined by row-wise boolean and string elements.
    Before creation the data is checked on consistency so that at index x in any row is the same type of value.
    The table data tolerates one not bool or string values ate the end of the list to store complex python object
    and thereby open more options in checkbox handling.
    """

    def __init__(self, markup_text, button_texts, callback, callback_args=(), table_header=None, table_data=None,
                 toggled_callback=None, message_type=gtk.MESSAGE_INFO, parent=None, width=None, standalone=True):

        super(RAFCONCheckBoxTableDialog, self).__init__(markup_text, button_texts, callback, callback_args, False,
                                                        message_type, parent, width)
        if table_header is None:
            table_header = ["CheckBox", "Description"]
        if table_data is None:
            table_data = [[True, "That is true."]]
        if toggled_callback is None:
            # set default toggled callback
            def on_toggled(cell, path, column_id):
                self.list_store[path][column_id] = False if cell.get_active() else True

            toggled_callback = on_toggled

        # check if data is consistent
        if not all(len(row) == len(table_header) or not isinstance(row[-1], (str, basestring, bool)) and
                len(row) == 1 + len(table_header) for row in table_data):
            raise ValueError("All rows of the table_data list has to be the same length as the table_header list "
                             "(+1 data element), here length = {0}". format(len(table_header)))

        if not all([isinstance(row_elem, (bool, str, basestring))
                   for index, row_elem in enumerate(table_data[0]) if not index + 1 == len(table_data[0])]):
            raise TypeError("All row elements have to be of type boolean or string except of last one.")

        first_row_data_types = [type(row_elem) for row_elem in table_data[0]]
        for row_index, row in enumerate(table_data):
            for column_index, row_elem in enumerate(row):
                if not isinstance(row_elem, first_row_data_types[column_index]):
                    raise TypeError("All rows have to have the same type at the same column index. Here you have at "
                                    "column index {0} and row_index {1} type: {2} and in row 0 at this index type: {3}"
                                    "".format(column_index, row_index, type(row_elem), type(first_row_data_types[column_index])))

        # create tree view
        self.tree_view = gtk.TreeView()
        hbox = self.get_action_area()
        vbox = hbox.parent
        vbox.add(self.tree_view)
        for index, column_type in enumerate(first_row_data_types):
            if column_type is bool:
                # create checkbox column
                check_box_renderer = gtk.CellRendererToggle()
                if toggled_callback is not None:
                    check_box_renderer.connect("toggled", toggled_callback, index)
                checkbox_column = gtk.TreeViewColumn(table_header[index], check_box_renderer, active=index)
                self.tree_view.append_column(checkbox_column)
            elif column_type in (str, basestring):
                text_renderer = gtk.CellRendererText()
                text_column = gtk.TreeViewColumn(table_header[index], text_renderer, text=index)
                self.tree_view.append_column(text_column)
            else:
                if not len(first_row_data_types) == index + 1:
                    logger.error("Unexpected case, the widget is not generate column of type: {0} and "
                                 "the column is not the last in the list.".format(column_type))

        # correct last list element if not boolean or string and table data length is +1 compared to table header
        if first_row_data_types and len(first_row_data_types) == len(table_header) + 1 and \
                not isinstance(first_row_data_types[-1], (bool, str, basestring)):
            first_row_data_types[-1] = gobject.TYPE_PYOBJECT

        # fill list store
        self.list_store = gtk.ListStore(*first_row_data_types)
        self.tree_view.set_model(self.list_store)
        for row in table_data:
            self.list_store.append(row)
        self.vbox.show_all()

        if standalone:
            self.finalize(callback, callback_args)
            self.grab_focus()
            self.run()
