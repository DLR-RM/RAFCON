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

from gi.repository import Gtk
from gi.repository import GObject
from future.utils import string_types

from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


def get_root_window():
    try:
        from rafcon.gui.singleton import main_window_controller
        return main_window_controller.get_root_window()
    except ImportError:
        pass


class RAFCONMessageDialog(Gtk.MessageDialog):
    """A dialog which consists of a gtk button and a markup text. This can be used for informing the user about
    important things happening

    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param markup_text: The text inside the dialog
    :param message_type: The gtk type of the dialog, e.g. Gtk.MessageType.INFO, Gtk.MessageType.QUESTION etc.
    :param flags: gtk flags passed to the __init__ of Gtk.MessageDialog
    :param bool | Gtk.Window parent: The parent widget of this dialog or `True` if the RAFCON root window is to be used
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, markup_text=None,
                 callback=None, callback_args=(),
                 message_type=Gtk.MessageType.WARNING, flags=Gtk.DialogFlags.MODAL, parent=None,
                 width=-1, standalone=False, title="RAFCON", height=-1):
        modal_flag = destroy_with_parent = False
        if flags is Gtk.DialogFlags.MODAL:
            modal_flag = destroy_with_parent = True
        if parent is True:
            transient_for = get_root_window()
        elif isinstance(parent, Gtk.Window):
            transient_for = parent
        else:
            transient_for = None

        # See https://stackoverflow.com/a/11589779 for how to pass parameters to GTK constructors
        if self.__class__.__name__ == "RAFCONMessageDialog":
            super(RAFCONMessageDialog, self).__init__(message_type=message_type, buttons=Gtk.ButtonsType.OK,
                                                      modal=modal_flag, destroy_with_parent=destroy_with_parent,
                                                      transient_for=transient_for)
        else:
            super(RAFCONMessageDialog, self).__init__(message_type=message_type,
                                                      modal=modal_flag, destroy_with_parent=destroy_with_parent,
                                                      transient_for=transient_for)
        self.set_title(title)

        self.set_default_size(width, height)

        if isinstance(markup_text, string_types):
            from xml.sax.saxutils import escape
            self.set_markup(escape(str(markup_text)))
        else:
            logger.debug("The specified message '{1}' text is not a string, but {0}".format(markup_text,
                                                                                            type(markup_text)))
        if callback:
            self.add_callback(callback, *callback_args)

        self.show_grab_focus_and_run(standalone)

    def add_callback(self, callback, *args):
        self.connect('response', callback, *args)

    def show_grab_focus_and_run(self, standalone=False):
        # TODO check if show all can be removed full -> at the moment it interferes grab focus -> dialog is not focused
        # self.show_all()
        # Only grab focus in the highest class, the inheriting classes should have the focus as well because they all
        # execute the init of this class
        self.grab_focus()
        # Run by yourself if this is the requested dialog. If not,
        # it doesnt make sense because the window can't be destroyed properly
        self.run() if standalone else None


class RAFCONButtonDialog(RAFCONMessageDialog):
    """A dialog which holds a markup text and a configurable amount of buttons specified in button_texts as a string list.
    The buttons' response ids are there indexes in the buttons_texts list plus 1, so the first button responds with 1 on
    a 'clicked' event.

    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param message_type: The gtk type of the dialog, e.g. Gtk.MessageType.INFO, Gtk.MessageType.QUESTION etc.
    :param flags: gtk flags passed to the __init__ of Gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, markup_text=None, button_texts=None,
                 callback=None, callback_args=(),
                 message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL, parent=None,
                 width=-1, standalone=False, title="RAFCON", height=-1):

        super(RAFCONButtonDialog, self).__init__(markup_text, callback, callback_args, message_type,
                                                 flags, parent, width, standalone, title, height)

        self.buttons = []
        if button_texts:
            for index, button in enumerate(button_texts, 1):
                self.buttons.append(self.add_button(button, index))
        else:
            logger.debug("No buttons where specified for the dialog from type or inheriting from RAFCONButtonDialog")

        self.show_grab_focus_and_run(standalone)

    def forward_response(self, widget, index):
        self.response(index)


class RAFCONInputDialog(RAFCONButtonDialog):
    """A dialog containing the a number of buttons specified in button_texts, an optional checkbox specified in
    checkbox_text and an entry line to write in. The state of the checkbox can be returned by get_checkbox_state(),
    the content of the entry box can be returned by get_entry_text(). Hitting the enter button results in the same
    response signal as the first button, so it is recommended to use "ok" or sth. similar as first button.

    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param checkbox_text: Define the text of the checkbox next to the entry line. If no present, no checkbox is created
    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param message_type: The gtk type of the dialog, e.g. Gtk.MessageType.INFO, Gtk.MessageType.QUESTION etc.
    :param flags: gtk flags passed to the __init__ of Gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, markup_text=None, button_texts=None, checkbox_text=None,
                 callback=None, callback_args=(),
                 message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL, parent=None,
                 width=-1, standalone=False, title="RAFCON", height=-1):

        super(RAFCONInputDialog, self).__init__(markup_text, button_texts, callback, callback_args,
                                                message_type, flags, parent, width, standalone, title, height)

        # Create a new Gtk.Hbox to put in the checkbox and entry
        hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, constants.GRID_SIZE)
        self.get_content_area().add(hbox)

        # Setup new text entry line
        self.entry = Gtk.Entry()
        self.entry.set_editable(True)
        self.entry.set_activates_default(True)
        self.entry.set_width_chars(10)

        # Hitting the enter button responds 1 from the widget
        # This is the same as the first button, so the first button should always be sth. approving the content of the
        # window. Probably a configurable flag would also make sense.
        self.entry.connect('activate', self.forward_response, 1)
        hbox.pack_start(self.entry, True, True, 1)

        self.checkbox = None

        if isinstance(checkbox_text, string_types):
            # If a checkbox_text is specified by the caller, we can assume that one should be used.
            self.checkbox = Gtk.CheckButton(label=checkbox_text)
            hbox.pack_end(self.checkbox, False, True, 1)
        hbox.show_all()

        self.show_grab_focus_and_run(standalone)

    def get_entry_text(self):
        return self.entry.get_text()

    def get_checkbox_state(self):
        if self.checkbox:
            return bool(self.checkbox.get_active())
        else:
            return False


class RAFCONColumnCheckboxDialog(RAFCONButtonDialog):
    """A dialog containing a column of checkboxes in addition to a number of buttons and a markup text.
    All checkboxes can be returned by get_checkboxes() in addition the state of a single checkbox can be either returned
    by index with get_checkbox_by_index() or by name with get_checkbox_by_name()

    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param checkbox_texts: The labels for checkboxes, also defines the number of checkboxes
    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param message_type: The gtk type of the dialog, e.g. Gtk.MessageType.INFO, Gtk.MessageType.QUESTION etc.
    :param flags: gtk flags passed to the __init__ of Gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, markup_text=None, button_texts=None, checkbox_texts=None,
                 callback=None, callback_args=(),
                 message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL, parent=None,
                 width=-1, standalone=False, title="RAFCON", height=-1):

        super(RAFCONColumnCheckboxDialog, self).__init__(markup_text, button_texts, callback, callback_args,
                                                         message_type, flags, parent, width, standalone, title, height)

        checkbox_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, constants.GRID_SIZE)
        self.get_content_area().add(checkbox_vbox)
        # this is not really needed i guess if I can get the checkboxes over the content area anyway
        # TODO change this to a solution without the list.

        if checkbox_texts:
            self.checkboxes = []

            for index, checkbox in enumerate(checkbox_texts):
                self.checkboxes.append(Gtk.CheckButton(label=checkbox))
                checkbox_vbox.pack_start(self.checkboxes[index], True, True, 1)
        else:
            logger.debug("Argument checkbox_text is None or empty, no checkboxes were created")

        self.show_all()
        self.run() if standalone else None

    def get_checkbox_state_by_name(self, checkbox_text):
        return [checkbox.get_active() for checkbox in self.checkboxes if checkbox.get_label() == checkbox_text]

    def get_checkbox_state_by_index(self, checkbox_index):
        return self.checkboxes[checkbox_index].get_active()

    def get_checkbox_states(self):
        return [checkbox.get_active() for checkbox in self.checkboxes]


class RAFCONCheckBoxTableDialog(RAFCONButtonDialog):
    """ The window creates a table with multiple rows of check box- and text field-columns.

    The header is defined by a list of strings. The table data is defined by row-wise boolean and string elements.
    Before creation the data is checked on consistency so that at index x in any row is the same type of value.
    The table data tolerates one not bool or string values ate the end of the list to store complex python object
    and thereby open more options in checkbox handling.

    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param checkbox_texts: The labels for checkboxes, also defines the number of checkboxes
    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param message_type: The gtk type of the dialog, e.g. Gtk.MessageType.INFO, Gtk.MessageType.QUESTION etc.
    :param flags: gtk flags passed to the __init__ of Gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, markup_text, button_texts, callback=None, callback_args=(), table_header=None, table_data=None,
                 toggled_callback=None, message_type=Gtk.MessageType.INFO, parent=None, width=-1, standalone=True,
                 title="RAFCON", height=-1):

        super(RAFCONCheckBoxTableDialog, self).__init__(markup_text, button_texts,
                                                        callback=callback, callback_args=callback_args,
                                                        message_type=message_type, parent=parent,
                                                        width=width, standalone=standalone, title=title, height=height)
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
        if not all(len(row) == len(table_header) or not isinstance(row[-1], (string_types, bool)) and
                len(row) == 1 + len(table_header) for row in table_data):
            raise ValueError("All rows of the table_data list has to be the same length as the table_header list "
                             "(+1 data element), here length = {0}". format(len(table_header)))

        if not all([isinstance(row_elem, (bool, string_types))
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
        self.tree_view = Gtk.TreeView()
        self.vbox.pack_start(self.tree_view, True, True, 0)
        for index, column_type in enumerate(first_row_data_types):
            if column_type is bool:
                # create checkbox column
                check_box_renderer = Gtk.CellRendererToggle()
                if toggled_callback is not None:
                    check_box_renderer.connect("toggled", toggled_callback, index)
                checkbox_column = Gtk.TreeViewColumn(table_header[index], check_box_renderer, active=index)
                self.tree_view.append_column(checkbox_column)
            elif issubclass(column_type, string_types):
                text_renderer = Gtk.CellRendererText()
                text_column = Gtk.TreeViewColumn(table_header[index], text_renderer, text=index)
                self.tree_view.append_column(text_column)
            else:
                if not len(first_row_data_types) == index + 1:
                    logger.error("Unexpected case, the widget is not generate column of type: {0} and "
                                 "the column is not the last in the list.".format(column_type))

        # correct last list element if not boolean or string and table data length is +1 compared to table header
        if first_row_data_types and len(first_row_data_types) == len(table_header) + 1 and \
                not issubclass(first_row_data_types[-1], (bool, string_types)):
            first_row_data_types[-1] = GObject.TYPE_PYOBJECT

        # fill list store
        self.list_store = Gtk.ListStore(*first_row_data_types)
        self.tree_view.set_model(self.list_store)
        for row in table_data:
            self.list_store.append(row)

        self.tree_view.show_all()
        self.show_grab_focus_and_run(standalone)
