# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import Pango

from rafcon.gui.utils import constants
from rafcon.gui.config import global_gui_config
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.utils.gtk_utils import is_point_on_screen


def create_tab_header_label(tab_name, icons):
    """Create the tab header labels for notebook tabs. If USE_ICONS_AS_TAB_LABELS is set to True in the gui_config,
    icons are used as headers. Otherwise, the titles of the tabs are rotated by 90 degrees.

    :param tab_name: The label text of the tab, written in small letters and separated by underscores, e.g. states_tree
    :param icons: A dict mapping each tab_name to its corresponding icon
    :return: The GTK Eventbox holding the tab label
    """
    tooltip_event_box = Gtk.EventBox()
    tooltip_event_box.set_tooltip_text(tab_name)
    tab_label = Gtk.Label()
    if global_gui_config.get_config_value('USE_ICONS_AS_TAB_LABELS', True):
        set_label_markup(tab_label, icons[tab_name], is_icon=True, size=constants.FONT_SIZE_BIG)
    else:
        tab_label.set_text(get_widget_title(tab_name))
        tab_label.set_angle(90)
    tab_label.show()
    tooltip_event_box.add(tab_label)
    tooltip_event_box.set_visible_window(False)
    tooltip_event_box.show()
    return tooltip_event_box


def create_label_widget_with_icon(icon, text, tooltip=None):
    hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)

    icon_label = Gtk.Label()
    set_label_markup(icon_label, icon, is_icon=True)
    icon_label.show()
    hbox.pack_start(icon_label, False, True, 2)

    text_label = Gtk.Label()
    set_label_markup(text_label, text, letter_spacing=constants.LETTER_SPACING_075PT)
    if tooltip is not None:
        text_label.set_tooltip_text(tooltip)
    text_label.show()
    hbox.pack_start(text_label, True, True, 2)

    hbox.show()
    return hbox


def get_label_of_menu_item_box(menu_item):
    return menu_item.get_child().get_children()[-1].get_text()


def set_label_of_menu_item_box(menu_item, new_label_text):
    return menu_item.get_child().get_children()[-1].set_text(new_label_text)


def set_icon_and_text_box_of_menu_item(menu_item, uni_code):
    menu_item_child = menu_item.get_child()
    # per default MenuItems created through the glade file have a AccelLabel as child
    # this we have to delete at first, as we want a Box, with two labels
    if isinstance(menu_item_child, Gtk.AccelLabel):
        label_text = menu_item_child.get_text()

        menu_item.remove(menu_item_child)

        menu_box, icon_label, text_label = create_menu_box_with_icon_and_label(label_text)
        menu_item.add(menu_box)
        menu_box.show()

        text_label.set_accel_widget(menu_item)
    else:
        box = menu_item_child
        icon_label = box.get_children()[0]

    # now add the awesome icon to the icon_label
    if uni_code is not None:
        set_label_markup(icon_label, uni_code, is_icon=True)


def create_menu_item(label_text="", icon_code=constants.BUTTON_COPY, callback=None, callback_args=(),
                     accel_code=None, accel_group=None):
    menu_item = Gtk.MenuItem()
    menu_item.set_label(label_text)
    set_icon_and_text_box_of_menu_item(menu_item, icon_code)
    if callback is not None:
        menu_item.connect("activate", callback, *callback_args)
    if accel_code is not None and accel_group is not None:
        key, mod = Gtk.accelerator_parse(accel_code)
        menu_item.add_accelerator("activate", accel_group, key, mod, Gtk.AccelFlags.VISIBLE)
    return menu_item


def create_check_menu_item(label_text="", is_active=False, callback=None, callback_args=(), is_sensitive=True,
                           accel_code=None, accel_group=None):
    icon_code = constants.BUTTON_CHECK if is_active else constants.BUTTON_SQUARE
    menu_item = create_menu_item(label_text, icon_code, callback, callback_args, accel_code, accel_group)
    menu_item.set_sensitive(is_sensitive)
    return menu_item


def append_sub_menu_to_parent_menu(name, parent_menu, icon_code=None):
    sub_menu_item = create_menu_item(name, icon_code)
    parent_menu.append(sub_menu_item)
    sub_menu = Gtk.Menu()
    sub_menu_item.set_submenu(sub_menu)
    return sub_menu_item, sub_menu


def create_widget_title(title, widget_name=None):
    widget_name = widget_name if widget_name else title.replace(' ', '_').lower()
    label = Gtk.Label.new(title)
    label.set_name("{}_title".format(widget_name))
    label.set_xalign(0.0)
    eventbox = Gtk.EventBox()
    eventbox.set_name("{}_title_eventbox".format(widget_name))
    eventbox.get_style_context().add_class("widget-title")
    eventbox.add(label)
    return eventbox


def create_button_label(icon, font_size=constants.FONT_SIZE_NORMAL):
    """Create a button label with a chosen icon.

    :param icon: The icon
    :param font_size: The size of the icon
    :return: The created label
    """
    label = Gtk.Label()
    set_label_markup(label, icon, is_icon=True, size=font_size)
    label.show()
    return label


def set_button_children_size_request(widget):
    try:
        if not isinstance(widget, Gtk.Container):
            return
        for child in widget.get_children():
            if isinstance(child, Gtk.Button):
                child.set_size_request(constants.BUTTON_MIN_WIDTH, -1)
            else:
                set_button_children_size_request(child)
    except AttributeError:
        return


def get_widget_title(tab_label_text):
    """Transform Notebook tab label to title by replacing underscores with white spaces and capitalizing the first
    letter of each word.

    :param tab_label_text: The string of the tab label to be transformed
    :return: The transformed title as a string
    """
    title = ''
    title_list = tab_label_text.split('_')
    for word in title_list:
        title += word.upper() + ' '
    title.strip()
    return title


def create_left_bar_window_title(upper_title, lower_title):
    """Create the title of the un-docked left-bar window based on the open tabs in the upper and lower notebooks.

    :param upper_title: The title of the currently-opened tab in the upper notebook
    :param lower_title: The title of the currently-opened tab in the lower notebook
    :return: The un-docked left-bar window title as a String
    """
    return upper_title + ' / ' + lower_title


def get_notebook_tab_title(notebook, page_num):
    """Helper function that gets a notebook's tab title given its page number

    :param notebook: The GTK notebook
    :param page_num: The page number of the tab, for which the title is required
    :return: The title of the tab
    """
    child = notebook.get_nth_page(page_num)
    tab_label_eventbox = notebook.get_tab_label(child)
    return get_widget_title(tab_label_eventbox.get_tooltip_text())


def set_notebook_title(notebook, page_num, title_label):
    """Set the title of a GTK notebook to one of its tab's titles

    :param notebook: The GTK notebook
    :param page_num: The page number of a specific tab
    :param title_label: The GTK label holding the notebook's title
    :return: The new title of the notebook
    """
    text = get_notebook_tab_title(notebook, page_num)
    set_label_markup(title_label, text, size=constants.FONT_SIZE_BIG, letter_spacing=constants.LETTER_SPACING_1PT)
    return text


def create_menu_box_with_icon_and_label(label_text):
    """ Creates a MenuItem box, which is a replacement for the former ImageMenuItem. The box contains, a label
        for the icon and one for the text.

    :param label_text: The text, which is displayed for the text label
    :return:
    """
    box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 10)
    box.set_border_width(0)
    icon_label = Gtk.Label()
    text_label = Gtk.AccelLabel.new(label_text)
    text_label.set_xalign(0)

    box.pack_start(icon_label, False, False, 0)
    box.pack_start(text_label, True, True, 0)
    return box, icon_label, text_label


def set_label_markup(label, text, is_icon=False, size=constants.FONT_SIZE_NORMAL,
                     letter_spacing=constants.LETTER_SPACING_NONE):
    font_family = constants.INTERFACE_FONT
    if is_icon:
        if text in constants.ICONS_IN_RAFCON_FONT:
            font_family = constants.ICON_FONT_RAFCON
        else:
            font_family = constants.ICON_FONT_FONTAWESOME
    label.set_markup('<span font_desc="{family} {size}" weight="{weight}" letter_spacing="{letter_spacing}">{text}</span>'.format(
        family=font_family,
        size=size,
        weight=900 if text in constants.ICONS_WITH_BOLD_FACE else 400,
        letter_spacing=letter_spacing,
        text=text))


def set_window_size_and_position(window, window_key):
    """Adjust GTK Window's size, position and maximized state according to the corresponding values in the
    runtime_config file. The maximize method is triggered last to restore also the last stored size and position of the
    window. If the runtime_config does not exist, or the corresponding values are missing in the file, default values
    for the window size are used, and the mouse position is used to adjust the window's position.

    :param window: The GTK Window to be adjusted
    :param window_key: The window's key stored in the runtime config file
     """
    size = global_runtime_config.get_config_value(window_key + '_WINDOW_SIZE')
    position = global_runtime_config.get_config_value(window_key + '_WINDOW_POS')
    maximized = global_runtime_config.get_config_value(window_key + '_WINDOW_MAXIMIZED')

    # un-maximize here on purpose otherwise resize and reposition fails
    if not maximized:
        window.unmaximize()

    if not size:
        size = constants.WINDOW_SIZE[window_key + '_WINDOW']
    window.resize(*size)
    if position:
        position = (max(0, position[0]), max(0, position[1]))
        if is_point_on_screen(*position):
            window.move(*position)
    else:
        window.set_position(Gtk.WindowPosition.MOUSE)
    if maximized:
        window.maximize()
    window.show()


def react_to_event(view, widget, event):
    """Checks whether the widget is supposed to react to passed event

    The function is intended for callback methods registering to shortcut actions. As several widgets can register to
    the same shortcut, only the one having the focus should react to it.

    :param gtkmvc3.View view: The view in which the widget is registered
    :param Gtk.Widget widget: The widget that subscribed to the shortcut action, should be the top widget of the view
    :param event: The event that caused the callback
    :return: Whether the widget is supposed to react to the event or not
    :rtype: bool
    """
    # See
    # http://pyGtk.org/pygtk2reference/class-gtkwidget.html#method-gtkwidget--is-focus and
    # http://pyGtk.org/pygtk2reference/class-gtkwidget.html#method-gtkwidget--has-focus
    # for detailed information about the difference between is_focus() and has_focus()
    if not view:  # view needs to be initialized
        return False
    # widget parameter must be set and a Gtk.Widget
    if not isinstance(widget, Gtk.Widget):
        return False
    # Either the widget itself or one of its children must be the focus widget within their toplevel
    child_is_focus = False if not isinstance(widget, Gtk.Container) else bool(widget.get_focus_child())
    if not child_is_focus and not widget.is_focus():
        return False

    def has_focus(widget):
        """Checks whether `widget` or one of its children ``has_focus()`` is ``True``

        :param Gtk.Widget widget: The widget to be checked
        :return: If any (child) widget has the global input focus
        """
        if widget.has_focus():
            return True
        if not isinstance(widget, Gtk.Container):
            return False
        return any(has_focus(child) for child in widget.get_children())
    # Either, for any of widget or its children, has_focus must be True, in this case the widget has the global focus.
    if has_focus(widget):
        return True
    # Or the callback was not triggered by a shortcut, but e.g. a mouse click or a call from a test.
    # If the callback was triggered by a shortcut action, the event has at least a length of two and the second
    # element is a Gdk.ModifierType
    if len(event) < 2 or (len(event) >= 2 and not isinstance(event[1], Gdk.ModifierType)):
        return True
    return False


def is_event_of_key_string(event, key_string):
    """Condition check if key string represent the key value of handed event and whether the event is of right type

    The function checks for constructed event tuple that are generated by the rafcon.gui.shortcut_manager.ShortcutManager.
    :param tuple event: Event tuple generated by the ShortcutManager
    :param str key_string: Key string parsed to a key value and for condition check
    """
    return len(event) >= 2 and not isinstance(event[1], Gdk.ModifierType) and event[0] == Gtk.accelerator_parse(key_string)[0]


def ellipsize_labels_recursively(widget, ellipsize=Pango.EllipsizeMode.END, width_chars=1):
    if isinstance(widget, Gtk.Label):
        widget.set_ellipsize(ellipsize)
        widget.set_width_chars(width_chars)
    elif isinstance(widget, Gtk.Container):
        for child_widget in widget.get_children():
            ellipsize_labels_recursively(child_widget, ellipsize, width_chars)
