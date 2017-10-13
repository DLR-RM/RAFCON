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

import gtk
from gtk import Container, Button

from rafcon.gui.utils import constants
from rafcon.gui.config import global_gui_config
from rafcon.gui.runtime_config import global_runtime_config


def create_tab_header_label(tab_name, icons):
    """Create the tab header labels for notebook tabs. If USE_ICONS_AS_TAB_LABELS is set to True in the gui_config,
    icons are used as headers. Otherwise, the titles of the tabs are rotated by 90 degrees.

    :param tab_name: The label text of the tab, written in small letters and separated by underscores, e.g. states_tree
    :param icons: A dict mapping each tab_name to its corresponding icon
    :return: The GTK Eventbox holding the tab label
    """
    tooltip_event_box = gtk.EventBox()
    tooltip_event_box.set_tooltip_text(tab_name)
    tab_label = gtk.Label()
    if global_gui_config.get_config_value('USE_ICONS_AS_TAB_LABELS', True):
        tab_label.set_markup('<span font_desc="%s %s">&#x%s;</span>' %
                            (constants.ICON_FONT,
                            constants.FONT_SIZE_BIG,
                            icons[tab_name]))
    else:
        tab_label.set_text(get_widget_title(tab_name))
        tab_label.set_angle(90)
    tab_label.show()
    tooltip_event_box.add(tab_label)
    tooltip_event_box.set_visible_window(False)
    tooltip_event_box.show()
    return tooltip_event_box


def create_label_with_text_and_spacing(text, font=constants.INTERFACE_FONT, font_size=constants.FONT_SIZE_NORMAL,
                                       letter_spacing=constants.LETTER_SPACING_NONE):
    label = gtk.Label()
    set_label_markup(label, text, font, font_size, letter_spacing)
    label.show()
    return label


def create_label_widget_with_icon(icon, text, tooltip=None):
    hbox = gtk.HBox()

    icon_label = gtk.Label()
    icon_label.set_markup('<span font_desc="{0} {1}">&#x{2};</span>'.format(constants.ICON_FONT,
                                                                            constants.FONT_SIZE_NORMAL,
                                                                            icon))
    icon_label.show()
    hbox.pack_start(icon_label, False, True, 2)

    text_label = gtk.Label()
    text_label.set_markup('<span font_desc="{0} {1}" letter_spacing="{2}">{3}</span>'.format(constants.INTERFACE_FONT,
                                                                                             constants.FONT_SIZE_NORMAL,
                                                                                             constants.LETTER_SPACING_075PT,
                                                                                            text))
    if tooltip is not None:
        text_label.set_tooltip_text(tooltip)
    text_label.show()
    hbox.pack_start(text_label, True, True, 2)

    hbox.show()
    return hbox


def create_image_menu_item(label_text="", icon_code=constants.BUTTON_COPY, callback=None, callback_args=(),
                           accel_code=None, accel_group=None):
    menu_item = gtk.ImageMenuItem()
    menu_item.set_image(create_label_widget_with_icon(icon_code, ""))
    menu_item.set_label(label_text)
    if callback is not None:
        menu_item.connect("activate", callback, *callback_args)
    menu_item.set_always_show_image(True)
    if accel_code is not None and accel_group is not None:
        key, mod = gtk.accelerator_parse(accel_code)
        menu_item.add_accelerator("activate", accel_group, key, mod, gtk.ACCEL_VISIBLE)
    return menu_item


def create_check_menu_item(label_text="", is_active=False, callback=None, callback_args=(), is_sensitive=True,
                           accel_code=None, accel_group=None):
    icon_code = constants.BUTTON_CHECK if is_active else constants.BUTTON_SQUARE
    menu_item = create_image_menu_item(label_text, icon_code, callback, callback_args, accel_code, accel_group)
    menu_item.set_sensitive(is_sensitive)
    return menu_item


def append_sub_menu_to_parent_menu(name, parent_menu, icon_code=None):
    sub_menu_item = create_image_menu_item(name, icon_code)
    parent_menu.append(sub_menu_item)
    sub_menu = gtk.Menu()
    sub_menu_item.set_submenu(sub_menu)
    return sub_menu_item, sub_menu


def create_button_label(icon, font_size=constants.FONT_SIZE_NORMAL):
    """Create a button label with a chosen icon.

    :param icon: The icon
    :param font_size: The size of the icon
    :return: The created label
    """
    label = gtk.Label()
    set_label_markup(label, '&#x' + icon + ';', constants.ICON_FONT, font_size)
    label.show()
    return label


def set_button_children_size_request(widget):
    try:
        if not isinstance(widget, Container):
            return
        for child in widget.get_children():
            if isinstance(child, Button):
                child.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)
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
    set_label_markup(title_label, text, constants.INTERFACE_FONT, constants.FONT_SIZE_BIG, constants.LETTER_SPACING_1PT)
    return text


def set_label_markup(label, text, font=constants.INTERFACE_FONT, font_size=constants.FONT_SIZE_NORMAL,
                     letter_spacing=constants.LETTER_SPACING_NONE):
    label.set_markup('<span font_desc="{0} {1}" letter_spacing="{2}">{3}</span>'.format(font, font_size,
                                                                                        letter_spacing, text))


def format_cell(cell, height=None, padding=None):
    cell.set_property("cell-background-set", True)
    cell.set_property("cell-background-gdk", global_gui_config.gtk_colors['INPUT_BACKGROUND'])
    cell.set_property("background-set", True)
    cell.set_property("background-gdk", global_gui_config.gtk_colors['INPUT_BACKGROUND'])
    cell.set_property("foreground-set", True)
    cell.set_property("foreground-gdk", global_gui_config.gtk_colors['TEXT_DEFAULT'])
    if height:
        cell.set_property("height", height)
    if padding:
        cell.set_padding(padding, padding)


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
        screen_width = gtk.gdk.screen_width()
        screen_height = gtk.gdk.screen_height()
        if position[0] < screen_width and position[1] < screen_height:
            window.move(*position)
    else:
        window.set_position(gtk.WIN_POS_MOUSE)
    if maximized:
        window.maximize()
    window.show()


def draw_for_all_gtk_states(object, function_name, color):
    """
    Call given draw function for an object with a given color
    :param object:  the object to call the draw function on
    :param function_name: the draw function to call
    :param color: the color to use for drawing
    :return:
    """
    getattr(object, function_name)(gtk.STATE_ACTIVE, color)
    getattr(object, function_name)(gtk.STATE_INSENSITIVE, color)
    getattr(object, function_name)(gtk.STATE_NORMAL, color)
    getattr(object, function_name)(gtk.STATE_PRELIGHT, color)
    getattr(object, function_name)(gtk.STATE_SELECTED, color)


def react_to_event(view, widget, event):
    """Checks whether the widget is supposed to react to passed event

    The function is intended for callback methods registering to shortcut actions. As several widgets can register to
    the same shortcut, only the one having the focus should react to it.

    :param gtkmvc.View view: The view in which the widget is registered
    :param gtk.Widget widget: The widget that subscribed to the shortcut action, should be the top widget of the view
    :param event: The event that caused the callback
    :return: Whether the widget is supposed to react to the event or not
    :rtype: bool
    """
    # See
    # http://pygtk.org/pygtk2reference/class-gtkwidget.html#method-gtkwidget--is-focus and
    # http://pygtk.org/pygtk2reference/class-gtkwidget.html#method-gtkwidget--has-focus
    # for detailed information about the difference between is_focus() and has_focus()
    if not view:  # view needs to be initialized
        return False
    # widget parameter must be set and a gtk.Widget
    if not isinstance(widget, gtk.Widget):
        return False
    # Either the widget itself or one of its children must be the focus widget within their toplevel
    child_is_focus = False if not isinstance(widget, gtk.Container) else bool(widget.get_focus_child())
    if not child_is_focus and not widget.is_focus():
        return False

    def has_focus(widget):
        """Checks whether `widget` or one of its children ``has_focus()`` is ``True``

        :param gtk.Widget widget: The widget to be checked
        :return: If any (child) widget has the global input focus
        """
        if widget.has_focus():
            return True
        if not isinstance(widget, gtk.Container):
            return False
        return any(has_focus(child) for child in widget.get_children())
    # Either, for any of widget or its children, has_focus must be True, in this case the widget has the global focus.
    if has_focus(widget):
        return True
    # Or the callback was not triggered by a shortcut, but e.g. a mouse click or a call from a test.
    # If the callback was triggered by a shortcut action, the event has at least a length of two and the second
    # element is a gtk.gdk.ModifierType
    if len(event) < 2 or (len(event) >= 2 and not isinstance(event[1], gtk.gdk.ModifierType)):
        return True
    return False


def is_event_of_key_string(event, key_string):
    """Condition check if key string represent the key value of handed event and whether the event is of right type

    The function checks for constructed event tuple that are generated by the rafcon.gui.shortcut_manager.ShortcutManager.
    :param tuple event: Event tuple generated by the ShortcutManager
    :param str key_string: Key string parsed to a key value and for condition check
    """
    return len(event) >= 2 and not isinstance(event[1], gtk.gdk.ModifierType) and event[0] == gtk.accelerator_parse(key_string)[0]
