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
from rafcon.gui.utils.gtk_utils import is_point_on_screen, iter_children

_MENU_WIDGETS_REMOVED = ("GtkMenu/GtkMenuItem were removed in GTK4 — context menus have to be built as "
                         "Gio.Menu models shown by Gtk.PopoverMenu (ported with the controllers in Stage 3 "
                         "of the GTK4 migration)")


def create_tab_header_label(tab_name, icons):
    """Create the tab header labels for notebook tabs. If USE_ICONS_AS_TAB_LABELS is set to True in the gui_config,
    icons are used as headers. Otherwise, the titles of the tabs are used.

    :param tab_name: The label text of the tab, written in small letters and separated by underscores, e.g. states_tree
    :param icons: A dict mapping each tab_name to its corresponding icon
    :return: The GTK Box holding the tab label
    """
    tooltip_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
    tooltip_box.set_tooltip_text(tab_name)
    tab_label = Gtk.Label()
    if global_gui_config.get_config_value('USE_ICONS_AS_TAB_LABELS', True):
        set_label_markup(tab_label, icons[tab_name], is_icon=True, size=constants.FONT_SIZE_BIG)
    else:
        # GTK4 removed Gtk.Label.set_angle, so the text labels stay horizontal
        tab_label.set_text(get_widget_title(tab_name))
    tooltip_box.append(tab_label)
    return tooltip_box


def create_label_widget_with_icon(icon, text, tooltip=None):
    hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)

    icon_label = Gtk.Label()
    set_label_markup(icon_label, icon, is_icon=True)
    icon_label.set_margin_start(2)
    icon_label.set_margin_end(2)
    hbox.append(icon_label)

    text_label = Gtk.Label()
    set_label_markup(text_label, text, letter_spacing=constants.LETTER_SPACING_075PT)
    if tooltip is not None:
        text_label.set_tooltip_text(tooltip)
    text_label.set_hexpand(True)
    text_label.set_margin_start(2)
    text_label.set_margin_end(2)
    hbox.append(text_label)

    return hbox


def get_label_of_menu_item_box(menu_item):
    raise NotImplementedError(_MENU_WIDGETS_REMOVED)


def set_icon_and_text_box_of_menu_item(menu_item, uni_code):
    raise NotImplementedError(_MENU_WIDGETS_REMOVED)


def create_menu_item(label_text="", icon_code=constants.BUTTON_COPY, callback=None, callback_args=(),
                     accel_code=None, accel_group=None):
    """Creates a context menu entry description

    GTK4 removed Gtk.MenuItem; entries are ContextMenuItem descriptions appended to a
    :class:`rafcon.gui.utils.context_menu.ContextMenu`. The icon and accelerator arguments
    are accepted for call-site compatibility, but GMenu-rendered popovers show neither
    FontAwesome icon boxes nor foreign accelerator hints.
    """
    from rafcon.gui.utils.context_menu import ContextMenuItem
    return ContextMenuItem(label_text, callback, callback_args)


def create_check_menu_item(label_text="", is_active=False, callback=None, callback_args=(), is_sensitive=True,
                           accel_code=None, accel_group=None):
    """Creates a checkable context menu entry description (see create_menu_item)"""
    from rafcon.gui.utils.context_menu import ContextMenuItem
    item = ContextMenuItem(label_text, callback, callback_args, checked=bool(is_active))
    item.set_sensitive(is_sensitive)
    return item


def append_sub_menu_to_parent_menu(name, parent_menu, icon_code=None):
    """Appends a sub menu to a ContextMenu and returns (sub menu entry, sub menu)

    The first return value was the Gtk.MenuItem in GTK3; the controllers only keep it around,
    so the nested ContextMenu is returned in both positions.
    """
    sub_menu = parent_menu.add_submenu(name)
    return sub_menu, sub_menu


def create_widget_title(title, widget_name=None):
    widget_name = widget_name if widget_name else title.replace(' ', '_').lower()
    label = Gtk.Label.new(title)
    label.set_name("{}_title".format(widget_name))
    label.set_xalign(0.0)
    title_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
    title_box.set_name("{}_title_eventbox".format(widget_name))
    title_box.add_css_class("widget-title")
    title_box.append(label)
    return title_box


def create_button_label(icon, font_size=constants.FONT_SIZE_NORMAL):
    """Create a button label with a chosen icon.

    :param icon: The icon
    :param font_size: The size of the icon
    :return: The created label
    """
    label = Gtk.Label()
    set_label_markup(label, icon, is_icon=True, size=font_size)
    return label


def set_button_children_size_request(widget):
    for child in iter_children(widget):
        if isinstance(child, Gtk.Button):
            child.set_size_request(constants.BUTTON_MIN_WIDTH, -1)
        else:
            set_button_children_size_request(child)


def get_widget_title(tab_label_text):
    """Transform Notebook tab label to title by replacing underscores with white spaces and capitalizing the first
    letter of each word.

    :param tab_label_text: The string of the tab label to be transformed
    :return: The transformed title as a string
    """
    title = ''
    title_list = tab_label_text.split('_')
    for word in title_list:
        title += word.upper()
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
    tab_label_box = notebook.get_tab_label(child)
    return get_widget_title(tab_label_box.get_tooltip_text())


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


def set_label_markup(label, text, is_icon=False, size=None, letter_spacing=None):
    font_family = constants.INTERFACE_FONT
    if is_icon:
        if text in constants.ICONS_IN_RAFCON_FONT:
            font_family = constants.ICON_FONT_RAFCON
        else:
            font_family = constants.ICON_FONT_FONTAWESOME

    # only add manual markup here if really required by the code
    # once set here, the markup cannot be changed via css styling anymore

    # font
    if size:
        size_tag = size
    else:
        size_tag = ''
    if font_family == constants.INTERFACE_FONT:
        font_tag = ''
    else:
        font_tag = 'font_desc="{family} {size_tag}"'.format(family=font_family, size_tag=size_tag)

    # weight
    if text in constants.ICONS_WITH_BOLD_FACE:
        weight_tag = 'weight="900"'
    else:
        weight_tag = ''

    # spacing
    if letter_spacing:
        letter_spacing_tag = 'letter_spacing="{letter_spacing}"'.format(letter_spacing=str(letter_spacing))
    else:
        letter_spacing_tag = ''
    markup = '<span {font_tag} {weight_tag} {letter_spacing_tag}>{text}</span>'.format(
        font_tag=font_tag,
        weight_tag=weight_tag,
        letter_spacing_tag=letter_spacing_tag,
        text=text)
    label.set_markup(markup)


def set_window_size_and_position(window, window_key):
    """Adjust GTK Window's size and maximized state according to the corresponding values in the
    runtime_config file. The maximize method is triggered last to restore also the last stored size of the
    window. If the runtime_config does not exist, or the corresponding values are missing in the file, default values
    for the window size are used.

    GTK4 removed programmatic window positioning (Gtk.Window.move and Gtk.WindowPosition), so only the
    size and the maximized state are restored; the window manager decides the position.

    :param window: The GTK Window to be adjusted
    :param window_key: The window's key stored in the runtime config file
     """
    size = global_runtime_config.get_config_value(window_key + '_WINDOW_SIZE')
    maximized = global_runtime_config.get_config_value(window_key + '_WINDOW_MAXIMIZED')

    # un-maximize here on purpose otherwise resize fails
    if not maximized:
        window.unmaximize()

    if not size:
        size = constants.WINDOW_SIZE[window_key + '_WINDOW']
    window.set_default_size(*size)
    if maximized:
        window.maximize()
    window.present()


def react_to_event(view, widget, event):
    """Checks whether the widget is supposed to react to passed event

    The function is intended for callback methods registering to shortcut actions. As several widgets can register to
    the same shortcut, only the one having the focus should react to it.

    :param view: The view in which the widget is registered
    :param Gtk.Widget widget: The widget that subscribed to the shortcut action, should be the top widget of the view
    :param event: The event that caused the callback
    :return: Whether the widget is supposed to react to the event or not
    :rtype: bool
    """
    if not view:  # view needs to be initialized
        return False
    # widget parameter must be set and a Gtk.Widget
    if not isinstance(widget, Gtk.Widget):
        return False
    # Either the widget itself or one of its children must be the focus widget within their toplevel
    child_is_focus = bool(widget.get_focus_child())
    if not child_is_focus and not widget.is_focus():
        return False

    def has_focus(widget):
        """Checks whether `widget` or one of its children ``has_focus()`` is ``True``

        :param Gtk.Widget widget: The widget to be checked
        :return: If any (child) widget has the global input focus
        """
        if widget.has_focus():
            return True
        return any(has_focus(child) for child in iter_children(widget))
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
    # GTK4: Gtk.accelerator_parse returns (success, keyval, mods)
    return len(event) >= 2 and not isinstance(event[1], Gdk.ModifierType) and \
        event[0] == Gtk.accelerator_parse(key_string)[1]


def ellipsize_labels_recursively(widget, ellipsize=Pango.EllipsizeMode.END, width_chars=1):
    if isinstance(widget, Gtk.Label):
        widget.set_ellipsize(ellipsize)
        widget.set_width_chars(width_chars)
    else:
        for child_widget in iter_children(widget):
            ellipsize_labels_recursively(child_widget, ellipsize, width_chars)
