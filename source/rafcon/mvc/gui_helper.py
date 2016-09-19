import gtk
from gtk import Container, Button

from rafcon.mvc import singleton as mvc_singleton
from rafcon.mvc.utils import constants
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config


def create_tab_header_label(tab_name, icons):
    """Create the tab header labels for notebook tabs. If USE_ICONS_AS_TAB_LABELS is set to True in the gui_config,
    icons are used as headers. Otherwise, the titles of the tabs are rotated by 90 degrees.

    :param tab_name: The label text of the tab, written in small letters and seperated by underscores, e.g. state_tree
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


def create_label_widget_with_icon(icon, text):
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
    text_label.show()
    hbox.pack_start(text_label, True, True, 2)

    hbox.show()
    return hbox


def create_image_menu_item(label_text="", icon_code=constants.BUTTON_COPY, callback=None, callback_args=()):
    menu_item = gtk.ImageMenuItem()
    menu_item.set_image(create_label_widget_with_icon(icon_code, ""))
    menu_item.set_label(label_text)
    if callback is not None:
        menu_item.connect("activate", callback, *callback_args)
    menu_item.set_always_show_image(True)
    return menu_item


def create_check_menu_item(label_text="", is_active=False, callback=None, callback_args=(), is_sensitive=True):
    icon_code = constants.BUTTON_CHECK if is_active else constants.BUTTON_SQUARE
    menu_item = create_image_menu_item(label_text, icon_code, callback, callback_args)
    menu_item.set_sensitive(is_sensitive)
    return menu_item


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
    """Adjust GTK Window's size and position according to the corresponding values in the runtime_config file. If the
    runtime_config does not exist, or the corresponding values are missing in the file, default values for the window
    size are used, and the mouse position is used to adjust the window's position.

    :param window: The GTK Window to be adjusted
    :param window_key: The window's key stored in the runtime config file
     """
    size = global_runtime_config.get_config_value(window_key+'_SIZE')
    position = global_runtime_config.get_config_value(window_key+'_POS')
    if not size:
        size = constants.WINDOW_SIZE[window_key]
    window.resize(size[0], size[1])

    if position:
        position = (max(0, position[0]), max(0, position[1]))
        screen_width = gtk.gdk.screen_width()
        screen_height = gtk.gdk.screen_height()
        if position[0] < screen_width and position[1] < screen_height:
            window.move(position[0], position[1])
    else:
        window.set_position(gtk.WIN_POS_MOUSE)
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
    if not view:  # view needs to be initialized
        return False
    if not widget or not isinstance(widget, gtk.Widget) or not widget.is_focus():  # widget must be in focus
        return False
    if widget.has_focus() or (len(event) == 2 and not isinstance(event[1], gtk.gdk.ModifierType)):
        return True
    return False
