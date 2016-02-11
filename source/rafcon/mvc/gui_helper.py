import gtk
from gtk import Container, Button
from rafcon.mvc.utils import constants


def create_tab_header_label(tab_name, icons):
    tooltip_event_box = gtk.EventBox()
    tooltip_event_box.set_tooltip_text(tab_name)
    tab_label = gtk.Label()
    tab_label.set_markup('<span font_desc="%s %s">&#x%s;</span>' %
                         (constants.ICON_FONT,
                          constants.FONT_SIZE_BIG,
                          icons[tab_name]))
    tab_label.show()
    tooltip_event_box.add(tab_label)
    tooltip_event_box.set_visible_window(False)
    tooltip_event_box.show()
    return tooltip_event_box


def create_label_with_text_and_spacing(text, font=constants.INTERFACE_FONT, font_size=constants.FONT_SIZE_NORMAL,
                                       letter_spacing=constants.LETTER_SPACING_NONE):
    label = gtk.Label()
    label.set_markup('<span font_desc="%s %s" letter_spacing="%s">%s</span>' % (font, font_size, letter_spacing, text))
    label.show()
    return label


def create_button_label(icon, font_size=constants.FONT_SIZE_NORMAL):
    """Create a button label with a chosen icon.

    :param icon: The icon
    :param font_size: The size of the icon
    :return: The created label
    """
    label = gtk.Label()
    label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT, font_size, icon))
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


def get_widget_title(tab_label):
    """Transform Notebook tab label to title by replacing underscores with white spaces and capitalizing the first
    letter of each word.

    :param tab_label: The string of the tab label to be transformed
    :return: The transformed title as a string
    """
    title = ''
    title_list = tab_label.split('_')
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
    title = get_notebook_tab_title(notebook, page_num)
    title_label.set_text(title)
    return title
