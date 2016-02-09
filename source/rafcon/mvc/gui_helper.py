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
