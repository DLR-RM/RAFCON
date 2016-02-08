import gtk

from rafcon.utils import constants


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
