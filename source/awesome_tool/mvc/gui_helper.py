import gtk
from awesome_tool.utils import constants


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