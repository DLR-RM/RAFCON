from gtk import Button
from gtk.gdk import color_parse
from rafcon.utils import constants

import re


def global_color_pool():
    import os
    file_path = os.path.dirname(os.path.realpath(__file__))
    color_file_path = os.path.join(file_path, '..', 'mvc', 'themes', 'black', 'gtk-2.0', 'gtkrc')
    f = open(color_file_path)
    lines = f.readlines()
    f.close()
    color_dict = {}
    for line in lines:
        if re.match("\s*color", line):
            color = re.findall(r'"(.*?)"', line)
            color_dict[color[0]] = color_parse(color[1])
    return color_dict


def set_button_children_size_request(widget):
        try:
            for child in widget.get_children():
                if isinstance(child, Button):
                    child.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)
                else:
                    set_button_children_size_request(child)
        except AttributeError:
            return