from gtk import Button
from awesome_tool.utils import constants


def set_button_children_size_request(widget):
        try:
            for child in widget.get_children():
                if isinstance(child, Button):
                    child.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)
                else:
                    set_button_children_size_request(child)
        except AttributeError:
            return