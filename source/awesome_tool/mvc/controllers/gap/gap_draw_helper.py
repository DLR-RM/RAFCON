from math import pi
from gtk.gdk import Color

from awesome_tool.mvc.controllers.gap.enums import SnappedSide
from awesome_tool.utils import constants


def limit_value_string_length(value):
    """
    This method limits the string representation of the value to MAX_VALUE_LABEL_TEXT_LENGTH + 3 characters.
    :param value: Value to limit string representation
    :return: String holding the value with a maximum length of MAX_VALUE_LABEL_TEXT_LENGTH + 3
    """
    if isinstance(value, str) and len(value) > constants.MAX_VALUE_LABEL_TEXT_LENGTH:
        value = value[:constants.MAX_VALUE_LABEL_TEXT_LENGTH] + "..."
        final_string = " " + value + " "
    elif isinstance(value, (dict, list)) and len(str(value)) > constants.MAX_VALUE_LABEL_TEXT_LENGTH:
        value_text = str(value)[:constants.MAX_VALUE_LABEL_TEXT_LENGTH] + "..."
        final_string = " " + value_text + " "
    else:
        final_string = " " + str(value) + " "

    return final_string


def get_col_rgba(col, transparent=False, alpha=None):
    """
    This class converts a gtk.gdk.Color into its r, g, b parts and adds an alpha according to needs
    :param col: Color to extract r, g and b from
    :param transparent: Whether the color shoud be tranparent or not (used for selection in "data-flow-mode"
    :return: Red, Green, Blue and Alpha value (all betwenn 0.0 - 1.0)
    """
    r, g, b = col.red, col.green, col.blue
    r /= 65535.
    g /= 65535.
    b /= 65535.
    if transparent:
        a = .25
    else:
        a = 1.

    if alpha:
        a = alpha
    return r, g, b, a


def draw_data_value_rect(context, color, value_size, name_size, pos, port_side):
    """
    This method draws the containing rect for the data port value, depending on the side and size of the label.
    :param context: Draw Context
    :param color: Background color of value part
    :param value_size: Size (width, height) of label holding the value
    :param name_size: Size (width, height) of label holding the name
    :param pos: Position of name label start point (upper left corner of label)
    :param port_side: Side on which the value part should be drawn
    :return: Rotation Angle (to rotate value accordingly), X-Position of value label start point, Y-Position
             of value label start point
    """
    c = context.cairo

    rot_angle = .0
    move_x = 0.
    move_y = 0.

    if port_side is SnappedSide.RIGHT:
        move_x = pos[0] + name_size[0]
        move_y = pos[1]

        c.rectangle(move_x, move_y, value_size[0], value_size[1])
    elif port_side is SnappedSide.BOTTOM:
        move_x = pos[0] - value_size[1]
        move_y = pos[1] + name_size[0]
        rot_angle = pi / 2.

        c.rectangle(move_x, move_y, value_size[1], value_size[0])
    elif port_side is SnappedSide.LEFT:
        move_x = pos[0] - value_size[0]
        move_y = pos[1]

        c.rectangle(move_x, move_y, value_size[0], value_size[1])
    elif port_side is SnappedSide.TOP:
        move_x = pos[0] - value_size[1]
        move_y = pos[1] - value_size[0]
        rot_angle = -pi / 2.

        c.rectangle(move_x, move_y, value_size[1], value_size[0])

    c.set_source_rgba(*color)
    c.fill_preserve()
    c.set_source_color(Color(constants.BLACK_COLOR))
    c.stroke()

    return rot_angle, move_x, move_y


def draw_connected_scoped_label(context, color, name_size, handle_pos, port_side, port_side_size,
                                draw_connection_to_port=False):
    """
    This method draws the label of a scoped variable connected to a data port. This is represented by drawing a bigger
    label where the top part is filled and the bottom part isn't.
    :param context: Draw Context
    :param color: Color to draw the label in (border and background fill color)
    :param name_size: Size of the name labels (scoped variable and port name) combined
    :param handle_pos: Position of port which label is connected to
    :param port_side: Side on which the label should be drawn
    :param port_side_size: Size of port (to have a relative size)
    :param draw_connection_to_port: Whether there should be a line connecting the label to the port
    :return: Rotation Angle (to rotate names accordingly), X-Position of name labels start point, Y-Position
             of name labels start point
    """
    c = context.cairo
    c.set_line_width(port_side_size * .03)

    c.set_source_color(Color(color))

    rot_angle = .0
    move_x = 0.
    move_y = 0.

    if port_side is SnappedSide.RIGHT:
        move_x = handle_pos.x + 2 * port_side_size
        move_y = handle_pos.y - name_size[1] / 2.

        c.move_to(move_x + name_size[0], move_y + name_size[1] / 2.)
        c.line_to(move_x + name_size[0], move_y)
        c.line_to(move_x, move_y)
        c.line_to(handle_pos.x + port_side_size, handle_pos.y)
        c.fill_preserve()
        c.stroke()
        if draw_connection_to_port:
            c.line_to(handle_pos.x + port_side_size / 2., handle_pos.y)
            c.line_to(handle_pos.x + port_side_size, handle_pos.y)
        else:
            c.move_to(handle_pos.x + port_side_size, handle_pos.y)
        c.line_to(move_x, move_y + name_size[1])
        c.line_to(move_x + name_size[0], move_y + name_size[1])
        c.line_to(move_x + name_size[0], move_y + name_size[1] / 2.)
    elif port_side is SnappedSide.BOTTOM:
        move_x = handle_pos.x + name_size[1] / 2.
        move_y = handle_pos.y + 2 * port_side_size
        rot_angle = pi / 2.

        c.move_to(move_x - name_size[1] / 2., move_y + name_size[0])
        c.line_to(move_x, move_y + name_size[0])
        c.line_to(move_x, move_y)
        c.line_to(handle_pos.x, move_y - port_side_size)
        c.fill_preserve()
        c.stroke()
        if draw_connection_to_port:
            c.line_to(handle_pos.x, handle_pos.y + port_side_size / 2.)
            c.line_to(handle_pos.x, move_y - port_side_size)
        else:
            c.move_to(handle_pos.x, move_y - port_side_size)
        c.line_to(move_x - name_size[1], move_y)
        c.line_to(move_x - name_size[1], move_y + name_size[0])
        c.line_to(move_x - name_size[1] / 2., move_y + name_size[0])
    elif port_side is SnappedSide.LEFT:
        move_x = handle_pos.x - 2 * port_side_size - name_size[0]
        move_y = handle_pos.y - name_size[1] / 2.

        c.move_to(move_x, move_y + name_size[1] / 2.)
        c.line_to(move_x, move_y)
        c.line_to(move_x + name_size[0], move_y)
        c.line_to(handle_pos.x - port_side_size, move_y + name_size[1] / 2.)
        c.fill_preserve()
        c.stroke()
        if draw_connection_to_port:
            c.line_to(handle_pos.x - port_side_size / 2., handle_pos.y)
            c.line_to(handle_pos.x - port_side_size, handle_pos.y)
        else:
            c.move_to(handle_pos.x - port_side_size, move_y + name_size[1] / 2.)
        c.line_to(move_x + name_size[0], move_y + name_size[1])
        c.line_to(move_x, move_y + name_size[1])
        c.line_to(move_x, move_y + name_size[1] / 2.)
    elif port_side is SnappedSide.TOP:
        move_x = handle_pos.x - name_size[1] / 2.
        move_y = handle_pos.y - 2 * port_side_size
        rot_angle = -pi / 2.

        c.move_to(move_x + name_size[1] / 2., move_y - name_size[0])
        c.line_to(move_x, move_y - name_size[0])
        c.line_to(move_x, move_y)
        c.line_to(handle_pos.x, move_y + port_side_size)
        c.fill_preserve()
        c.stroke()
        if draw_connection_to_port:
            c.line_to(handle_pos.x, handle_pos.y - port_side_size / 2.)
            c.line_to(handle_pos.x, move_y + port_side_size)
        else:
            c.move_to(handle_pos.x, move_y + port_side_size)
        c.line_to(move_x + name_size[1], move_y)
        c.line_to(move_x + name_size[1], move_y - name_size[0])
        c.line_to(move_x + name_size[1] / 2., move_y - name_size[0])
    c.stroke()

    return rot_angle, move_x, move_y


def draw_name_label(context, color, name_size, handle_pos, port_side, port_side_size, draw_connection_to_port=False,
                    fill=False):
    """
    Draws a normal label indicating the port name.
    :param context: Draw Context
    :param color: Color to draw the label in (border and background fill color)
    :param name_size: Size of the name labels (scoped variable and port name) combined
    :param handle_pos: Position of port which label is connected to
    :param port_side: Side on which the label should be drawn
    :param port_side_size: Size of port (to have a relative size)
    :param draw_connection_to_port: Whether there should be a line connecting the label to the port
    :param fill: Whether the label should be filled or not
    :return: Rotation Angle (to rotate name accordingly), X-Position of name label start point, Y-Position
             of name label start point
    """
    c = context.cairo
    c.set_line_width(port_side_size * .03)

    c.set_source_rgba(*color)

    rot_angle = .0
    move_x = 0.
    move_y = 0.

    if port_side is SnappedSide.RIGHT:
        move_x = handle_pos.x + 2 * port_side_size
        move_y = handle_pos.y - name_size[1] / 2.

        c.move_to(move_x, move_y)
        c.line_to(move_x + name_size[0], move_y)
        c.line_to(move_x + name_size[0], move_y + name_size[1])
        c.line_to(move_x, move_y + name_size[1])
        c.line_to(handle_pos.x + port_side_size, handle_pos.y)
        if draw_connection_to_port:
            c.line_to(handle_pos.x + port_side_size / 2., handle_pos.y)
            c.line_to(handle_pos.x + port_side_size, handle_pos.y)
        c.line_to(move_x, move_y)
    elif port_side is SnappedSide.BOTTOM:
        move_x = handle_pos.x + name_size[1] / 2.
        move_y = handle_pos.y + 2 * port_side_size
        rot_angle = pi / 2.

        c.move_to(move_x, move_y)
        c.line_to(move_x, move_y + name_size[0])
        c.line_to(move_x - name_size[1], move_y + name_size[0])
        c.line_to(move_x - name_size[1], move_y)
        c.line_to(handle_pos.x, move_y - port_side_size)
        if draw_connection_to_port:
            c.line_to(handle_pos.x, handle_pos.y + port_side_size / 2.)
            c.line_to(handle_pos.x, move_y - port_side_size)
        c.line_to(move_x, move_y)
    elif port_side is SnappedSide.LEFT:
        move_x = handle_pos.x - 2 * port_side_size - name_size[0]
        move_y = handle_pos.y - name_size[1] / 2.

        c.move_to(move_x, move_y)
        c.line_to(move_x + name_size[0], move_y)
        c.line_to(handle_pos.x - port_side_size, move_y + name_size[1] / 2.)
        if draw_connection_to_port:
            c.line_to(handle_pos.x - port_side_size / 2., handle_pos.y)
            c.line_to(handle_pos.x - port_side_size, handle_pos.y)
        c.line_to(move_x + name_size[0], move_y + name_size[1])
        c.line_to(move_x, move_y + name_size[1])
        c.line_to(move_x, move_y)
    elif port_side is SnappedSide.TOP:
        move_x = handle_pos.x - name_size[1] / 2.
        move_y = handle_pos.y - 2 * port_side_size
        rot_angle = -pi / 2.

        c.move_to(move_x, move_y)
        c.line_to(move_x, move_y - name_size[0])
        c.line_to(move_x + name_size[1], move_y - name_size[0])
        c.line_to(move_x + name_size[1], move_y)
        c.line_to(handle_pos.x, move_y + port_side_size)
        if draw_connection_to_port:
            c.line_to(handle_pos.x, handle_pos.y - port_side_size / 2.)
            c.line_to(handle_pos.x, move_y + port_side_size)
        c.line_to(move_x, move_y)
    if fill:
        c.fill_preserve()
    c.stroke()

    return rot_angle, move_x, move_y