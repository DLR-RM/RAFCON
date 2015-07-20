from math import pi
from gtk.gdk import Color

from awesome_tool.mvc.controllers.gap.enums import SnappedSide
from awesome_tool.utils import constants


def get_col_rgba(col, transparent=False):
        r, g, b = col.red, col.green, col.blue
        r /= 65535.
        g /= 65535.
        b /= 65535.
        if transparent:
            a = .25
        else:
            a = 1.
        return r, g, b, a


def draw_data_value_rect(context, color, value_size, name_size, pos, port_side):
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