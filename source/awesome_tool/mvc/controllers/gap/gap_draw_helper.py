from math import pi
from gtk.gdk import Color

# from awesome_tool.mvc.views.gap.ports import SnappedSide
from awesome_tool.mvc.controllers.gap.enums import SnappedSide


def draw_name_label(context, color, name_size, handle_pos, port_side, port_side_size, draw_connection_to_port=True,
                    fill=False):
    c = context.cairo
    c.set_line_width(port_side_size * .03)

    c.set_source_color(Color(color))

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