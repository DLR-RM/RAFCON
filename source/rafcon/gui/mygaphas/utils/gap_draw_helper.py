# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from future.utils import string_types
from builtins import str
from math import pi

from gi.repository.Pango import SCALE, FontDescription
# from cairo import Antialias
from gi.repository import PangoCairo

from gaphas.painter import CairoBoundingBoxContext

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.utils import constants
from rafcon.gui.mygaphas.utils.enums import SnappedSide
from rafcon.utils.geometry import deg2rad

# Fixed font size when drawing on Pango layout
FONT_SIZE = 5.


def limit_value_string_length(value):
    """This method limits the string representation of the value to MAX_VALUE_LABEL_TEXT_LENGTH + 3 characters.

    :param value: Value to limit string representation
    :return: String holding the value with a maximum length of MAX_VALUE_LABEL_TEXT_LENGTH + 3
    """
    if isinstance(value, string_types) and len(value) > constants.MAX_VALUE_LABEL_TEXT_LENGTH:
        value = value[:constants.MAX_VALUE_LABEL_TEXT_LENGTH] + "..."
        final_string = " " + value + " "
    elif isinstance(value, (dict, list)) and len(str(value)) > constants.MAX_VALUE_LABEL_TEXT_LENGTH:
        value_text = str(value)[:constants.MAX_VALUE_LABEL_TEXT_LENGTH] + "..."
        final_string = " " + value_text + " "
    else:
        final_string = " " + str(value) + " "

    return final_string


def get_col_rgba(color, transparency=None, opacity=None):
    """This class converts a Gdk.Color into its r, g, b parts and adds an alpha according to needs

    If both transparency and opacity is None, alpha is set to 1 => opaque

    :param Gdk.Color color: Color to extract r, g and b from
    :param float | None  transparency: Value between 0 (opaque) and 1 (transparent) or None if opacity is to be used
    :param float | None opacity: Value between 0 (transparent) and 1 (opaque) or None if transparency is to be used
    :return: Red, Green, Blue and Alpha value (all between 0.0 - 1.0)
    """
    r, g, b = color.red, color.green, color.blue
    # Convert from 0-6535 to 0-1
    r /= 65535.
    g /= 65535.
    b /= 65535.

    if transparency is not None or opacity is None:
        transparency = 0 if transparency is None else transparency  # default value
        if transparency < 0 or transparency > 1:
            raise ValueError("Transparency must be between 0 and 1")
        alpha = 1 - transparency
    else:
        if opacity < 0 or opacity > 1:
            raise ValueError("Opacity must be between 0 and 1")
        alpha = opacity

    return r, g, b, alpha


def get_side_length_of_resize_handle(view, item):
    """Calculate the side length of a resize handle

    :param rafcon.gui.mygaphas.view.ExtendedGtkView view: View
    :param rafcon.gui.mygaphas.items.state.StateView item: StateView
    :return: side length
    :rtype: float
    """
    from rafcon.gui.mygaphas.items.state import StateView, NameView
    if isinstance(item, StateView):
        return item.border_width * view.get_zoom_factor() / 1.5
    elif isinstance(item, NameView):
        return item.parent.border_width * view.get_zoom_factor() / 2.5
    return 0


def draw_data_value_rect(cairo_context, color, value_size, name_size, pos, port_side):
    """This method draws the containing rect for the data port value, depending on the side and size of the label.

    :param cairo_context: Draw Context
    :param color: Background color of value part
    :param value_size: Size (width, height) of label holding the value
    :param name_size: Size (width, height) of label holding the name
    :param pos: Position of name label start point (upper left corner of label)
    :param port_side: Side on which the value part should be drawn
    :return: Rotation Angle (to rotate value accordingly), X-Position of value label start point, Y-Position
             of value label start point
    """
    c = cairo_context

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
    c.set_source_rgb(*gui_config.gtk_colors['BLACK'].to_floats())
    c.stroke()

    return rot_angle, move_x, move_y


def draw_connected_scoped_label(context, color, name_size, handle_pos, port_side, port_side_size,
                                draw_connection_to_port=False):
    """Draw label of scoped variable

    This method draws the label of a scoped variable connected to a data port. This is represented by drawing a bigger
    label where the top part is filled and the bottom part isn't.

    :param context: Draw Context
    :param Gdk.Color color: Color to draw the label in (border and background fill color)
    :param name_size: Size of the name labels (scoped variable and port name) combined
    :param handle_pos: Position of port which label is connected to
    :param port_side: Side on which the label should be drawn
    :param port_side_size: Size of port (to have a relative size)
    :param draw_connection_to_port: Whether there should be a line connecting the label to the port
    :return: Rotation Angle (to rotate names accordingly), X-Position of name labels start point, Y-Position of name
             labels start point
    """
    c = context.cairo
    c.set_line_width(port_side_size * .03)

    c.set_source_rgb(*color.to_floats())

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


def draw_port_label(context, port, transparency, fill, label_position, show_additional_value=False,
                    additional_value=None, only_extent_calculations=False):
    """Draws a normal label indicating the port name.

    :param context: Draw Context
    :param port: The PortView
    :param transparency: Transparency of the text
    :param fill: Whether the label should be filled or not
    :param label_position: Side on which the label should be drawn
    :param show_additional_value: Whether to show an additional value (for data ports)
    :param additional_value: The additional value to be shown
    :param only_extent_calculations: Calculate only the extends and do not actually draw
    """
    c = context
    cairo_context = c
    if isinstance(c, CairoBoundingBoxContext):
        cairo_context = c._cairo

    # Gtk TODO
    # c.set_antialias(Antialias.GOOD)

    text = port.name
    label_color = get_col_rgba(port.fill_color, transparency)
    text_color = port.text_color
    port_height = port.port_size[1]

    port_position = c.get_current_point()

    layout = PangoCairo.create_layout(cairo_context)
    layout.set_text(text, -1)

    font_name = constants.INTERFACE_FONT
    font = FontDescription(font_name + " " + str(FONT_SIZE))
    layout.set_font_description(font)

    ink_extents, logical_extents = layout.get_extents()
    extents = [extent / float(SCALE) for extent in [logical_extents.x, logical_extents.y,
                                                    logical_extents.width, logical_extents.height]]
    real_text_size = extents[2], extents[3]
    desired_height = port_height
    scale_factor = real_text_size[1] / desired_height

    # margin is the distance between the text and the border line
    margin = desired_height / 2.5
    arrow_height = desired_height
    # The real_text_size dimensions are rotated by 90 deg compared to the label, as the label is drawn upright
    text_size = desired_height, real_text_size[0] / scale_factor,
    text_size_with_margin = text_size[0] + 2 * margin, text_size[1] + 2 * margin + arrow_height
    port_distance = desired_height
    port_offset = desired_height / 2.

    if label_position is SnappedSide.RIGHT:
        label_angle = deg2rad(-90)
        text_angle = 0
    elif label_position is SnappedSide.BOTTOM:
        label_angle = 0
        text_angle = deg2rad(-90)
    elif label_position is SnappedSide.LEFT:
        label_angle = deg2rad(90)
        text_angle = 0
    else:  # label_position is SnappedSide.TOP:
        label_angle = deg2rad(180)
        text_angle = deg2rad(90)

    # Draw (filled) outline of label
    c.move_to(*port_position)
    c.save()
    c.rotate(label_angle)
    draw_label_path(c, text_size_with_margin[0], text_size_with_margin[1], arrow_height, port_distance, port_offset)
    c.restore()

    c.set_line_width(port_height * .03)
    c.set_source_rgba(*label_color)
    label_extents = c.stroke_extents()
    if label_extents[0] == 0:
        label_extents = c.fill_extents()

    if only_extent_calculations:
        c.new_path()
    else:
        if fill:
            c.fill_preserve()
        c.stroke()

        # Move to the upper left corner of the desired text position
        c.save()
        c.move_to(*port_position)
        c.rotate(label_angle)
        c.rel_move_to(0, port_distance + arrow_height + 2 * margin)
        c.scale(1. / scale_factor, 1. / scale_factor)
        c.rel_move_to(-real_text_size[1] / 2 - extents[1], real_text_size[0] - extents[0])
        c.restore()

        # Show text in correct orientation
        c.save()
        c.rotate(text_angle)
        c.scale(1. / scale_factor, 1. / scale_factor)
        # Correction for labels positioned right: as the text is mirrored, the anchor point must be moved
        if label_position is SnappedSide.RIGHT:
            c.rel_move_to(-real_text_size[0], -real_text_size[1])
        c.set_source_rgba(*get_col_rgba(text_color, transparency))
        PangoCairo.update_layout(cairo_context, layout)
        PangoCairo.show_layout(cairo_context, layout)
        c.restore()

    if show_additional_value:
        value_text = limit_value_string_length(additional_value)
        value_layout = PangoCairo.create_layout(cairo_context)
        value_layout.set_text(value_text, -1)
        value_layout.set_font_description(font)

        ink_extents, logical_extents = value_layout.get_extents()
        extents = [extent / float(SCALE) for extent in [logical_extents.x, logical_extents.y,
                                                        logical_extents.width, logical_extents.height]]
        value_text_size = extents[2], real_text_size[1]

        # Move to the upper left corner of the additional value box
        c.save()
        c.move_to(*port_position)
        c.rotate(label_angle)
        c.rel_move_to(-text_size_with_margin[0] / 2., text_size_with_margin[1] + port_distance)
        # Draw rectangular path
        c.rel_line_to(text_size_with_margin[0], 0)
        c.rel_line_to(0, value_text_size[0] / scale_factor + 2 * margin)
        c.rel_line_to(-text_size_with_margin[0], 0)
        c.close_path()
        c.restore()

        value_extents = c.stroke_extents()

        if only_extent_calculations:
            c.new_path()
        else:
            # Draw filled outline
            c.set_source_rgba(*get_col_rgba(gui_config.gtk_colors['DATA_VALUE_BACKGROUND']))
            c.fill_preserve()
            c.set_source_rgb(*gui_config.gtk_colors['BLACK'].to_floats())
            c.stroke()

            # Move to the upper left corner of the desired text position
            c.save()
            c.move_to(*port_position)
            c.rotate(label_angle)
            c.rel_move_to(0, margin + text_size_with_margin[1] + port_distance)
            c.scale(1. / scale_factor, 1. / scale_factor)
            c.rel_move_to(-real_text_size[1] / 2., value_text_size[0])
            c.restore()

            # Show text in correct orientation
            c.save()
            c.rotate(text_angle)
            c.scale(1. / scale_factor, 1. / scale_factor)
            # Correction for labels positioned right: as the text is mirrored, the anchor point must be moved
            if label_position is SnappedSide.RIGHT:
                c.rel_move_to(-value_text_size[0] - margin * scale_factor, -real_text_size[1])
            c.set_source_rgba(*get_col_rgba(gui_config.gtk_colors['SCOPED_VARIABLE_TEXT']))
            PangoCairo.update_layout(cairo_context, value_layout)
            PangoCairo.show_layout(cairo_context, value_layout)
            c.restore()

        label_extents = min(label_extents[0], value_extents[0]), min(label_extents[1], value_extents[1]), \
                        max(label_extents[2], value_extents[2]), max(label_extents[3], value_extents[3])

    return label_extents


def draw_label_path(context, width, height, arrow_height, distance_to_port, port_offset):
    """Draws the path for an upright label

    :param context: The Cairo context
    :param float width: Width of the label
    :param float height: Height of the label
    :param float distance_to_port: Distance to the port related to the label
    :param float port_offset: Distance from the port center to its border
    :param bool draw_connection_to_port: Whether to draw a line from the tip of the label to the port
    """
    c = context
    # The current point is the port position

    # Mover to outer border of state
    c.rel_move_to(0, port_offset)
    # Draw line to arrow tip of label
    c.rel_line_to(0, distance_to_port)

    # Line to upper left corner
    c.rel_line_to(-width / 2., arrow_height)
    # Line to lower left corner
    c.rel_line_to(0, height - arrow_height)
    # Line to lower right corner
    c.rel_line_to(width, 0)
    # Line to upper right corner
    c.rel_line_to(0, -(height - arrow_height))
    # Line to center top (tip of label)
    c.rel_line_to(-width / 2., -arrow_height)
    # Close path
    c.close_path()


def get_text_layout(cairo_context, text, size):
    c = cairo_context
    layout = PangoCairo.create_layout(c)
    layout.set_text(text, -1)

    font_name = constants.INTERFACE_FONT

    font = FontDescription(font_name + " " + str(size))
    layout.set_font_description(font)

    return layout
