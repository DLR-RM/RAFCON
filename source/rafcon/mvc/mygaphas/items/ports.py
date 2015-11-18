from weakref import ref
from pango import SCALE, FontDescription

from gaphas.state import observed
from gaphas.connector import Handle
import cairo
from gtk.gdk import Color

from rafcon.utils import constants
from rafcon.utils.geometry import deg2rad

from rafcon.statemachine.states.container_state import ContainerState

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.models.outcome import OutcomeModel
from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel

from rafcon.mvc.mygaphas.connector import RectanglePointPort
from rafcon.mvc.mygaphas.utils import gap_draw_helper
from rafcon.mvc.mygaphas.utils.enums import SnappedSide, Direction
from rafcon.mvc.mygaphas.utils.cache.image_cache import ImageCache


class PortView(object):
    def __init__(self, in_port, port_side_size, name=None, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = RectanglePointPort(self.handle.pos, port_side_size, port_side_size)
        self._side = None
        self.side = side
        self._parent = parent

        self._draw_connection_to_port = False

        self.text_color = constants.LABEL_COLOR
        self.fill_color = constants.LABEL_COLOR

        self._incoming_handles = []
        self._outgoing_handles = []
        self._connected_connections = []
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

        self._name = name

        self._is_in_port = in_port

        self._port_side_size = port_side_size
        self.update_port_side_size()

        self.label_print_inside = True

        self._port_image_cache = ImageCache()
        self._label_image_cache = ImageCache()
        self._last_label_size = 0, 0
        self._last_label_relative_pos = 0, 0

    @property
    def side(self):
        return self._side

    @side.setter
    @observed
    def side(self, side):
        assert isinstance(side, SnappedSide)
        self._side = side

    @property
    def port_side_size(self):
        return self._port_side_size

    @port_side_size.setter
    def port_side_size(self, port_side_size):
        self._port_side_size = port_side_size
        self.port.width = port_side_size
        self.port.height = port_side_size

    @property
    def name(self):
        return self._name

    @property
    def parent(self):
        return self._parent

    @property
    def pos(self):
        return self.handle.pos

    @property
    def handle_pos(self):
        return self.handle.pos

    @property
    def port_pos(self):
        return self.port.point

    @property
    def port_size(self):
        return self.port_side_size / 1.5, self.port_side_size

    def has_outgoing_connection(self):
        return len(self._outgoing_handles) > 0

    def has_incoming_connection(self):
        return len(self._incoming_handles) > 0

    def add_connected_handle(self, handle, connection_view, moving=False):
        from rafcon.mvc.mygaphas.items.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if not moving and handle is connection_view.from_handle() and handle not in self._outgoing_handles:
            self._outgoing_handles.append(handle)
            self._add_connection(connection_view)
        elif not moving and handle is connection_view.to_handle() and handle not in self._incoming_handles:
            self._incoming_handles.append(handle)
            self._add_connection(connection_view)

    def _add_connection(self, connection_view):
        if connection_view not in self._connected_connections:
            self._connected_connections.append(connection_view)

    def remove_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle in self._incoming_handles:
            self._incoming_handles.remove(handle)
            for conn in self._connected_connections:
                if conn.to_handle() is handle:
                    self._connected_connections.remove(conn)
        elif handle in self._outgoing_handles:
            self._outgoing_handles.remove(handle)
            for conn in self._connected_connections:
                if conn.from_handle() is handle:
                    self._connected_connections.remove(conn)

    def tmp_connect(self, handle, connection_view):
        if handle is connection_view.from_handle():
            self._tmp_outgoing_connected = True
        elif handle is connection_view.to_handle():
            self._tmp_incoming_connected = True

    def tmp_disconnect(self):
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

    @property
    def connected_outgoing(self):
        if len(self._outgoing_handles) == 0:
            return self._tmp_outgoing_connected
        return True

    @property
    def connected_incoming(self):
        if len(self._incoming_handles) == 0:
            return self._tmp_incoming_connected
        return True

    def is_connected_to_scoped_variable(self):
        from rafcon.mvc.mygaphas.items.connection import ScopedVariableDataFlowView
        for conn in self._connected_connections:
            if isinstance(conn, ScopedVariableDataFlowView):
                return True
        return False

    def get_port_area(self, view):
        """Calculates the drawing area affected by the (hovered) port
        """
        state_v = self.parent
        center = self.handle.pos
        margin = self.port_side_size / 4.
        upper_left = center[0] - self.port_size[0] - margin, center[1] - self.port_size[1] - margin
        lower_right = center[0] + self.port_size[0] + margin, center[1] + self.port_size[1] + margin
        port_upper_left = view.get_matrix_i2v(state_v).transform_point(*upper_left)
        port_lower_right = view.get_matrix_i2v(state_v).transform_point(*lower_right)
        size = port_lower_right[0] - port_upper_left[0], port_lower_right[1] - port_upper_left[1]
        return port_upper_left[0], port_upper_left[1], size[0], size[1]

    def draw(self, context, state):
        raise NotImplementedError

    def draw_port(self, context, fill_color, transparent, draw_label=True, value=None):
        c = context.cairo
        view = self._parent.canvas.get_first_view()
        self.update_port_side_size()
        side_length = self.port_side_size

        direction = None
        if self.side is SnappedSide.LEFT:
            direction = Direction.RIGHT if self._is_in_port else Direction.LEFT
        elif self.side is SnappedSide.TOP:
            direction = Direction.DOWN if self._is_in_port else Direction.UP
        elif self.side is SnappedSide.RIGHT:
            direction = Direction.LEFT if self._is_in_port else Direction.RIGHT
        elif self.side is SnappedSide.BOTTOM:
            direction = Direction.UP if self._is_in_port else Direction.DOWN

        parameters = {
            'direction': direction,
            'side_length': side_length,
            'fill_color': fill_color,
            'transparency': transparent,
            'incoming': self.connected_incoming,
            'outgoing': self.connected_outgoing,
        }

        upper_left_corner = (self.pos.x.value - side_length / 2., self.pos.y.value - side_length / 2.)
        current_zoom = view.get_zoom_factor()
        from_cache, image, zoom = self._port_image_cache.get_cached_image(side_length, side_length,
                                                                          current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "from cache"
            self._port_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw"
            c = self._port_image_cache.get_context_for_image(current_zoom)

            c.move_to(0, 0)
            if isinstance(self._parent.model.state, ContainerState):
                self._draw_container_state_port(c, direction, fill_color, transparent)
            else:
                self._draw_simple_state_port(c, direction, fill_color, transparent)

            # Copy image surface to current cairo context
            self._port_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

        if self.name and draw_label:  # not self.has_outgoing_connection() and draw_label:
            self.draw_name(context, transparent, value)

        if self.handle is view.hovered_handle or context.draw_all:
            context.cairo.move_to(*self.pos)
            self._draw_hover_effect(context.cairo, direction, fill_color, transparent)

    def draw_name(self, context, transparency, value):
        if self.is_connected_to_scoped_variable():
            return

        c = context.cairo
        side_length = self.port_side_size
        label_position = self.side if not self.label_print_inside else self.side.opposite()
        fill_color = gap_draw_helper.get_col_rgba(Color(self.fill_color), transparency)

        show_additional_value = False
        if global_gui_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False) and value is not None:
            show_additional_value = True

        parameters = {
            'name': self.name,
            'side_length': side_length,
            'side': label_position,
            'fill_color': fill_color,
            'show_additional_value': show_additional_value
        }

        # add value to parameters only when value is shown on label
        if show_additional_value:
            parameters['value'] = value

        upper_left_corner = (self.pos[0] + self._last_label_relative_pos[0],
                             self.pos[1] + self._last_label_relative_pos[1])
        current_zoom = self._parent.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._label_image_cache.get_cached_image(self._last_label_size[0],
                                                                           self._last_label_size[1],
                                                                           current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache and not context.draw_all:
            # print "draw port name from cache"
            self._label_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw port name"

            # First we have to do a "dry run", in order to determine the size of the new label
            c.move_to(self.pos.x.value, self.pos.y.value)
            extents = gap_draw_helper.draw_port_label(c, self.name, fill_color, self.text_color, transparency,
                                                      False, label_position, side_length, self._draw_connection_to_port,
                                                      show_additional_value, value, only_extent_calculations=True)
            from rafcon.mvc.mygaphas.utils.gap_helper import extend_extents
            extents = extend_extents(extents, factor=1.02)
            label_pos = extents[0], extents[1]
            relative_pos = label_pos[0] - self.pos[0], label_pos[1] - self.pos[1]
            label_size = extents[2] - extents[0], extents[3] - extents[1]
            self._last_label_relative_pos = relative_pos
            self._last_label_size = label_size

            # The size information is used to update the caching parameters and retrieve an image with the correct size
            self._label_image_cache.get_cached_image(label_size[0], label_size[1], current_zoom, parameters, clear=True)
            c = self._label_image_cache.get_context_for_image(current_zoom)
            c.move_to(-relative_pos[0], -relative_pos[1])

            gap_draw_helper.draw_port_label(c, self.name, fill_color, self.text_color, transparency,
                                            False, label_position, side_length, self._draw_connection_to_port,
                                            show_additional_value, value)

            # Copy image surface to current cairo context
            upper_left_corner = (self.pos[0] + relative_pos[0], self.pos[1] + relative_pos[1])
            self._label_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

            # draw_all means, the bounding box of the state is calculated
            # As we are using drawing operation, not supported by Gaphas, we manually need to update the bounding box
            if context.draw_all:
                from gaphas.geometry import Rectangle
                view = self._parent.canvas.get_first_view()
                abs_pos = view.get_matrix_i2v(self._parent).transform_point(*label_pos)
                abs_pos1 = view.get_matrix_i2v(self._parent).transform_point(extents[2], extents[3])
                bounds = Rectangle(abs_pos[0], abs_pos[1], x1=abs_pos1[0], y1=abs_pos1[1])
                context.cairo._update_bounds(bounds)

    def _draw_simple_state_port(self, context, direction, color, transparency):
        """Draw the port of a simple state (ExecutionState, LibraryState)

        Connector for execution states can only be connected to the outside. Thus the connector fills the whole
        border of the state.

        :param context: Cairo context
        :param direction: The direction the port is pointing to
        :param color: Desired color of the port
        :param transparency: The level of transparency
        """
        c = context

        width, height = self.port_size
        c.set_line_width(self.port_side_size * 0.03 * self._port_image_cache.multiplicator)

        # Save/restore context, as we move and rotate the connector to the desired pose
        c.save()
        c.rel_move_to(self.port_side_size / 2., self.port_side_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_single_connector(c, width, height)
        c.restore()

        # Colorize the generated connector path
        if self.connected_incoming or self.connected_outgoing:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        else:
            c.set_source_color(Color(constants.BLACK_COLOR))
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        c.stroke()

    def _draw_container_state_port(self, context, direction, color, transparency):
        """Draw the port of a container state

        Connector for container states are split in an inner connector and an outer connector.

        :param context: Cairo context
        :param direction: The direction the port is pointing to
        :param color: Desired color of the port
        :param transparency: The level of transparency
        """
        c = context

        width, height = self.port_size
        c.set_line_width(self.port_side_size * 0.03 * self._port_image_cache.multiplicator)

        # Save/restore context, as we move and rotate the connector to the desired pose
        cur_point = c.get_current_point()
        c.save()
        c.rel_move_to(self.port_side_size / 2., self.port_side_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_inner_connector(c, width, height)
        c.restore()

        if self.connected_incoming:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        else:
            c.set_source_color(Color(constants.BLACK_COLOR))
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        c.stroke()

        c.move_to(*cur_point)
        c.save()
        c.rel_move_to(self.port_side_size / 2., self.port_side_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_outer_connector(c, width, height)
        c.restore()

        if self.connected_outgoing:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        else:
            c.set_source_color(Color(constants.BLACK_COLOR))
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        c.stroke()

    def _draw_hover_effect(self, context, direction, color, transparency):
        c = context

        width, height = self.port_size
        c.set_line_width(self.port_side_size * 0.03 * self._port_image_cache.multiplicator)
        margin = self.port_side_size / 4.

        # Save/restore context, as we move and rotate the connector to the desired pose
        c.save()
        # c.rel_move_to(port_size / 2., port_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_rectangle(c, width + margin, height + margin)
        c.restore()

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(color), transparency))
        c.stroke()

    @staticmethod
    def _draw_single_connector(context, width, height):
        """Draw the connector for execution states

        Connector for execution states can only be connected to the outside. Thus the connector fills the whole
        border of the state.

        :param context: Cairo context
        :param float port_size: The side length of the port
        """
        c = context
        # Current pos is center
        # Arrow is drawn upright

        arrow_height = height / 6.

        # First move to bottom left corner
        c.rel_move_to(-width / 2., height / 2.)
        # Draw line to bottom right corner
        c.rel_line_to(width, 0)
        # Draw line to upper right corner
        c.rel_line_to(0, -(height - arrow_height))
        # Draw line to center top (arrow)
        c.rel_line_to(-width / 2., -arrow_height)
        # Draw line to upper left corner
        c.rel_line_to(-width / 2., arrow_height)
        # Draw line back to the origin (lower left corner)
        c.close_path()

    @staticmethod
    def _draw_inner_connector(context, width, height):
        """Draw the connector for container states

        Connector for container states can be connected from the inside and the outside. Thus the connector is split
        in two parts: A rectangle on the inside and an arrow on the outside. This methods draws the inner rectangle.

        :param context: Cairo context
        :param float port_size: The side length of the port
        """
        c = context
        # Current pos is center
        # Arrow is drawn upright

        gap = height / 6.
        connector_height = (height - gap) / 2.

        # First move to bottom left corner
        c.rel_move_to(-width / 2., height / 2.)

        # Draw inner connector (rectangle)
        c.rel_line_to(width, 0)
        c.rel_line_to(0, -connector_height)
        c.rel_line_to(-width, 0)
        c.close_path()

    @staticmethod
    def _draw_outer_connector(context, width, height):
        """Draw the outer connector for container states

        Connector for container states can be connected from the inside and the outside. Thus the connector is split
        in two parts: A rectangle on the inside and an arrow on the outside. This method draws the outer arrow.

        :param context: Cairo context
        :param float port_size: The side length of the port
        """
        c = context
        # Current pos is center
        # Arrow is drawn upright

        arrow_height = height / 6.
        gap = height / 6.
        connector_height = (height - gap) / 2.

        # Move to bottom left corner of outer connector
        c.rel_move_to(-width / 2., -gap / 2.)

        # Draw line to bottom right corner
        c.rel_line_to(width, 0)
        # Draw line to upper right corner
        c.rel_line_to(0, -(connector_height - arrow_height))
        # Draw line to center top (arrow)
        c.rel_line_to(-width / 2., -arrow_height)
        # Draw line to upper left corner
        c.rel_line_to(-width / 2., arrow_height)
        # Draw line back to the origin (lower left corner)
        c.close_path()

    @staticmethod
    def _draw_rectangle(context, width, height):
        """Draw a rectangle

        Assertion: The current point is the center point of the rectangle

        :param context: Cairo context
        :param width: Width of the rectangle
        :param height: Height of the rectangle
        """
        c = context
        # First move to upper left corner
        c.rel_move_to(-width / 2., -height / 2.)
        # Draw closed rectangle
        c.rel_line_to(width, 0)
        c.rel_line_to(0, height)
        c.rel_line_to(-width, 0)
        c.close_path()

    @staticmethod
    def _rotate_context(context, direction):
        """Moves the current position to 'position' and rotates the context according to 'direction'

        :param context: Cairo context
        :param direction: Direction enum
        """
        if direction is Direction.UP:
            pass
        elif direction is Direction.RIGHT:
            context.rotate(deg2rad(90))
        elif direction is Direction.DOWN:
            context.rotate(deg2rad(180))
        elif direction is Direction.LEFT:
            context.rotate(deg2rad(-90))

    def update_port_side_size(self):
        return
        # if self._parent:
        #     self._port_side_size = min(self._parent.width, self._parent.height) / 20.
        # else:
        #     self._port_side_size = 5.


class LogicPortView(PortView):
    """Base class for ports connecting transitions

    A logic port is either a income our an outcome.
    """

    def draw(self, context, state):
        raise NotImplementedError


class IncomeView(LogicPortView):
    def __init__(self, parent, port_side_size):
        super(IncomeView, self).__init__(in_port=True, port_side_size=port_side_size, parent=parent,
                                         side=SnappedSide.LEFT)

    def draw(self, context, state):
        self.draw_port(context, constants.LABEL_COLOR, state.transparent)


class OutcomeView(LogicPortView):
    def __init__(self, outcome_m, parent, port_side_size):
        super(OutcomeView, self).__init__(in_port=False, port_side_size=port_side_size, name=outcome_m.outcome.name,
                                          parent=parent)

        assert isinstance(outcome_m, OutcomeModel)
        self._outcome_m = ref(outcome_m)
        self.sort = outcome_m.outcome.outcome_id

    @property
    def outcome_m(self):
        return self._outcome_m()

    @property
    def outcome_id(self):
        return self.outcome_m.outcome.outcome_id

    @property
    def name(self):
        return self.outcome_m.outcome.name

    def draw(self, context, state):
        if self.outcome_id == -2:
            fill_color = constants.PREEMPTED_COLOR
        elif self.outcome_id == -1:
            fill_color = constants.ABORTED_COLOR
        else:
            fill_color = constants.LABEL_COLOR

        draw_label = True
        if self.has_outgoing_connection():
            draw_label = False
        if not global_gui_config.get_config_value("SHOW_ABORTED_PREEMPTED", False) and self.outcome_id in [-1, -2]:
            draw_label = False

        self.draw_port(context, fill_color, state.transparent, draw_label=draw_label)


class ScopedVariablePortView(PortView):
    def __init__(self, parent, port_side_size, scoped_variable_m):
        super(ScopedVariablePortView, self).__init__(False, port_side_size, parent=parent, side=SnappedSide.TOP)

        assert isinstance(scoped_variable_m, ScopedVariableModel)
        self._scoped_variable_m = ref(scoped_variable_m)
        self._last_label_span = 0

    @property
    def model(self):
        return self._scoped_variable_m()

    @property
    def port_id(self):
        return self.model.scoped_variable.data_port_id

    @property
    def name(self):
        return self.model.scoped_variable.name

    @PortView.port_side_size.setter
    def port_side_size(self, port_side_size):
        # Methods needs to be overwritten, to prevent the change of the port size here, as the port size is
        # calculated in the draw method, depending on the length of the port name
        self._port_side_size = port_side_size

    def draw(self, context, state):
        c = context.cairo
        self.update_port_side_size()
        side_length = self.port_side_size

        parameters = {
            'name': self.name,
            'side': self.side,
            'side_length': side_length,
            'transparency': state.transparent
        }
        current_zoom = self._parent.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._port_image_cache.get_cached_image(self._last_label_size[0],
                                                                          self._last_label_size[1],
                                                                          current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print "from cache"

            center_pos = self._get_port_center_position(self._last_label_span)
            upper_left_corner = center_pos[0] - self._last_label_size[0] / 2., \
                                center_pos[1] - self._last_label_size[1] / 2.
            self._port_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print "draw"

            # First we have to do a "dry run", in order to determine the size of the port
            c.move_to(*self.pos)
            name_size = self.draw_name(c, state.transparent, only_calculate_size=True)
            extents = self._draw_rectangle_path(c, name_size[0], side_length, only_get_extents=True)

            port_size = extents[2] - extents[0], extents[3] - extents[1]
            self.port.width = port_size[0]
            self.port.height = port_size[1]
            self._last_label_size = port_size
            self._last_label_span = name_size[0]

            # The size information is used to update the caching parameters and retrieve a new context with an image
            # surface of the correct size
            self._port_image_cache.get_cached_image(port_size[0], port_size[1], current_zoom, parameters, clear=True)
            c = self._port_image_cache.get_context_for_image(current_zoom)

            # First, draw the filled rectangle
            # Set the current point to be in the center of the rectangle
            c.move_to(port_size[0] / 2., port_size[1] / 2.)
            self._draw_rectangle_path(c, name_size[0], side_length)
            c.set_line_width(self.port_side_size / 50. * self._port_image_cache.multiplicator)
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(constants.DATA_PORT_COLOR), state.transparent))
            c.fill_preserve()
            c.stroke()

            # Second, write the text in the rectangle (scoped variable name)
            # Set the current point to be in the center of the rectangle
            c.move_to(port_size[0] / 2., port_size[1] / 2.)
            self.draw_name(c, state.transparent)

            # Copy image surface to current cairo context
            center_pos = self._get_port_center_position(name_size[0])
            upper_left_corner = center_pos[0] - port_size[0] / 2., center_pos[1] - port_size[1] / 2.
            self._port_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

    def draw_name(self, context, transparency, only_calculate_size=False):
        """Draws the name of the port

        Offers the option to only calculate the size of the name.

        :param context: The context to draw on
        :param transparency: The transparency of the text
        :param only_calculate_size: Whether to only calculate the size
        :return: Size of the name
        :rtype: float, float
        """
        c = context
        c.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

        side_length = self.port_side_size

        layout = c.create_layout()
        font_name = constants.FONT_NAMES[0]
        font_size = side_length * .6
        font = FontDescription(font_name + " " + str(font_size))
        layout.set_font_description(font)
        layout.set_text(self.name)

        # Determine the size of the text, increase the width to have more margin left and right of the text
        real_name_size = layout.get_size()[0] / float(SCALE), layout.get_size()[1] / float(SCALE)
        name_size = real_name_size[0] + side_length / 2., side_length

        # Only the size is required, stop here
        if only_calculate_size:
            return name_size

        # Current position is the center of the port rectangle
        c.save()
        if self.side is SnappedSide.RIGHT or self.side is SnappedSide.LEFT:
            c.rotate(deg2rad(-90))
        c.rel_move_to(-real_name_size[0] / 2., -real_name_size[1] / 2.)

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(Color(constants.SCOPED_VARIABLE_TEXT_COLOR), transparency))
        c.update_layout(layout)
        c.show_layout(layout)
        c.restore()

        return name_size

    def _draw_rectangle_path(self, context, width, height, only_get_extents=False):
        """Draws the rectangle path for the port

        The rectangle is correctly rotated. Height therefore refers to the border thickness and width to the length
        of the port.

        :param context: The context to draw on
        :param float width: The width of the rectangle
        :param float height: The height of the rectangle
        """
        c = context

        # Current position is the center of the rectangle
        c.save()
        if self.side is SnappedSide.LEFT or self.side is SnappedSide.RIGHT:
            c.rotate(deg2rad(90))
        c.rel_move_to(-width / 2., - height / 2.)
        c.rel_line_to(width, 0)
        c.rel_line_to(0, height)
        c.rel_line_to(-width, 0)
        c.close_path()
        c.restore()

        if only_get_extents:
            extents = c.path_extents()
            c.new_path()
            return extents

    def _get_port_center_position(self, width):
        """Calculates the center position of the port rectangle

        The port itself can be positioned in the corner, the center of the port rectangle however is restricted by
        the width of the rectangle. This method therefore calculates the center, depending on the position of the
        port and the width of the rectangle.
        :param float width: The width of the rectangle
        :return: The center position of the rectangle
        :rtype: float, float
        """
        x, y = self.pos.x.value, self.pos.y.value
        if self.side is SnappedSide.TOP or self.side is SnappedSide.BOTTOM:
            if x - width / 2. < 0:
                x = width / 2
            elif x + width / 2. > self.parent.width:
                x = self.parent.width - width / 2.
        else:
            if y - width / 2. < 0:
                y = width / 2
            elif y + width / 2. > self.parent.height:
                y = self.parent.height - width / 2.
        return x, y


class DataPortView(PortView):
    def __init__(self, in_port, parent, port_m, side, port_side_size):
        assert isinstance(port_m, DataPortModel)
        super(DataPortView, self).__init__(in_port=in_port, port_side_size=port_side_size, name=port_m.data_port.name,
                                           parent=parent, side=side)

        self._port_m = ref(port_m)
        self.sort = port_m.data_port.data_port_id

        self._value = None

        self.text_color = constants.DATA_PORT_COLOR
        self.fill_color = constants.DATA_PORT_COLOR

    @property
    def port_m(self):
        return self._port_m()

    @property
    def port_id(self):
        return self.port_m.data_port.data_port_id

    @property
    def name(self):
        return self.port_m.data_port.name

    def draw(self, context, state):
        draw_label = state.selected or state.show_data_port_label or context.draw_all
        self.draw_port(context, constants.DATA_PORT_COLOR, state.transparent, draw_label, self._value)


class InputPortView(DataPortView):
    def __init__(self, parent, port_m, port_side_size):
        super(InputPortView, self).__init__(True, parent, port_m, SnappedSide.LEFT, port_side_size)
        self.label_print_inside = False

    def draw(self, context, state):
        input_data = self.parent.model.state.input_data
        if len(self.parent.model.state.input_data) > 0 and self.name in input_data.iterkeys():
            self._value = input_data[self.name]
        super(InputPortView, self).draw(context, state)


class OutputPortView(DataPortView):
    def __init__(self, parent, port_m, port_side_size):
        super(OutputPortView, self).__init__(False, parent, port_m, SnappedSide.RIGHT, port_side_size)
        self.label_print_inside = True

    def draw(self, context, state):
        output_data = self.parent.model.state.output_data
        if len(self.parent.model.state.input_data) > 0 and self.name in output_data.iterkeys():
            self._value = output_data[self.name]
        super(OutputPortView, self).draw(context, state)
