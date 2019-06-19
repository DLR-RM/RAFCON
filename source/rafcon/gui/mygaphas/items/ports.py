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
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import object
from builtins import str
from weakref import ref
from gi.repository.Pango import SCALE, FontDescription
from gi.repository import PangoCairo
# from cairo import Antialias

from gaphas.state import observed
from gaphas.connector import Handle
from gaphas.painter import CairoBoundingBoxContext
import cairo

from rafcon.gui.utils import constants
from rafcon.utils.geometry import deg2rad

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.models.logical_port import IncomeModel, OutcomeModel
from rafcon.gui.models.data_port import DataPortModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.models.container_state import ContainerStateModel

from rafcon.gui.mygaphas.connector import RectanglePointPort
from rafcon.gui.mygaphas.utils import gap_draw_helper
from rafcon.gui.mygaphas.utils.enums import SnappedSide, Direction
from rafcon.gui.mygaphas.utils.cache.image_cache import ImageCache

from rafcon.utils import log
logger = log.get_logger(__name__)


class PortView(object):
    def __init__(self, in_port, name=None, parent=None, side=SnappedSide.RIGHT):
        self.handle = Handle(connectable=True)
        self.port = RectanglePointPort(self.handle.pos, self)
        self._is_in_port = in_port
        self._side = None
        self.direction = None
        self.side = side
        self._parent = ref(parent)
        self._view = None

        self.text_color = gui_config.gtk_colors['LABEL']
        self.fill_color = gui_config.gtk_colors['LABEL']

        self._incoming_handles = []
        self._outgoing_handles = []
        self._connected_connections = []
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

        self._name = name


        self.label_print_inside = True

        self._port_image_cache = ImageCache()
        self._label_image_cache = ImageCache()
        self._last_label_size = self.port_side_size, self.port_side_size
        self._last_label_relative_pos = 0, 0

    def __getattr__(self, name):
        """Return parental attributes for unknown attributes

        The PortView class is now Gaphas item, however it is often treated like that. Therefore, several expected
        attributes are missing. In these cases, the corresponding attribute of the parental StateView is returned.

        :param str name: Name of teh requested attribute
        :return: Parental value of the attribute
        """
        try:
            return getattr(self.parent, name)
        except Exception:
            # This workarounds are needed because parent is a weak reference and if the the state is already destroy
            # the name of its parent can not be accessed even if the view of the port still exists
            # TODO D-check if ports can be destroyed proper before the state view is collected by the garbage collector
            if name == "name":
                return self._name
            raise

    def handles(self):
        return [self.handle]

    @property
    def side(self):
        return self._side

    @side.setter
    @observed
    def side(self, side):
        self._side = side
        self.direction = None
        if self.side is SnappedSide.LEFT:
            self.direction = Direction.RIGHT if self._is_in_port else Direction.LEFT
        elif self.side is SnappedSide.TOP:
            self.direction = Direction.DOWN if self._is_in_port else Direction.UP
        elif self.side is SnappedSide.RIGHT:
            self.direction = Direction.LEFT if self._is_in_port else Direction.RIGHT
        elif self.side is SnappedSide.BOTTOM:
            self.direction = Direction.UP if self._is_in_port else Direction.DOWN

    @property
    def port_side_size(self):
        parent = self.parent
        if not parent:
            logger.warning("PortView without parent: {}".format(self))
            return 1
        return parent.border_width

    @property
    def name(self):
        return self._name

    @property
    def parent(self):
        return self._parent()

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

    @property
    def view(self):
        if not self._view:
            self._view = self.parent.canvas.get_first_view()
        return self._view

    def has_outgoing_connection(self):
        return len(self._outgoing_handles) > 0

    def has_incoming_connection(self):
        return len(self._incoming_handles) > 0

    def add_connected_handle(self, handle, connection_view, moving=False):
        from rafcon.gui.mygaphas.items.connection import ConnectionView
        assert isinstance(handle, Handle)
        assert isinstance(connection_view, ConnectionView)
        if not moving and handle is connection_view.from_handle() and handle not in self._outgoing_handles:
            self._outgoing_handles.append(handle)
            self._add_connection(connection_view)
        elif not moving and handle is connection_view.to_handle() and handle not in self._incoming_handles:
            self._incoming_handles.append(handle)
            self._add_connection(connection_view)

    def has_label(self):
        return False

    def is_selected(self):
        return self in self.parent.canvas.get_first_view().selected_items

    def _add_connection(self, connection_view):
        if connection_view not in self.connected_connections:
            self._connected_connections.append(ref(connection_view))

    def remove_connected_handle(self, handle):
        assert isinstance(handle, Handle)
        if handle in self._incoming_handles:
            self._incoming_handles.remove(handle)
            for conn in self.connected_connections:
                if conn.to_handle() is handle:
                    self._connected_connections.remove(ref(conn))
        elif handle in self._outgoing_handles:
            self._outgoing_handles.remove(handle)
            for conn in self.connected_connections:
                if conn.from_handle() is handle:
                    self._connected_connections.remove(ref(conn))

    def tmp_connect(self, handle, connection_view):
        if handle is connection_view.from_handle():
            self._tmp_outgoing_connected = True
        elif handle is connection_view.to_handle():
            self._tmp_incoming_connected = True

    def tmp_disconnect(self):
        self._tmp_incoming_connected = False
        self._tmp_outgoing_connected = False

    @property
    def connected(self):
        return self.connected_incoming or self.connected_outgoing

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

    @property
    def connected_connections(self):
        return [connection() for connection in self._connected_connections]

    def get_port_area(self, view):
        """Calculates the drawing area affected by the (hovered) port
        """
        state_v = self.parent
        center = self.handle.pos
        margin = self.port_side_size / 4.
        if self.side in [SnappedSide.LEFT, SnappedSide.RIGHT]:
            height, width = self.port_size
        else:
            width, height = self.port_size
        upper_left = center[0] - width / 2 - margin, center[1] - height / 2 - margin
        lower_right = center[0] + width / 2 + margin, center[1] + height / 2 + margin
        port_upper_left = view.get_matrix_i2v(state_v).transform_point(*upper_left)
        port_lower_right = view.get_matrix_i2v(state_v).transform_point(*lower_right)
        size = port_lower_right[0] - port_upper_left[0], port_lower_right[1] - port_upper_left[1]
        return port_upper_left[0], port_upper_left[1], size[0], size[1]

    def draw(self, context, state):
        raise NotImplementedError

    def draw_port(self, context, fill_color, transparency, value=None):
        c = context.cairo
        view = self.parent.canvas.get_first_view()
        side_length = self.port_side_size
        position = self.pos

        # Do not draw ports below a certain threshold size
        matrix_i2v = view.get_matrix_i2v(self.parent)
        view_length, _ = matrix_i2v.transform_distance(side_length, 0)
        if view_length < constants.MINIMUM_PORT_SIZE_FOR_DISPLAY and not context.draw_all:
            return
        # Do not draw port outside of the view
        center = (position.x.value, position.y.value)
        view_center = matrix_i2v.transform_point(*center)
        if view_center[0] + view_length / 2. < 0 or \
                view_center[0] - view_length / 2. > view.get_allocation().width or \
                view_center[1] + view_length / 2. < 0 or \
                view_center[1] - view_length / 2. > view.get_allocation().height:
            if not context.draw_all:
                return

        parent_state_m = self.parent.model
        is_library_state_with_content_shown = self.parent.show_content()

        parameters = {
            'selected': self.is_selected(),
            'direction': self.direction,
            'side_length': side_length,
            'fill_color': fill_color,
            'transparency': transparency,
            'incoming': self.connected_incoming,
            'outgoing': self.connected_outgoing,
            'is_library_state_with_content_shown': is_library_state_with_content_shown,
            'draw_all': context.draw_all
        }

        upper_left_corner = (position.x.value - side_length / 2., position.y.value - side_length / 2.)

        current_zoom = view.get_zoom_factor()
        from_cache, image, zoom = self._port_image_cache.get_cached_image(side_length, side_length,
                                                                          current_zoom, parameters)

        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print("from cache")
            self._port_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        elif not context.draw_all:
            # print("draw")
            c = self._port_image_cache.get_context_for_image(current_zoom)

            c.move_to(0, 0)

            if isinstance(parent_state_m, ContainerStateModel) or is_library_state_with_content_shown:
                self._draw_container_state_port(c, self.direction, fill_color, transparency)
            else:
                self._draw_simple_state_port(c, self.direction, fill_color, transparency)

            # Copy image surface to current cairo context
            self._port_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

        if self.name and self.has_label():
            self.draw_name(context, transparency, value)

        if self.is_selected() or self.handle is view.hovered_handle or context.draw_all:
            context.cairo.move_to(*self.pos)
            self._draw_hover_effect(context.cairo, self.direction, fill_color, transparency)

    def draw_name(self, context, transparency, value):
        c = context.cairo
        port_height = self.port_size[1]
        label_position = self.side if not self.label_print_inside else self.side.opposite()
        position = self.pos

        show_additional_value = False
        if global_runtime_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True) and value is not None:
            show_additional_value = True

        parameters = {
            'name': self.name,
            'port_height': port_height,
            'side': label_position,
            'transparency': transparency,
            'show_additional_value': show_additional_value,
            'draw_all': context.draw_all
        }

        # add value to parameters only when value is shown on label
        if show_additional_value:
            parameters['value'] = value

        upper_left_corner = (position[0] + self._last_label_relative_pos[0],
                             position[1] + self._last_label_relative_pos[1])
        current_zoom = self.parent.canvas.get_first_view().get_zoom_factor()
        from_cache, image, zoom = self._label_image_cache.get_cached_image(self._last_label_size[0],
                                                                           self._last_label_size[1],
                                                                           current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            # print("draw port name from cache")
            self._label_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # print("draw port name")

            # First we have to do a "dry run", in order to determine the size of the new label
            c.move_to(position.x.value, position.y.value)
            extents = gap_draw_helper.draw_port_label(c, self, transparency, False, label_position,
                                                      show_additional_value, value, only_extent_calculations=True)
            from rafcon.gui.mygaphas.utils.gap_helper import extend_extents
            extents = extend_extents(extents, factor=1.1)
            label_pos = extents[0], extents[1]
            relative_pos = label_pos[0] - position[0], label_pos[1] - position[1]
            label_size = extents[2] - extents[0], extents[3] - extents[1]

            # print(label_size[0], self.name, self.parent.model.state.name)
            # if label_size[0] < constants.MINIMUM_PORT_NAME_SIZE_FOR_DISPLAY and self.parent:
            #     return
            self._last_label_relative_pos = relative_pos
            self._last_label_size = label_size

            if not context.draw_all:
                # The size information is used to update the caching parameters and retrieve an image with the correct size
                self._label_image_cache.get_cached_image(label_size[0], label_size[1], current_zoom, parameters, clear=True)
                c = self._label_image_cache.get_context_for_image(current_zoom)
                c.move_to(-relative_pos[0], -relative_pos[1])

                gap_draw_helper.draw_port_label(c, self, transparency, False, label_position, show_additional_value, value)

                # Copy image surface to current cairo context
                upper_left_corner = (position[0] + relative_pos[0], position[1] + relative_pos[1])
                self._label_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

                   # draw_all means, the bounding box of the state is calculated
                   # As we are using drawing operation, not supported by Gaphas, we manually need to update the bounding box
            else:  # context.draw_all:
                from gaphas.geometry import Rectangle
                view = self.parent.canvas.get_first_view()
                abs_pos = view.get_matrix_i2v(self.parent).transform_point(*label_pos)
                abs_pos1 = view.get_matrix_i2v(self.parent).transform_point(extents[2], extents[3])
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
        if self.connected:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        else:
            c.set_source_rgb(*gui_config.gtk_colors['PORT_UNCONNECTED'].to_floats())
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        c.stroke()

    def _draw_container_state_port(self, context, direction, color, transparency):
        """Draw the port of a container state

        Connector for container states are split in an inner connector and an outer connector.

        :param context: Cairo context
        :param direction: The direction the port is pointing to
        :param color: Desired color of the port
        :param float transparency: The level of transparency
        """
        c = context

        width, height = self.port_size
        c.set_line_width(self.port_side_size / constants.BORDER_WIDTH_OUTLINE_WIDTH_FACTOR *
                         self._port_image_cache.multiplicator)

        # Save/restore context, as we move and rotate the connector to the desired pose
        cur_point = c.get_current_point()
        c.save()
        c.rel_move_to(self.port_side_size / 2., self.port_side_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_inner_connector(c, width, height)
        c.restore()

        if self.connected_incoming:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        else:
            c.set_source_rgb(*gui_config.gtk_colors['PORT_UNCONNECTED'].to_floats())
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        c.stroke()

        c.move_to(*cur_point)
        c.save()
        c.rel_move_to(self.port_side_size / 2., self.port_side_size / 2.)
        PortView._rotate_context(c, direction)
        PortView._draw_outer_connector(c, width, height)
        c.restore()

        if self.connected_outgoing:
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        else:
            c.set_source_rgb(*gui_config.gtk_colors['PORT_UNCONNECTED'].to_floats())
        c.fill_preserve()
        c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
        c.stroke()

    def _draw_hover_effect(self, context, direction, color, transparency):
        c = context

        width, height = self.port_size
        c.set_line_width(self.port_side_size * 0.03 * self._port_image_cache.multiplicator)
        margin = self.port_side_size / 4.

        # Save/restore context, as we move and rotate the connector to the desired pose
        c.save()
        PortView._rotate_context(c, direction)
        PortView._draw_rectangle(c, width + margin, height + margin)
        c.restore()

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(color, transparency))
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

        arrow_height = height / 2.0

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

        arrow_height = height / 2.5
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


class LogicPortView(PortView):
    """Base class for ports connecting transitions

    A logic port is either a income our an outcome.
    """

    def draw(self, context, state, highlight):
        raise NotImplementedError


class IncomeView(LogicPortView):
    def __init__(self, income_m, parent):
        super(IncomeView, self).__init__(in_port=True, parent=parent, side=SnappedSide.LEFT)

        assert isinstance(income_m, IncomeModel)
        self._income_m = ref(income_m)
        
        self.text_color = gui_config.gtk_colors['OUTCOME_PORT']
        self.fill_color = gui_config.gtk_colors['OUTCOME_PORT']

    def draw(self, context, state, highlight=False):
        self.draw_port(context, self.fill_color, state.transparency)

    @property
    def model(self):
        return self._income_m()


class OutcomeView(LogicPortView):
    def __init__(self, outcome_m, parent):
        super(OutcomeView, self).__init__(in_port=False, name=outcome_m.outcome.name, parent=parent)

        assert isinstance(outcome_m, OutcomeModel)
        self._outcome_m = ref(outcome_m)
        self.sort = outcome_m.outcome.outcome_id

    @property
    def model(self):
        return self._outcome_m()

    @property
    def outcome_id(self):
        return self.model.outcome.outcome_id

    @property
    def name(self):
        return self.model.outcome.name

    def has_label(self):
        if self.has_outgoing_connection():
            return False
        if not global_runtime_config.get_config_value("SHOW_ABORTED_PREEMPTED", False) and self.outcome_id in [-1, -2]:
            return False
        return True

    def draw(self, context, state, highlight=False):
        # Do not draw if the core element has already been destroyed
        if not self.model.core_element:
            return

        if highlight:
            fill_color = gui_config.gtk_colors['STATE_ACTIVE_BORDER']
        elif self.outcome_id == -2:
            fill_color = gui_config.gtk_colors['PREEMPTED']
        elif self.outcome_id == -1:
            fill_color = gui_config.gtk_colors['ABORTED']
        else:
            fill_color = gui_config.gtk_colors['OUTCOME_PORT']

        self.draw_port(context, fill_color, state.transparency)


class ScopedVariablePortView(PortView):
    def __init__(self, parent, scoped_variable_m):
        super(ScopedVariablePortView, self).__init__(False, parent=parent, side=SnappedSide.TOP)
        self.fill_color = gui_config.gtk_colors['DATA_PORT']
        self.text_color = gui_config.gtk_colors['SCOPED_VARIABLE_TEXT']

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

    @PortView.port_size.getter
    def port_size(self):
        if self.side in [SnappedSide.TOP, SnappedSide.BOTTOM]:
            return self._last_label_size
        return self._last_label_size[1], self._last_label_size[0]

    def draw(self, context, state):
        # Do not draw if the core element has already been destroyed
        if not self.model.core_element:
            return

        c = context.cairo
        view = self.parent.canvas.get_first_view()
        side_length = self.port_side_size

        parameters = {
            'name': self.name,
            'side': self.side,
            'side_length': side_length,
            'selected': self.is_selected(),
            'transparency': state.transparency,
            'draw_all': context.draw_all
        }
        current_zoom = view.get_zoom_factor()
        from_cache, image, zoom = self._port_image_cache.get_cached_image(self._last_label_size[0],
                                                                          self._last_label_size[1],
                                                                          current_zoom, parameters)
        # The parameters for drawing haven't changed, thus we can just copy the content from the last rendering result
        if from_cache:
            center_pos = self._get_port_center_position(self._last_label_span)
            upper_left_corner = center_pos[0] - self._last_label_size[0] / 2., \
                                center_pos[1] - self._last_label_size[1] / 2.
            self._port_image_cache.copy_image_to_context(c, upper_left_corner)

        # Parameters have changed or nothing in cache => redraw
        else:
            # First we have to do a "dry run", in order to determine the size of the port
            c.move_to(*self.pos)
            name_size = self.draw_name(c, state.transparency, only_calculate_size=True)
            extents = self._draw_rectangle_path(c, name_size[0], side_length, only_get_extents=True)

            port_size = extents[2] - extents[0], extents[3] - extents[1]
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
            c.set_source_rgba(*gap_draw_helper.get_col_rgba(self.fill_color, state.transparency))
            c.fill_preserve()
            c.stroke()

            # Second, write the text in the rectangle (scoped variable name)
            # Set the current point to be in the center of the rectangle
            if not context.draw_all:
                c.move_to(port_size[0] / 2., port_size[1] / 2.)
                self.draw_name(c, state.transparency)

            # Copy image surface to current cairo context
            center_pos = self._get_port_center_position(name_size[0])
            upper_left_corner = center_pos[0] - port_size[0] / 2., center_pos[1] - port_size[1] / 2.
            self._port_image_cache.copy_image_to_context(context.cairo, upper_left_corner, zoom=current_zoom)

        if self.is_selected() or self.handle is view.hovered_handle or context.draw_all:
            context.cairo.move_to(*self._get_port_center_position(self._last_label_span))
            self._draw_hover_effect(context.cairo, self.direction, self.fill_color, state.transparency)

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
        cairo_context = c
        if isinstance(c, CairoBoundingBoxContext):
            cairo_context = c._cairo
        # c.set_antialias(Antialias.GOOD)

        side_length = self.port_side_size

        layout = PangoCairo.create_layout(cairo_context)
        font_name = constants.INTERFACE_FONT
        font_size = gap_draw_helper.FONT_SIZE
        font = FontDescription(font_name + " " + str(font_size))
        layout.set_font_description(font)
        layout.set_text(self.name, -1)

        ink_extents, logical_extents = layout.get_extents()
        extents = [extent / float(SCALE) for extent in [logical_extents.x, logical_extents.y,
                                                        logical_extents.width, logical_extents.height]]
        real_name_size = extents[2], extents[3]
        desired_height = side_length * 0.75
        scale_factor = real_name_size[1] / desired_height

        # Determine the size of the text, increase the width to have more margin left and right of the text
        margin = side_length / 4.
        name_size = real_name_size[0] / scale_factor, desired_height
        name_size_with_margin = name_size[0] + margin * 2, name_size[1] + margin * 2

        # Only the size is required, stop here
        if only_calculate_size:
            return name_size_with_margin

        # Current position is the center of the port rectangle
        c.save()
        if self.side is SnappedSide.RIGHT or self.side is SnappedSide.LEFT:
            c.rotate(deg2rad(-90))
        c.rel_move_to(-name_size[0] / 2, -name_size[1] / 2)
        c.scale(1. / scale_factor, 1. / scale_factor)
        c.rel_move_to(-extents[0], -extents[1])

        c.set_source_rgba(*gap_draw_helper.get_col_rgba(self.text_color, transparency))
        PangoCairo.update_layout(cairo_context, layout)
        PangoCairo.show_layout(cairo_context, layout)
        c.restore()

        return name_size_with_margin

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
    def __init__(self, in_port, parent, port_m, side):
        assert isinstance(port_m, DataPortModel)
        super(DataPortView, self).__init__(in_port=in_port, name=port_m.data_port.name, parent=parent, side=side)

        self._port_m = ref(port_m)
        self.sort = port_m.data_port.data_port_id

        self._value = None

        self.text_color = gui_config.gtk_colors['DATA_PORT']
        self.fill_color = gui_config.gtk_colors['DATA_PORT']

    @property
    def model(self):
        return self._port_m()

    @property
    def port_id(self):
        return self.model.data_port.data_port_id

    @property
    def name(self):
        return self.model.data_port.name

    def has_label(self):
        return self.parent.selected or self.parent.show_data_port_label

    def draw(self, context, state):
        # Do not draw if the core element has already been destroyed
        if not self.model.core_element:
            return

        self.draw_port(context, self.fill_color, state.transparency, self._value)


class InputPortView(DataPortView):
    def __init__(self, parent, port_m):
        super(InputPortView, self).__init__(True, parent, port_m, SnappedSide.LEFT)
        self.label_print_inside = False

    def draw(self, context, state):
        input_data = self.parent.model.state.input_data
        if len(self.parent.model.state.input_data) > 0 and self.name in input_data:
            self._value = input_data[self.name]
        super(InputPortView, self).draw(context, state)


class OutputPortView(DataPortView):
    def __init__(self, parent, port_m):
        super(OutputPortView, self).__init__(False, parent, port_m, SnappedSide.RIGHT)
        self.label_print_inside = True

    def draw(self, context, state):
        output_data = self.parent.model.state.output_data
        if len(self.parent.model.state.input_data) > 0 and self.name in output_data:
            self._value = output_data[self.name]
        super(OutputPortView, self).draw(context, state)
