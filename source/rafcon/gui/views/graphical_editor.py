# Copyright (C) 2014-2018 DLR
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
from builtins import range
from math import sin, cos, pi, atan2
from enum import Enum

from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
# careful import
try:
    import gtk.gtkgl
    import gtk.gdkgl
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *

    # Activate the following line in the production code th increase speed
    # http://pyopengl.sourceforge.net/documentation/opengl_diffs.html:
    # OpenGL.ERROR_CHECKING = False
    OpenGL.FULL_LOGGING = True
    # Also in productive code, set the previous statement to false and activate the next one
    # OpenGL.ERROR_LOGGING = False
    GL_ENABLED = True
except (ImportError, RuntimeError):
    GL_ENABLED = False

from gtkmvc3.view import View

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.utils.geometry import dist

from rafcon.utils import log
logger = log.get_logger(__name__)


class Direction(Enum):
    top = 1
    right = 2
    bottom = 3
    left = 4


class Color(object):

    _r = 0
    _g = 0
    _b = 0
    _a = 0

    def __init__(self, r, g, b, a=1):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    @staticmethod
    def from_hex_string(hex_string):
        hex_string = hex_string[1:]
        if len(hex_string) > 3:
            r = int(hex_string[0:2], 16)
            g = int(hex_string[2:4], 16)
            b = int(hex_string[4:6], 16)
        else:
            r = int(hex_string[0:1] + hex_string[0:1], 16)
            g = int(hex_string[1:2] + hex_string[1:2], 16)
            b = int(hex_string[2:3] + hex_string[2:3], 16)
        return Color.from_dec(r, g, b)

    @staticmethod
    def from_hex(rgb, a=0xFF):
        r = (rgb >> 16) & 0xFF
        g = (rgb >> 8) & 0xFF
        b = rgb & 0xFF
        return Color.from_dec(r, g, b, a)

    @staticmethod
    def from_dec(r, g, b, a=255):
        if a < 1.:
            a *= 255
        return Color(r / 255., g / 255., b / 255., a / 255.)

    @property
    def r(self):
        return self._r

    @property
    def g(self):
        return self._g

    @property
    def b(self):
        return self._b

    @property
    def a(self):
        return self._a

    @r.setter
    def r(self, r):
        self._r = self._check_range(r)

    @g.setter
    def g(self, g):
        self._g = self._check_range(g)

    @b.setter
    def b(self, b):
        self._b = self._check_range(b)

    @a.setter
    def a(self, a):
        self._a = self._check_range(a)

    @staticmethod
    def _check_range(c):
        if c < 0:
            return 0
        if c > 1:
            return 1

    def set(self):
        """Set the color as current OpenGL color
        """
        glColor4f(self.r, self.g, self.b, self.a)


class GraphicalEditorView(View):

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        View.__init__(self)

        # Configure OpenGL frame buffer.
        # Try to get a double-buffered frame buffer configuration,
        # if not successful then exit program
        display_mode = (Gtk.gdkgl.MODE_RGB | Gtk.gdkgl.MODE_DEPTH | Gtk.gdkgl.MODE_DOUBLE)
        try:
            glconfig = Gtk.gdkgl.Config(mode=display_mode)
        except Gtk.gdkgl.NoMatches:
            raise SystemExit

        self.v_box = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.editor = GraphicalEditor(glconfig)
        self.editor.add_events(Gdk.EventMask.BUTTON_PRESS_MASK | Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.BUTTON_MOTION_MASK |
                               Gdk.EventMask.KEY_PRESS_MASK | Gdk.EventMask.KEY_RELEASE_MASK | Gdk.EventMask.POINTER_MOTION_MASK)
        self.editor.set_size_request(0, 0)
        self.editor.set_flags(Gtk.CAN_FOCUS)

        self.v_box.pack_end(self.editor, True, True, 0)

        self['main_frame'] = self.v_box
        self.top = 'main_frame'


# old; GTK TODO
# class GraphicalEditor(Gtk.DrawingArea, Gtk.gtkgl.Widget):
class GraphicalEditor(Gtk.DrawingArea, Gtk.Widget):
    # background_color = Color.from_hex(0x17242f)
    background_color = Color.from_hex_string(gui_config.colors['BASE_COLOR'])
    state_color = Color.from_hex(0xd7e0ec)  # Color(0.9, 0.9, 0.9, 0.8)
    state_selected_color = Color.from_hex(0xd7e0ec)  # Color(0.7, 0, 0, 0.8)
    state_active_color = Color.from_hex(0xb7d9b0)  # Color(0.7, 0, 0, 0.8)
    state_child_active_color = Color.from_hex(0xCFDEDD)  # Color(0.7, 0, 0, 0.8)
    state_waiting_for_next_state_color = Color.from_hex(0xffffcc, 150)
    state_name_color = Color.from_hex(0x0b0b17)  # Color(0.2, 0.2, 0.2, 1)
    border_color = Color.from_hex(0x0b0b17)  # Color(0.2, 0.2, 0.2, 1)
    border_selected_color = Color.from_hex(0x3aaf59)  # Color(0, 0.8, 0.8, 1)
    border_active_color = Color.from_hex(0x0b0b17)  # Color(0, 0.8, 0.8, 1)
    port_color = Color.from_hex(0xD1DDF4, 200)  # Color(0.7, 0.7, 0.7, 0.8)
    port_name_color = state_name_color  # Color(0.1, 0.1, 0.1, 1)
    port_connector_fill_color = state_selected_color  # Color(0.2, 0.2, 0.2, 0.5)
    transition_color = Color.from_hex(0xabce6d)  # Color(0.4, 0.4, 0.4, 0.8)
    transition_selected_color = border_selected_color  # Color.from_hex(0xabce6d)  # Color(0.7, 0, 0, 0.8)
    data_flow_color = Color.from_hex(0x7fd1c6)  # Color(0.6, 0.6, 0.6, 0.8)
    data_flow_selected_color = border_selected_color  # Color.from_hex(0x3aaf59)  # Color(0.7, 0, 0, 0.8)
    outcome_plain_color = Color.from_hex(0x97d88a)  # Color(0.4, 0.4, 0.4, 0.8)
    outcome_aborted_color = Color.from_hex(0x792b40)  # Color(0.6, 0, 0, 0.8)
    outcome_preempted_color = Color.from_hex(0x4769bd)  # Color(0.1, 0.1, 0.7, 0.8)
    income_color = outcome_plain_color  # Color(0.4, 0.4, 0.4, 0.8)
    frame_fill_color = Color.from_hex(0xd7e0ec, 125)
    frame_border_color = Color.from_hex(0x0b0b17, 100)

    def __init__(self, glconfig):
        """The graphical editor manages the OpenGL functions.

        It only works in combination with its controller.

        :param glconfig: Configuration flags for OpenGl
        """
        Gtk.DrawingArea.__init__(self)

        # default outer coordinate values which will later be overwritten by the controller
        self.left = -10
        self.right = 110
        self.top = 10
        self.bottom = -110

        # Used to generate unique ids for drawn objects
        self.name_counter = 0

        # Set OpenGL-capability to the drawing area
        self.set_gl_capability(glconfig)

        # Needed for glut functions
        glutInit([])

        # Connect the relevant signals.
        self.connect_after('realize', self._realize)
        self.connect('configure_event', self._configure)
        # self.connect('expose_event',    self.expose)

    def _realize(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        # gldrawable = self.get_gl_drawable()
        # glcontext = self.get_gl_context()

        # Query the GLX and OpenGL extension version.
        opengl_version = glGetString(GL_VERSION)
        major, minor = Gtk.gdkgl.query_version()
        logger.info("OpenGL version: {0}".format(opengl_version))
        logger.info("GLX version: {0}.{1}".format(major, minor))

        glEnable(GL_DEPTH_TEST)  # Draw with respect to the z coordinate (hide objects beneath others)
        glEnable(GL_BLEND)  # Make use of alpha channel for colors
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  # Configure alpha blending
        glEnable(GL_LINE_SMOOTH)  # Smooth lines
        glClearColor(self.background_color.r, self.background_color.g, self.background_color.b, 1)  # Background color

    def _configure(self, *args):
        """Configure viewport

        This method is called when the widget is resized or something triggers a redraw. The method configures the
        view to show all elements in an orthogonal perspective.
        """
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        # logger.debug("configure")
        # OpenGL begin
        if not gldrawable or not gldrawable.gl_begin(glcontext):
            return False

        # Draw on the full viewport
        glViewport(0, 0, self.get_allocation().width, self.get_allocation().height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        # Orthogonal view with correct aspect ratio
        self._apply_orthogonal_view()

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # OpenGL end
        gldrawable.gl_end()

        return False

    def screen_to_opengl_coordinates(self, pos):
        conversion = self.pixel_to_size_ratio()
        viewport = glGetInteger(GL_VIEWPORT)
        # Screen to window coordinates
        window = (pos[0], viewport[3] - pos[1] + viewport[1])

        left, _, bottom, _ = self.get_view_coordinates()

        # Window to OpenGL coordinates
        opengl = (window[0] / conversion + left, window[1] / conversion + bottom)
        return opengl

    def pixel_to_size_ratio(self):
        """Calculates the ratio between pixel and OpenGL distances

        OpenGL keeps its own coordinate system. This method can be used to transform between pixel and OpenGL
        coordinates.

        :return: pixel/size ratio
        """
        left, right, _, _ = self.get_view_coordinates()
        width = right - left
        display_width = self.get_allocation().width
        return display_width / float(width)

    def expose_init(self, *args):
        """Process the drawing routine
        """
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        # OpenGL begin
        if not gldrawable or not gldrawable.gl_begin(glcontext):
            return False

        # logger.debug("expose_init")

        # Reset buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Prepare name stack
        glInitNames()
        glPushName(0)
        self.name_counter = 1

        return False

    def expose_finish(self, *args):
        """Finish drawing process
        """
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        # glcontext = self.get_gl_context()

        if not gldrawable:
            return

        # Put the buffer on the screen!
        if gldrawable.is_double_buffered():
            gldrawable.swap_buffers()
        else:
            glFlush()

        # OpenGL end
        gldrawable.gl_end()

        # logger.debug("expose_finish")

    def draw_frame(self, corner1, corner2, depth):
        # corner2 = (corner1[0], corner3[1])
        # corner4 = (corner1[1], corner3[0])
        self._draw_rect(corner1[0], corner2[0], corner1[1], corner2[1], depth, fill_color=self.frame_fill_color,
                        border_color=self.frame_border_color)

    def draw_state(self, name, pos, size, outcomes=None, input_ports_m=None, output_ports_m=None, selected=False,
                   active=False, depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified (container) state.

        :param name: Name of the state
        :param pos: position (x, y) of the state
        :param size: size (width, height) of the state
        :param outcomes: outcomes of the state (list with outcome objects)
        :param input_ports_m: input ports of the state
        :param output_ports_m: output ports of the state
        :param selected: whether to display the state as selected
        :param active: in which color to display the state
        :param depth: The z layer
        :return: The OpenGL id and the positions of teh outcomes (as dictionary with outcome id as key)
        """
        if not outcomes:
            outcomes = []
        if not input_ports_m:
            input_ports_m = []
        if not output_ports_m:
            output_ports_m = []
        # "Generate" unique ID for each object
        opengl_id = self.name_counter
        self.name_counter += 1
        glPushName(opengl_id)
        self._set_closest_stroke_width(1.5)

        width = size[0]
        height = size[1]

        # First draw the face of the rectangular, then the outline
        fill_color = self.state_color
        border_color = self.border_color

        from rafcon.core.states.state import StateExecutionStatus
        border_width = min(size) / 10.
        if active is StateExecutionStatus.ACTIVE:
            fill_color = self.state_active_color
        elif active is StateExecutionStatus.EXECUTE_CHILDREN:
            fill_color = self.state_child_active_color
        elif active is StateExecutionStatus.WAIT_FOR_NEXT_STATE:
            fill_color = self.state_waiting_for_next_state_color

        if selected:
            border_width *= 2
            border_color = self.border_selected_color

        self._draw_rect(pos[0], pos[0] + width, pos[1] - height, pos[1], depth, border_width,
                        fill_color, border_color)

        # Put the name of the state in the upper left corner of the state
        margin = min(size) / 12.
        font_size = min(size) / 8.
        name = self._shorten_string(name, font_size, width - 2 * margin)
        self._write_string(name, pos[0] + margin, pos[1] - margin, font_size, self.state_name_color, True,
                           False, depth=depth + 0.01)

        resize_length = min(width, height) / 8.
        p1 = (pos[0] + width, pos[1] - height)
        p2 = (p1[0] - resize_length, p1[1])
        p3 = (p1[0], p1[1] + resize_length)
        self._draw_triangle(p1, p2, p3, depth + 0.01, border_width, fill_color, border_color)

        # Draw outcomes as circle on the right side of the state
        # Every state has at least the default outcomes "aborted" and "preempted"
        num_outcomes = max(0, len(outcomes))
        if num_outcomes < 2:
            # logger.warning("Expecting at least 2 outcomes, found {num:d}".format(num=num_outcomes))
            pass
        else:
            num_outcomes -= 2
        i = 0
        outcome_pos = {}
        outcome_radius = min(min(height, width) / 23., min(height, width) / (2. * num_outcomes + 3))
        for key in outcomes:
            # Color of outcome is defined by its type, "aborted", "preempted" or else
            outcome_name = outcomes[key].name
            color = self.outcome_plain_color

            # Distribute outcomes (as circles) on the right edge of the state
            outcome_x = pos[0] + width
            outcome_y = pos[1] - height / (num_outcomes + 1) * (i + 1)

            if outcome_name in ["aborted", "preempted"]:
                step = min(size) / 10.
                outcome_y = pos[1]
                if outcome_name == "aborted":
                    outcome_x -= step
                    color = self.outcome_aborted_color
                else:
                    outcome_x -= 3 * step
                    color = self.outcome_preempted_color
            else:
                outcome_font_size = font_size * 0.5
                max_outcome_name_width = width / 2.
                outcome_name = self._shorten_string(outcome_name, outcome_font_size, max_outcome_name_width)
                outcome_name_pos_x = outcome_x - margin
                outcome_name_pos_y = outcome_y + outcome_font_size * 0.65
                self._write_string(outcome_name, outcome_name_pos_x, outcome_name_pos_y, outcome_font_size,
                                   self.state_name_color, align_right=True, depth=depth+0.1)
                i += 1

            color.set()
            outcome_pos[key] = (outcome_x, outcome_y)

            self._draw_circle(outcome_x, outcome_y, outcome_radius, depth + 0.1, fill_color=color)

        # Draw "income" as a half circle of the left center
        income_pos = (pos[0], pos[1] - height / 2)
        self._draw_circle(income_pos[0], income_pos[1], outcome_radius, depth + 0.1, fill_color=self.income_color,
                          from_angle=1.5 * pi, to_angle=0.5 * pi)

        # Draw input and output data ports
        port_radius = margin / 4.
        num_ports = len(input_ports_m) + len(output_ports_m)
        if num_ports > 0:
            max_name_width = 0
            margin = min(size) / 10.0
            max_allowed_name_width = 0.7 * width - margin
            str_height = height / 12.0

            # Determine the maximum width of all port labels
            for ports in [input_ports_m, output_ports_m]:
                for port_m in ports:
                    str_width = self._string_width(port_m.data_port.name, str_height)
                    if str_width > max_name_width:
                        max_name_width = str_width

            #fill_color = self.port_color if not selected else self.state_selected_color

            port_width = min(max_name_width, max_allowed_name_width)
            port_pos_left_x = pos[0] + (width - port_width - margin) / 2
            port_pos_right_x = port_pos_left_x + port_width + margin
            port_pos_bottom_y = pos[1] - height - num_ports * (str_height + margin)
            self._draw_rect(port_pos_left_x, port_pos_right_x, port_pos_bottom_y, pos[1] - height, depth, border_width,
                            fill_color, border_color)

            def draw_port(port_m, num, is_input):

                port_name = self._shorten_string(port_m.data_port.name, str_height, port_width)

                string_pos_x = port_pos_left_x + margin / 2.
                if not is_input:
                    string_pos_x += port_width
                string_pos_y = pos[1] - height - margin / 2. - num * (str_height + margin)
                self._write_string(port_name, string_pos_x, string_pos_y,
                                   str_height, self.port_name_color, False, not is_input, depth + 0.01)

                circle_pos_x = port_pos_left_x if is_input else port_pos_right_x
                circle_pos_y = string_pos_y - margin / 2.
                self._draw_circle(circle_pos_x, circle_pos_y, port_radius, depth + 0.02, stroke_width=margin / 5.,
                                  border_color=self.port_name_color, fill_color=self.port_connector_fill_color)
                return circle_pos_x, circle_pos_y

            output_num = 0
            for port_m in input_ports_m:
                con_pos_x, con_pos_y = draw_port(port_m, output_num, True)
                port_m.temp['gui']['editor']['outer_connector_radius'] = port_radius
                port_m.temp['gui']['editor']['outer_connector_pos'] = (con_pos_x, con_pos_y)
                output_num += 1

            for port_m in output_ports_m:
                con_pos_x, con_pos_y = draw_port(port_m, output_num, False)
                port_m.temp['gui']['editor']['outer_connector_radius'] = port_radius
                port_m.temp['gui']['editor']['outer_connector_pos'] = (con_pos_x, con_pos_y)
                output_num += 1

        glPopName()
        return opengl_id, income_pos, outcome_pos, outcome_radius, resize_length

    def draw_inner_input_data_port(self, port_name, port_m, pos, size, selected, depth):
        return self._draw_inner_data_port(port_name, port_m, pos, size, Direction.right, selected, True, depth)

    def draw_inner_output_data_port(self, port_name, port_m, pos, size, selected, depth):
        return self._draw_inner_data_port(port_name, port_m, pos, size, Direction.left, selected, True, depth)

    def draw_scoped_data_port(self, port_name, port_m, pos, size, selected, depth):
        return self._draw_inner_data_port(port_name, port_m, pos, size, Direction.bottom, selected, False, depth)

    def _draw_inner_data_port(self, port_name, port_m, pos, size, arrow_position, selected, inner, depth):
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)

        width = size[0]
        height = size[1]

        margin = height / 5.
        name_height = height - 2 * margin
        name_width = self._string_width(port_name, name_height)
        if name_width > width - 2 * margin:
            port_name = self._shorten_string(port_name, name_height, width - 2 * margin)
        name_width = self._string_width(port_name, name_height)
        width = name_width + 2 * margin

        if arrow_position == Direction.right or arrow_position == Direction.bottom:
            left = pos[0]
            right = pos[0] + width
        else:
            left = pos[0] - width
            right = pos[0]

        fill_color = self.port_color  # if not selected else self.state_selected_color
        border_color = self.border_color if not selected else self.border_selected_color
        border_width = margin * 2 if not selected else margin * 4
        arrow_pos, visible = self._draw_rect_arrow(left, right, pos[1] - height, pos[1], arrow_position, depth,
                                                   border_width=border_width,
                                                   border_color=border_color, fill_color=fill_color)
        radius = margin / 1.5
        self._draw_circle(arrow_pos[0], arrow_pos[1], radius, depth + 0.02, stroke_width=margin,
                          border_color=self.port_name_color, fill_color=self.port_connector_fill_color)
        self._write_string(port_name, left + margin, pos[1] - margin / 2, name_height,
                           color=self.port_name_color,
                           depth=depth + 0.1)
        prefix = 'inner_' if inner else ''
        port_m.temp['gui']['editor'][prefix + 'connector_pos'] = arrow_pos
        port_m.temp['gui']['editor'][prefix + 'connector_radius'] = radius
        if arrow_position == Direction.right:
            actual_width = arrow_pos[0] - left
            actual_height = height
        elif arrow_position == Direction.left:
            actual_width = right - arrow_pos[0]
            actual_height = height
        elif arrow_position == Direction.top:
            actual_width = right - left
            actual_height = abs(arrow_pos[1] - pos[1])
        else:  # arrow_position == Direction.bottom:
            actual_width = right - left
            actual_height = abs(arrow_pos[1] - pos[1])

        port_m.temp['gui']['editor']['size'] = (actual_width, actual_height)
        port_m.temp['gui']['editor']['size_rect'] = (width, height)

        glPopName()

        return id

    def draw_transition(self, from_pos, to_pos, width, waypoints=None, selected=False, depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified transition.

        :param tuple from_pos: Starting position
        :param tuple to_pos: Ending position
        :param float width: A measure for the width of a transition line
        :param list waypoints: A list of optional waypoints to connect in between
        :param bool selected: Whether the transition shell be shown as active/selected
        :param float depth: The Z layer
        :return: The OpenGL id of the transition
        :rtype: int
        """
        if not waypoints:
            waypoints = []
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        self._set_closest_stroke_width(width)

        color = self.transition_color if not selected else self.transition_selected_color
        color.set()

        points = [from_pos]
        points.extend(waypoints)

        last_p = to_pos  # Transition endpoint
        sec_last_p = points[len(points) - 1]  # Point before endpoint
        # Calculate max possible arrow length
        length = min(width, dist(sec_last_p, last_p) / 2.)
        mid, p2, p3 = self._calculate_arrow_points(last_p, sec_last_p, length)
        self._draw_triangle(last_p, p2, p3, depth, fill_color=color)

        points.append(mid)

        # Draw the transitions as simple straight line connecting start- way- and endpoints
        glBegin(GL_LINE_STRIP)
        for point in points:
            glVertex3f(point[0], point[1], depth)
        glEnd()

        self._set_closest_stroke_width(width / 1.5)
        for waypoint in waypoints:
            self._draw_circle(waypoint[0], waypoint[1], width / 6., depth + 1, fill_color=color)

        glPopName()

        return id

    def draw_data_flow(self, from_pos, to_pos, width, waypoints=None, selected=False, depth=0):
        """Draw a data flow connection between two ports

        The ports can be input, output or scoped ports and are only specified by their position. Optional waypoints
        allow non-direct connection.

        :param tuple from_pos: Starting position
        :param tuple to_pos: Ending position
        :param float width: A measure for the width of a transition line
        :param list waypoints: A list of optional waypoints to connect in between
        :param bool selected: Whether the transition shell be shown as active/selected
        :param in tdepth: The Z layer
        """
        if not waypoints:
            waypoints = []
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        width /= 1.3
        self._set_closest_stroke_width(width)

        color = self.data_flow_color if not selected else self.data_flow_selected_color
        color.set()

        points = [from_pos]
        points.extend(waypoints)

        last_p = to_pos  # Transition endpoint
        sec_last_p = points[len(points) - 1]  # Point before endpoint
        # Calculate max possible arrow length
        length = min(width / 1.2, dist(sec_last_p, last_p) / 2.)
        mid, p2, p3 = self._calculate_arrow_points(last_p, sec_last_p, length)
        self._draw_triangle(last_p, p2, p3, depth, fill_color=color)

        points.append(mid)

        # Draw the transitions as simple straight line connecting start- way- and endpoints
        glBegin(GL_LINE_STRIP)
        for point in points:
            glVertex3f(point[0], point[1], depth)
        glEnd()

        self._set_closest_stroke_width(width / 1.5)
        for waypoint in waypoints:
            self._draw_circle(waypoint[0], waypoint[1], width / 6., depth + 1, fill_color=color)

        glPopName()

        return id

    def _write_string(self, string, pos_x, pos_y, height, color, bold=False, align_right=False, depth=0.):
        """Write a string

        Writes a string with a simple OpenGL method in the given size at the given position.

        :param string: The string to draw
        :param pos_x: x starting position
        :param pos_y: y starting position
        :param height: desired height
        :param bold: flag whether to use a bold font
        :param depth: the Z layer
        """
        stroke_width = height / 8.
        if bold:
            stroke_width = height / 5.
        color.set()
        self._set_closest_stroke_width(stroke_width)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        pos_y -= height
        if not align_right:
            glTranslatef(pos_x, pos_y, depth)
        else:
            width = self._string_width(string, height)
            glTranslatef(pos_x - width, pos_y, depth)
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        glScalef(scale_factor, scale_factor, scale_factor)
        for c in string:
            # glTranslatef(0, 0, 0)
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
            # width = glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))

        glPopMatrix()

    @staticmethod
    def _string_width(string, height):
        width = 0
        for c in string:
            width += glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        return width * scale_factor

    def prepare_selection(self, pos_x, pos_y, width, height):
        """Prepares the selection rendering

        In order to find out the object being clicked on, the scene has to be rendered again around the clicked position

        :param pos_x: x coordinate
        :param pos_y: y coordinate
        """
        glSelectBuffer(self.name_counter * 6)
        viewport = glGetInteger(GL_VIEWPORT)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()

        glRenderMode(GL_SELECT)

        glLoadIdentity()

        if width < 1:
            width = 1
        if height < 1:
            height = 1
        pos_x += width / 2.
        pos_y += height / 2.
        # The system y axis is inverse to the OpenGL y axis
        gluPickMatrix(pos_x, viewport[3] - pos_y + viewport[1], width, height, viewport)

        self._apply_orthogonal_view()

    @staticmethod
    def find_selection():
        """Finds the selected ids

        After the scene has been rendered again in selection mode, this method gathers and returns the ids of the
        selected object and restores the matrices.

        :return: The selection stack
        """
        hits = glRenderMode(GL_RENDER)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

        return hits

    @staticmethod
    def transition_stroke_width(parent_state_m):
        return min(parent_state_m.get_meta_data_editor(for_gaphas=False)['size']) / 25.0

    @staticmethod
    def data_flow_stroke_width(parent_state_m):
        return min(parent_state_m.get_meta_data_editor(for_gaphas=False)['size']) / 25.0

    def _set_closest_stroke_width(self, width):
        """Sets the line width to the closest supported one

        Not all line widths are supported. This function queries both minimum and maximum as well as the step size of
        the line width and calculates the width, which is closest to the given one. This width is then set.

        :param width: The desired line width
        """

        # Adapt line width to zooming level
        width *= self.pixel_to_size_ratio() / 6.
        stroke_width_range = glGetFloatv(GL_LINE_WIDTH_RANGE)
        stroke_width_granularity = glGetFloatv(GL_LINE_WIDTH_GRANULARITY)

        if width < stroke_width_range[0]:
            glLineWidth(stroke_width_range[0])
            return
        if width > stroke_width_range[1]:
            glLineWidth(stroke_width_range[1])
            return
        glLineWidth(round(width / stroke_width_granularity) * stroke_width_granularity)

    @staticmethod
    def _calculate_arrow_points(end_point, base_point, max_length):
        # Arrow direction (vector)
        vec = (end_point[0] - base_point[0], end_point[1] - base_point[1])
        # Arrow direction (angle)
        angle = atan2(vec[1], vec[0])
        angle -= pi  # backwards
        # Middle point of back end of arrow
        m_x = end_point[0] + cos(angle) * max_length
        m_y = end_point[1] + sin(angle) * max_length
        angle += pi / 2
        dx = cos(angle) * max_length / 2
        dy = sin(angle) * max_length / 2
        # Corner points
        p2 = (m_x + dx, m_y + dy)
        p3 = (m_x - dx, m_y - dy)
        return (m_x, m_y), p2, p3

    def _draw_polygon(self, points, depth, border_width=1., fill_color=None, border_color=None):
        # TODO: Think of method to check for visibility
        # visible = False
        # for p in points:
        # if not self.point_outside_view(p):
        # visible = True
        #         break

        # if not visible:
        #     return False

        types = []
        if fill_color is not None:
            types.append(GL_POLYGON)
        if border_color is not None:
            types.append(GL_LINE_LOOP)

        for type in types:
            if type == GL_POLYGON:
                fill_color.set()
            else:
                self._set_closest_stroke_width(border_width)
                border_color.set()
            glBegin(type)
            for p in points:
                glVertex3f(p[0], p[1], depth)
            glEnd()
        return True

    def _draw_triangle(self, p1, p2, p3, depth, border_width=1., fill_color=None, border_color=None):
        self._draw_polygon([p1, p2, p3], depth, border_width, fill_color, border_color)

    def _draw_rect(self, left_x, right_x, bottom_y, top_y, depth, border_width=1., fill_color=None, border_color=None):
        p1 = (left_x, bottom_y)
        p2 = (right_x, bottom_y)
        p3 = (right_x, top_y)
        p4 = (left_x, top_y)

        return self._draw_polygon([p1, p2, p3, p4], depth, border_width, fill_color, border_color)

    def _draw_rect_arrow(self, left_x, right_x, bottom_y, top_y, arrow_pos, depth,
                         border_width=1., fill_color=None, border_color=None):
        visible = self._draw_rect(left_x, right_x, bottom_y, top_y, depth, border_width, fill_color, border_color)

        width = right_x - left_x
        height = top_y - bottom_y
        arrow_width = min(width, height) / 1.5
        a = (0, 0)
        b = (0, 0)
        c = (0, 0)

        if arrow_pos == Direction.top:
            a = (left_x + width / 2 - arrow_width / 2, top_y)
            b = (left_x + width / 2 + arrow_width / 2, top_y)
            c = (left_x + width / 2, top_y + arrow_width)
        elif arrow_pos == Direction.bottom:
            a = (left_x + width / 2 - arrow_width / 2, bottom_y)
            b = (left_x + width / 2 + arrow_width / 2, bottom_y)
            c = (left_x + width / 2, bottom_y - arrow_width)
        elif arrow_pos == Direction.left:
            a = (left_x, bottom_y + height / 2 - arrow_width / 2)
            b = (left_x, bottom_y + height / 2 + arrow_width / 2)
            c = (left_x - arrow_width, bottom_y + height / 2)
        elif arrow_pos == Direction.right:
            a = (right_x, bottom_y + height / 2 - arrow_width / 2)
            b = (right_x, bottom_y + height / 2 + arrow_width / 2)
            c = (right_x + arrow_width, bottom_y + height / 2)

        points = [a, b, c]
        visible |= self._draw_polygon(points, depth, border_width, fill_color, border_color)

        return c, visible

    def _draw_circle(self, pos_x, pos_y, radius, depth, stroke_width=1., fill_color=None, border_color=None,
                     from_angle=0., to_angle=2 * pi):
        """Draws a circle

        Draws a circle with a line segment a desired position with desired size.

        :param float pos_x: Center x position
        :param float pos_y: Center y position
        :param float depth: The Z layer
        :param float radius: Radius of the circle
        """

        visible = False
        # Check whether circle center is in the viewport
        if not self.point_outside_view((pos_x, pos_y)):
            visible = True
        # Check whether at least on point on the border of the circle is within the viewport
        if not visible:
            for i in range(0, 8):
                angle = 2 * pi / 8. * i
                x = pos_x + cos(angle) * radius
                y = pos_y + sin(angle) * radius
                if not self.point_outside_view((x, y)):
                    visible = True
                    break
        if not visible:
            return False

        angle_sum = to_angle - from_angle
        if angle_sum < 0:
            angle_sum = float(to_angle + 2 * pi - from_angle)
        segments = self.pixel_to_size_ratio() * radius * 1.5
        segments = max(4, segments)
        segments = int(round(segments * angle_sum / (2. * pi)))

        types = []
        if fill_color is not None:
            types.append(GL_POLYGON)
        if border_color is not None:
            types.append(GL_LINE_LOOP)

        for type in types:
            if type == GL_POLYGON:
                fill_color.set()
            else:
                self._set_closest_stroke_width(stroke_width)
                border_color.set()
            glBegin(type)
            angle = from_angle
            for i in range(0, segments):
                x = pos_x + cos(angle) * radius
                y = pos_y + sin(angle) * radius
                glVertex3f(x, y, depth)
                angle += angle_sum / (segments - 1)
                if angle > 2 * pi:
                    angle -= 2 * pi
                if i == segments - 2:
                    angle = to_angle
            glEnd()

        return True

    def _shorten_string(self, string, height, max_width):
        trim_len = len(string)
        while trim_len > 1:
            if self._string_width(string[0:trim_len - 1], height) <= max_width:
                break
            trim_len -= 1
        if trim_len < 3 and trim_len < len(string):
            string = ''
        elif trim_len < len(string):
            string = string[0:trim_len - 2] + '~'
        return string

    def point_outside_view(self, p):
        left, right, bottom, top = self.get_view_coordinates()
        if p[0] < left or p[0] > right:
            return True
        elif p[1] < bottom or p[1] > top:
            return True
        return False

    def get_view_coordinates(self):
        left = self.left
        right = self.right
        top = self.top
        bottom = self.bottom

        aspect = self.get_allocation().width / float(self.get_allocation().height)

        if aspect < 1:
            bottom /= aspect
            top /= aspect
        elif aspect > 1:
            left *= aspect
            right *= aspect

        return left, right, bottom, top

    def get_size(self):
        left, right, bottom, top = self.get_view_coordinates()
        return right - left, top - bottom

    def _apply_orthogonal_view(self):
        """Orthogonal view with respect to current aspect ratio
        """
        left, right, bottom, top = self.get_view_coordinates()
        glOrtho(left, right, bottom, top, -10, 0)
