from utils import log

logger = log.get_logger(__name__)

from math import sin, cos, pi, floor, ceil
from itertools import chain
from enum import Enum

import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
# Activate the following line in the production code th increase speed
# http://pyopengl.sourceforge.net/documentation/opengl_diffs.html:
# OpenGL.ERROR_CHECKING = False
OpenGL.FULL_LOGGING = True
# Also in productive code, set the previous statement to false and activate the next one
# OpenGL.ERROR_LOGGING = False
from OpenGL.GLUT import *

# from gdk import eve
import gtk
import gtk.gtkgl
import gtk.gdkgl
from gtkmvc import View


class Direction(Enum):
    top = 1
    right = 2
    bottom = 3
    left = 4


class Color:
    def __init__(self, r, g, b, a=1):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

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
    top = 'main_frame'

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        View.__init__(self)

        # Configure OpenGL frame buffer.
        # Try to get a double-buffered frame buffer configuration,
        # if not successful then exit program
        display_mode = (gtk.gdkgl.MODE_RGB | gtk.gdkgl.MODE_DEPTH | gtk.gdkgl.MODE_DOUBLE)
        try:
            glconfig = gtk.gdkgl.Config(mode=display_mode)
        except gtk.gdkgl.NoMatches:
            raise SystemExit

        # Only temporary, later the editor won't be in an own window
        # self.win = gtk.Window(gtk.WINDOW_TOPLEVEL)
        # self.win.set_title("Graphical Editor")
        # self.win.set_position(1)
        self.v_box = gtk.VBox()
        #self.test_label = gtk.Label("Hallo")
        self.editor = GraphicalEditor(glconfig)
        self.editor.add_events(gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK | gtk.gdk.BUTTON_MOTION_MASK |
                               gtk.gdk.KEY_PRESS_MASK | gtk.gdk.KEY_RELEASE_MASK)
        self.editor.set_size_request(500, 500)
        self.editor.set_flags(gtk.CAN_FOCUS)

        #self.v_box.pack_start(self.test_label)
        self.v_box.pack_end(self.editor)

        # self.win.add(self.v_box)
        # self.win.show_all()
        # self.win.connect("destroy", lambda w: gtk.main_quit())
        self['main_frame'] = self.v_box

        # Query the OpenGL extension version.
        print "OpenGL extension version - %d.%d\n" % gtk.gdkgl.query_version()

        # def get_top_widget(self):
        #     return self.win


class GraphicalEditor(gtk.DrawingArea, gtk.gtkgl.Widget):
    border_color = Color(0.2, 0.2, 0.2, 1)
    border_active_color = Color(0, 0.8, 0.8, 1)
    state_color = Color(0.9, 0.9, 0.9, 0.8)
    state_selected_color = Color(0.7, 0, 0, 0.8)
    state_name_color = Color(0.2, 0.2, 0.2, 1)
    port_color = Color(0.7, 0.7, 0.7, 0.8)
    port_name_color = Color(0.2, 0.2, 0.2, 1)
    transition_color = Color(0.4, 0.4, 0.4, 0.8)
    transition_selected_color = Color(0.7, 0, 0, 0.8)
    data_flow_color = Color(0.6, 0.6, 0.6, 0.8)
    data_flow_selected_color = Color(0.7, 0, 0, 0.8)
    outcome_plain_color = Color(0.4, 0.4, 0.4, 0.8)
    outcome_aborted_color = Color(0.6, 0, 0, 0.8)
    outcome_preempted_color = Color(0.1, 0.1, 0.7, 0.8)

    def __init__(self, glconfig):
        """The graphical editor manages the OpenGL functions.

        It only works in combination with its controller.

        :param glconfig: Configuration flags for OpenGl
        """
        gtk.DrawingArea.__init__(self)

        # default outer coordinate values which will later be overwritten by the controller
        self.left = -10
        self.right = 110
        self.top = 110
        self.bottom = -10

        # Used to generate unique ids for drawn objects
        self.name_counter = 0

        # Set OpenGL-capability to the drawing area
        self.set_gl_capability(glconfig)

        # Needed for glut functions
        glutInit([])

        # Connect the relevant signals.
        self.connect_after('realize', self._realize)
        self.connect('configure_event', self._configure)
        #self.connect('expose_event',    self.expose)

    @staticmethod
    def _realize(*args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        #gldrawable = self.get_gl_drawable()
        #glcontext = self.get_gl_context()

        glEnable(GL_DEPTH_TEST)  # Draw with respect to the z coordinate (hide objects beneath others)
        glEnable(GL_BLEND)  # Make use of alpha channel for colors
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  # Configure alpha blending
        glEnable(GL_LINE_SMOOTH)  # Smooth lines
        glClearColor(33. / 255, 49. / 255, 92. / 255, 1)  # Background color

        logger.debug("realize")

        #self.configure()

    def _configure(self, *args):
        """Configure viewport

        This method is called when the widget is resized or something triggers a redraw. The method configures the
        view to show all elements in an orthogonal perspective.
        """
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        #logger.debug("configure")
        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        # Draw on the full viewport
        glViewport(0, 0, self.allocation.width, self.allocation.height)

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

        left, _, bottom, _ = self._get_view_coordinates()

        # Window to OpenGL coordinates
        opengl = (window[0] / conversion + left, window[1] / conversion + bottom)
        return opengl

    def pixel_to_size_ratio(self):
        """Calculates the ratio between pixel and OpenGL distances

        OpenGL keeps its own coordinate system. This method can be used to transform between pixel and OpenGL
        coordinates.
        :return: pixel/size ratio
        """
        width = self.right - self.left
        if self.allocation.width > self.allocation.height:
            width *= self.allocation.width / float(self.allocation.height)
        display_width = self.allocation.width
        return display_width / float(width)

    def expose_init(self, *args):
        """Process the drawing routine
        """
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        #logger.debug("expose_init")

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
        #glcontext = self.get_gl_context()

        # Put the buffer on the screen!
        if gldrawable.is_double_buffered():
            gldrawable.swap_buffers()
        else:
            glFlush()

        # OpenGL end
        gldrawable.gl_end()

        #logger.debug("expose_finish")

    def draw_state(self, name, pos_x, pos_y, width, height, outcomes=0, inputs={}, outputs={}, scoped_vars={},
                   selected=False, active=False, depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified (container) state.
        :param name: Name of the state
        :param pos_x: x position of the state
        :param pos_y: y position of the state
        :param width: width of the state
        :param height: height of the state
        :param outcomes: outcomes of the state (list with outcome objects)
        :param inputs: input ports of the state
        :param outputs: output ports of the state
        :param scoped_vars: scoped variable ports of the state
        :param active: whether to display the state as active/selected
        :param depth: The z layer
        :return: The OpenGL id and the positions of teh outcomes (as dictionary with outcome id as key)
        """
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1
        glPushName(id)
        self._set_closest_stroke_width(1.5)

        # First draw the face of the rectangular, then the outline
        fill_color = self.state_color if not selected else self.state_selected_color

        border_width = min(width, height) / 10.

        border_color = self.border_color if not active else self.border_active_color
        self._draw_rect(pos_x, pos_x + width, pos_y, pos_y + height, depth, border_width,
                        fill_color, border_color)

        # Put the name of the state in the upper left corner of the state
        margin = min(width, height) / 8.
        font_size = min(width, height) / 8.
        name = self._shorten_string(name, font_size, width - 2 * margin)
        self._write_string(name, pos_x + margin, pos_y + height - margin, font_size, self.state_name_color, True,
                           False, depth=depth + 0.01)

        resize_length = min(width, height) / 8.
        p1 = (pos_x + width, pos_y)
        p2 = (p1[0] - resize_length, p1[1])
        p3 = (p1[0], p1[1] + resize_length)
        self._draw_triangle(p1, p2, p3, depth + 0.01, border_width, fill_color, border_color)

        # Draw outcomes as circle on the right side of the state
        # Every state has at least the default outcomes "aborted" and "preempted"
        num_outcomes = max(0, len(outcomes))
        if num_outcomes < 2:
            logger.warn("Expecting at least 2 outcomes, found {num:d}".format(num=num_outcomes))
        i = 0
        outcome_pos = {}
        outcome_radius = min(5, height / 15., height / (2. * num_outcomes + 3))
        for key in outcomes:
            # Color of outcome is defined by its type, "aborted", "preempted" or else
            outcome_name = outcomes[key].name
            color = self.outcome_plain_color
            if outcome_name == "aborted":
                color = self.outcome_aborted_color
            elif outcome_name == "preempted":
                color = self.outcome_preempted_color

            color.set()

            # TODO: Show name of the outcome

            # Distribute outcomes (as circles) on the right edge of the state
            outcome_x = pos_x + width
            outcome_y = pos_y + height / (num_outcomes + 1) * (i + 1)
            outcome_pos[key] = (outcome_x, outcome_y)
            self._draw_circle(outcome_x, outcome_y, outcome_radius, depth + 0.1, fill_color=color)
            i += 1

        # Draw input and output data ports
        num_ports = len(inputs) + len(outputs)
        input_connector_pos = {}
        output_connector_pos = {}
        if num_ports > 0:
            max_name_width = 0
            margin = min(width, height) / 10.0
            max_allowed_name_width = 0.7 * width - margin
            str_height = height / 12.0

            # Determine the maximum width of all port labels
            for ports in [inputs, outputs]:
                for port in ports:
                    port_name = ports[port].name
                    str_width = self._string_width(port_name, str_height)
                    if str_width > max_name_width:
                        max_name_width = str_width

            fill_color = self.port_color
            if active:
                fill_color = self.state_selected_color
            port_width = min(max_name_width, max_allowed_name_width)
            port_pos_left_x = pos_x + (width - port_width - margin) / 2
            port_pos_right_x = port_pos_left_x + port_width + margin
            port_pos_bottom_y = pos_y - num_ports * (str_height + margin)
            self._draw_rect(port_pos_left_x, port_pos_right_x, port_pos_bottom_y, pos_y, depth, border_width,
                            fill_color, self.border_color)

            def draw_port(port, num, is_input):
                port_name = self._shorten_string(port.name, str_height, port_width)

                string_pos_x = port_pos_left_x + margin / 2.
                if not is_input:
                    string_pos_x += port_width
                string_pos_y = pos_y - margin / 2. - num * (str_height + margin)
                self._write_string(port_name, string_pos_x, string_pos_y,
                                   str_height, self.port_name_color, False, not is_input, depth + 0.01)

                circle_pos_x = port_pos_left_x if is_input else port_pos_right_x
                circle_pos_y = string_pos_y - margin / 2.
                self._draw_circle(circle_pos_x, circle_pos_y, margin / 4., depth + 0.02, stroke_width=margin / 5.,
                                  border_color=self.port_name_color)
                return circle_pos_x, circle_pos_y

            output_num = 0
            for port in inputs.itervalues():
                con_pos_x, con_pos_y = draw_port(port, output_num, True)
                input_connector_pos[port.data_port_id] = (con_pos_x, con_pos_y)
                output_num += 1

            for port in outputs.itervalues():
                con_pos_x, con_pos_y = draw_port(port, output_num, False)
                output_connector_pos[port.data_port_id] = (con_pos_x, con_pos_y)
                output_num += 1

        # Draw input and output data ports
        scoped_connector_pos = {}
        if len(scoped_vars) > 0:
            max_scope_width = width * 0.9
            margin = min(height, width) / 50.
            max_single_scope_width = max_scope_width / len(scoped_vars) - 2 * margin
            str_height = height / 35.0
            port_pos_left_x = pos_x + width / 2 - (len(scoped_vars) * max_single_scope_width) / 2
            port_pos_top_y = pos_y + height - margin
            num = 0

            for key in scoped_vars:
                port_name = self._shorten_string(scoped_vars[key].name, str_height, max_single_scope_width - margin)
                str_width = self._string_width(port_name, str_height)

                move_x = num * (str_width + 2 * margin)
                connect, _ = self._draw_rect_arrow(port_pos_left_x + move_x,
                                                   port_pos_left_x + move_x + str_width + margin,
                                                   port_pos_top_y - str_height - margin, port_pos_top_y,
                                                   Direction.bottom, depth + 0.01, border_width / 5.,
                                                   self.port_color, self.border_color)

                string_pos_x = port_pos_left_x + margin / 2. + move_x
                string_pos_y = port_pos_top_y - margin / 2.  # - num * (str_height + margin)
                self._write_string(port_name, string_pos_x, string_pos_y, str_height, self.port_name_color, False,
                                   False, depth + 0.02)

                #circle_pos_x = string_pos_x + margin/2. + str_width/2.
                #circle_pos_y = string_pos_y - margin - str_height
                self._draw_circle(connect[0], connect[1], margin / 4., depth + 0.02,
                                  stroke_width=border_width / 8., border_color=self.port_name_color)
                num += 1
                scoped_connector_pos[key] = connect
                pass

        glPopName()
        return id, outcome_pos, outcome_radius, input_connector_pos, output_connector_pos, scoped_connector_pos, \
               resize_length

    def draw_transition(self, from_pos_x, from_pos_y, to_pos_x, to_pos_y, width, waypoints=[], selected=False,
                        depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified transition.
        :param name: Name of the transition
        :param from_pos_x: Starting x position
        :param from_pos_y: Starting y position
        :param to_pos_x: Ending x position
        :param to_pos_y: Ending y position
        :param width: A measure for the width of a transition line
        :param waypoints: A list of optional waypoints to connect in between
        :param selected: Whether the transition shell be shown as active/selected
        :param depth: The Z layer
        :return: The OpenGL id of the transition
        """
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        self._set_closest_stroke_width(width)

        color = self.transition_color if not selected else self.transition_selected_color
        color.set()

        points = [(from_pos_x, from_pos_y)]
        points.extend(waypoints)
        points.append((to_pos_x, to_pos_y))

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

    def draw_data_flow(self, from_pos_x, from_pos_y, to_pos_x, to_pos_y, width, waypoints=[], selected=False, depth=0):
        """Draw a data flow connection between two ports

        The ports can be input, output or scoped ports and are only specified by their position. Optional waypoints
        allow non-direct connection.

        :param from_pos_x: Starting x position
        :param from_pos_y: Starting y position
        :param to_pos_x: Ending x position
        :param to_pos_y: Ending y position
        :param width: A measure for the width of a transition line
        :param waypoints: A list of optional waypoints to connect in between
        :param selected: Whether the transition shell be shown as active/selected
        :param depth: The Z layer
        """
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        width /= 2
        self._set_closest_stroke_width(width)

        color = self.data_flow_color if not selected else self.data_flow_selected_color
        color.set()

        points = [(from_pos_x, from_pos_y)]
        points.extend(waypoints)
        points.append((to_pos_x, to_pos_y))

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

    def _write_string(self, string, pos_x, pos_y, height, color, bold=False, align_right=False, depth=0):
        """Write a string

        Writes a string with a simple OpenGL method in the given size at the given position
        :param string: The string to draw
        :param pos_x: x starting position
        :param pos_y: y starting position
        :param height: desired height
        :param stroke_width: thickness of the letters
        :param depth: the Z layer
        """
        stroke_width = height / 8.
        if bold:
            stroke_width = height / 5.
        color.set()
        self._set_closest_stroke_width(stroke_width)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        pos_y = pos_y - height
        if not align_right:
            glTranslatef(pos_x, pos_y, depth)
        else:
            width = self._string_width(string, height)
            glTranslatef(pos_x - width, pos_y, depth)
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        glScalef(scale_factor, scale_factor, scale_factor)
        for c in string:
            #glTranslatef(0, 0, 0)
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
            #width = glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))

        glPopMatrix()

    @staticmethod
    def _string_width(string, height):
        width = 0
        for c in string:
            width += glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        return width * scale_factor

    def prepare_selection(self, pos_x, pos_y):
        """Prepares the selection rendering

        In order to find out the object being clicked on, the scene has to be rendered again around the clicked position
        :param pos_x: x coordinate
        :param pos_y: y coordinate
        """
        glSelectBuffer(128)
        viewport = glGetInteger(GL_VIEWPORT)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()

        glRenderMode(GL_SELECT)

        glLoadIdentity()
        # The system y axis is inverse to the OpenGL y axis
        range = 5  # self.pixel_to_size_ratio() / 3.
        gluPickMatrix(pos_x, viewport[3] - pos_y + viewport[1], range, range, viewport)

        self._apply_orthogonal_view()

    @staticmethod
    def find_selection():
        """Finds the selected ids

        After the scene has been rendered again in selection mode, this method gathers and returns the ids of the
        selected object and restores the matrices.
        :return:
        """
        hits = glRenderMode(GL_RENDER)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

        return hits

    #@staticmethod
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

    def _draw_polygon(self, points, depth, border_width=1, fill_color=None, border_color=None):
        # TODO: Think of method to check for visibility
        # visible = False
        # for p in points:
        #     if not self.point_outside_view(p):
        #         visible = True
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

    def _draw_triangle(self, p1, p2, p3, depth, border_width=1, fill_color=None, border_color=None):
        self._draw_polygon([p1, p2, p3], depth, border_width, fill_color, border_color)

    def _draw_rect(self, left_x, right_x, bottom_y, top_y, depth, border_width=1, fill_color=None, border_color=None):
        p1 = (left_x, bottom_y)
        p2 = (right_x, bottom_y)
        p3 = (right_x, top_y)
        p4 = (left_x, top_y)

        return self._draw_polygon([p1, p2, p3, p4], depth, border_width, fill_color, border_color)

    def _draw_rect_arrow(self, left_x, right_x, bottom_y, top_y, arrow_pos, depth,
                         border_width=1, fill_color=None, border_color=None):
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
        elif arrow_pos == Direction.right:
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

    def _draw_circle(self, pos_x, pos_y, radius, depth, stroke_width=1, fill_color=None, border_color=None):
        """Draws a circle

        Draws a circle with a line segment a desired position with desired size.
        :param pos_x: Center x position
        :param pos_y: Center y position
        :param depth: The Z layer
        :param radius: Radius of the circle
        :param segments: Number of segments to draw (the more the more exact)
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

        segments = max(4, int(self.pixel_to_size_ratio() * radius * 1.5))

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
            for i in range(0, segments):
                angle = 2 * pi / segments * i
                x = pos_x + cos(angle) * radius
                y = pos_y + sin(angle) * radius
                glVertex3f(x, y, depth)
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
        left, right, bottom, top = self._get_view_coordinates()
        if p[0] < left or p[0] > right:
            return True
        elif p[1] < bottom or p[1] > top:
            return True
        return False

    def _get_view_coordinates(self):
        left = self.left
        right = self.right
        top = self.top
        bottom = self.bottom

        aspect = self.allocation.width / float(self.allocation.height)

        if aspect < 1:
            bottom /= aspect
            top /= aspect
        elif aspect > 1:
            left *= aspect
            right *= aspect

        return left, right, bottom, top

    def _apply_orthogonal_view(self):
        """Orthogonal view with respect to current aspect ratio
        """
        left, right, bottom, top = self._get_view_coordinates()
        glOrtho(left, right, bottom, top, -10, 0)

