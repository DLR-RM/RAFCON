
from utils import log
logger = log.get_logger(__name__)

from math import sin, cos, pi

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

#from gdk import eve
import gtk
import gtk.gtkgl
import gtk.gdkgl
from gtkmvc import View


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
        display_mode = (gtk.gdkgl.MODE_RGB | gtk.gdkgl.MODE_DEPTH | gtk.gdkgl.MODE_SINGLE)
        try:
            glconfig = gtk.gdkgl.Config(mode=display_mode)
        except gtk.gdkgl.NoMatches:
            raise SystemExit

        # Only temporary, later the editor won't be in an own window
        self.win = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.win.set_title("Graphical Editor")
        self.win.set_position(1)
        self.v_box = gtk.VBox()
        self.test_label = gtk.Label("Hallo")
        self.editor = GraphicalEditor(glconfig)
        self.editor.add_events(gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK | gtk.gdk.BUTTON_MOTION_MASK)
        self.editor.set_size_request(500, 500)

        self.v_box.pack_start(self.test_label)
        self.v_box.pack_end(self.editor)

        self.win.add(self.v_box)
        self.win.show_all()
        self.win.connect("destroy", lambda w: gtk.main_quit())

        # Query the OpenGL extension version.
        print "OpenGL extension version - %d.%d\n" % gtk.gdkgl.query_version()

    def get_top_widget(self):
        return self.win


class GraphicalEditor(gtk.DrawingArea, gtk.gtkgl.Widget):

    def __init__(self, glconfig):
        """The graphical editor manages the OpenGL functions.

        It only works in combination with its controller.

        :param glconfig: Configuration flags for OpenGl
        """
        gtk.DrawingArea.__init__(self)

        # default outer coordinate values which will later be overwritten by the controller
        self.left = 0
        self.right = 100
        self.top = 100
        self.bottom = 0

        # Used to generate unique ids for drawn objects
        self.name_counter = 0

        # Set OpenGL-capability to the drawing area
        self.set_gl_capability(glconfig)

        # Needed for glut functions
        glutInit([])

        # Connect the relevant signals.
        self.connect_after('realize',   self._realize)
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
        glClearColor(33./255, 49./255, 92./255, 1)  # Background color

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
        window = (pos[0], viewport[3] - pos[1] + viewport[1])  # Screen to window coordinates

        left = self.left
        bottom = self.bottom
        aspect = self.allocation.width/float(self.allocation.height)
        if self.allocation.width <= self.allocation.height:
            bottom = self.bottom/aspect
        else:
            left = self.left*aspect


        opengl = (window[0] / float(conversion) + left, window[1] / float(conversion) + bottom)  # Window to
        # OpenGL
        # coordinates
        return opengl

    def pixel_to_size_ratio(self):
        """Calculates the ratio between pixel and OpenGL distances

        OpenGL keeps its own coordinate system. This method can be used to transform between pixel and OpenGL
        coordinates.
        :return: pixel/size ratio
        """
        width = self.right - self.left
        if self.allocation.width > self.allocation.height:
            width *= self.allocation.width/float(self.allocation.height)
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

    def draw_state(self, name, pos_x, pos_y, width, height, outcomes=0, active=False, depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified (container) state.
        :param name: Name of the state
        :param pos_x: x position of the state
        :param pos_y: y position of the state
        :param width: width of the state
        :param height: height of the state
        :param outcomes: outcomes of teh state (list with outcome objects)
        :param active: whether to display the state as active/selected
        :param depth: The z layer
        :return: The OpenGL id and the positions of teh outcomes (as dictionary with outcome id as key)
        """
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1
        glPushName(id)
        self._set_closest_line_width(1.5)

        # First draw the face of the rectangular, then the outline
        for type in (GL_POLYGON, GL_LINE_LOOP):
            if type == GL_POLYGON:
                if active:
                    glColor4f(0.7, 0, 0, 0.8)
                else:
                    glColor4f(0.9, 0.9, 0.9, 0.8)
            else:
                glColor4f(0.2, 0.2, 0.2, 1)
            glBegin(type)
            glVertex3f(pos_x, pos_y, depth)
            glVertex3f(pos_x + width, pos_y, depth)
            glVertex3f(pos_x + width, pos_y + height, depth)
            glVertex3f(pos_x, pos_y + height, depth)
            glEnd()

        # Every state has at least the default outcomes "aborted" and "preempted"
        num_outcomes = max(0, len(outcomes))
        if num_outcomes < 2:
            logger.warn("Expecting at least 2 outcomes, found {num:d}".format(num=num_outcomes))
        i = 0
        outcome_pos = {}
        for key in outcomes:
            # Color of outcome is defined by its type, "aborted", "preempted" or else
            if key == 1:
                glColor3f(0.8, 0, 0)
            elif key == 2:
                glColor3f(0.1, 0.1, 0.7)
            else:
                glColor3f(0.4, 0.4, 0.4)

            # TODO: Show name of the outcome

            # Distribute outcomes (as circles) on the right edge of the state
            outcome_x = pos_x + width
            outcome_y = pos_y + height / (num_outcomes + 1) * (i + 1)
            outcome_pos[key] = (outcome_x, outcome_y)
            self._draw_circle(outcome_x, outcome_y, depth + 0.1, min(5, height/15.0, height/(2*num_outcomes+3)), 15)
            i += 1

        # Put the name of the state in the upper left corner of teh state
        margin = min(width, height) / 8.0
        self._write_string(name, pos_x + margin, pos_y + height - margin, height / 8.0, depth=depth+0.01)

        glPopName()
        return (id, outcome_pos)

    def draw_transition(self, name, from_pos_x, from_pos_y, to_pos_x, to_pos_y, width, waypoints=[], active=False,
                        depth=0):
        """Draw a state with the given properties

        This method is called by the controller to draw the specified transition.
        :param name: Name of the transition
        :param from_pos_x: Starting x position
        :param from_pos_y: Starting y position
        :param to_pos_x: Ending x position
        :param to_pos_y: Ending y position
        :param active: Whether the transition shell be shown as active/selected
        :param depth: The Z layer
        :return: The OpenGL id of the transition
        """
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        self._set_closest_line_width(width)

        # TODO: Show name of the transition

        if active:
            glColor4f(0.7, 0, 0, 0.8)
        else:
            glColor4f(0.6, 0.6, 0.6, 0.8)

        points = [(from_pos_x, from_pos_y)]
        points.extend(waypoints)
        points.append((to_pos_x, to_pos_y))

        # Draw the transitions as simple straight line connecting start- way- and endpoints
        glBegin(GL_LINE_STRIP)
        for point in points:
            glVertex3f(point[0], point[1], depth)
        glEnd()

        self._set_closest_line_width(width / 1.5)
        for waypoint in waypoints:
            self._draw_circle(waypoint[0], waypoint[1], depth + 1, width / 8.)

        glPopName()

        return id

    def _write_string(self, string, pos_x, pos_y, height, line_width=1, depth=0):
        """Write a string

        Writes a string with a simple OpenGL method in the given size at the given position
        :param string: The string to draw
        :param pos_x: x starting position
        :param pos_y: y starting position
        :param height: desired height
        :param line_width: thickness of the letters
        :param depth: the Z layer
        """
        glColor3f(0, 0.5, 0.5)
        self._set_closest_line_width(line_width)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        pos_y = pos_y - height
        glTranslatef(pos_x, pos_y, depth)
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        glScalef(scale_factor, scale_factor, scale_factor)
        for c in string:
            #glTranslatef(0, 0, 0)
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
            #width = glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))

        glPopMatrix()

    def prepare_selection(self, pos_x, pos_y):
        """Prepares the selection rendering

        In order to find out the object being clicked on, the scene has to be rendered again around the clicked position
        :param pos_x: x coordinate
        :param pos_y: y coordinate
        """
        glSelectBuffer(64)
        viewport = glGetInteger(GL_VIEWPORT)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()

        glRenderMode(GL_SELECT)

        glLoadIdentity()
        # The system y axis is inverse to the OpenGL y axis
        gluPickMatrix(pos_x, viewport[3] - pos_y + viewport[1], 3, 3, viewport)

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

    @staticmethod
    def _set_closest_line_width(width):
        """Sets the line width to the closest supported one

        Not all line widths are supported. This function queries both minimum and maximum as well as the step size of
        the line width and calculates the width, which is closest to the given one. This width is then set.
        :param width: The desired line width
        """
        line_width_range = glGetFloatv(GL_LINE_WIDTH_RANGE)
        line_width_granularity = glGetFloatv(GL_LINE_WIDTH_GRANULARITY)

        if width < line_width_range[0]:
            glLineWidth(line_width_range[0])
            return
        if width > line_width_range[1]:
            glLineWidth(line_width_range[1])
            return
        glLineWidth(round(width / line_width_granularity) * line_width_granularity)

    @staticmethod
    def _draw_circle(pos_x, pos_y, depth, radius, segments=10):
        """Draws a circle

        Draws a circle with a line segment a desired position with desired size.
        :param pos_x: Center x position
        :param pos_y: Center y position
        :param depth: The Z layer
        :param radius: Radius of the circle
        :param segments: Number of segments to draw (the more the more exact)
        """
        glBegin(GL_LINE_LOOP)
        segments = max(4, segments)
        for i in range(0, segments):
            angle = 2 * pi / segments * i
            x = pos_x + cos(angle) * radius
            y = pos_y + sin(angle) * radius
            glVertex3f(x, y, depth)
        glEnd()

    def _apply_orthogonal_view(self):
        """Orthogonal view with respect to current aspect ratio
        """
        aspect = self.allocation.width/float(self.allocation.height)

        if self.allocation.width <= self.allocation.height:
            glOrtho(self.left, self.right, self.bottom/aspect, self.top/aspect, -10, 0)
        else:
            glOrtho(self.left*aspect, self.right*aspect, self.bottom, self.top, -10, 0)