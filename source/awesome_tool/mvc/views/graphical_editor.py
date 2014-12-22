
from utils import log
logger = log.get_logger(__name__)

from math import sin, cos, pi
from random import random

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
from gtkmvc import View


class GraphicalEditorView(View):

    def __init__(self):
        View.__init__(self)

        # Configure OpenGL framebuffer.
        # Try to get a double-buffered framebuffer configuration,
        # if not successful then exit program
        display_mode = (gtk.gdkgl.MODE_RGB | gtk.gdkgl.MODE_DEPTH | gtk.gdkgl.MODE_SINGLE)
        try:
            glconfig = gtk.gdkgl.Config(mode=display_mode)
        except gtk.gdkgl.NoMatches:
            raise SystemExit

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
        gtk.DrawingArea.__init__(self)

        self.left = 0
        self.right = 100
        self.top = 100
        self.bottom = 0

        self.name_counter = 0

        # Set OpenGL-capability to the drawing area
        self.set_gl_capability(glconfig)

        glutInit([])

        # Connect the relevant signals.
        self.connect_after('realize',   self._realize)
        self.connect('configure_event', self._configure)
        #self.connect('expose_event',    self.expose)


    def _realize(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glEnable(GL_LINE_SMOOTH)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(33./255, 49./255, 92./255, 1)
        #glClearColor(0, 1, 0, 1)

        logger.debug("realize")

        #self.configure()

    def _configure(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()
        #glClearColor(0, 1, 0, 1)
        #logger.debug("configure")
        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        glViewport(0, 0, self.allocation.width, self.allocation.height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        self._apply_ortogonal_view()

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # OpenGL end
        gldrawable.gl_end()

        return False

    def _apply_ortogonal_view(self):
        aspect = self.allocation.width/float(self.allocation.height)

        if self.allocation.width <= self.allocation.height:
            glOrtho(self.left, self.right, self.bottom/aspect, self.top/aspect, -10, 0)
        else:
            glOrtho(self.left*aspect, self.right*aspect, self.bottom, self.top, -10, 0)

    def pixel_to_size_ratio(self):
        width = self.right - self.left
        if self.allocation.width > self.allocation.height:
            width *= self.allocation.width/float(self.allocation.height)
        display_width = self.allocation.width
        return display_width / float(width)

    def expose_init(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        #logger.debug("expose_init")

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glInitNames()
        glPushName(0)
        self.name_counter = 1

        #glColor4f(1, 0, 0, 0.5)
        #glRectf(-25, 25, 25, -25)


        return False

    def expose_finish(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        if gldrawable.is_double_buffered():
            gldrawable.swap_buffers()
        else:
            glFlush()

        # OpenGL end
        gldrawable.gl_end()

        #logger.debug("expose_finish")

    def draw_state(self, name, pos_x, pos_y, width, height, outcomes=0, active=False, depth=0):
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1
        glPushName(id)
        #glRectf(pos_x, pos_y, pos_x + width, pos_y + height)
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
            if key == 1:
                glColor3f(0.8, 0, 0)
            elif key == 2:
                glColor3f(0.1, 0.1, 0.7)
            else:
                glColor3f(0.4, 0.4, 0.4)

            outcome_x = pos_x + width
            outcome_y = pos_y + height / (num_outcomes + 1) * (i + 1)
            outcome_pos[key] = (outcome_x, outcome_y)
            i += 1
            self._draw_circle(outcome_x, outcome_y, depth + 0.1, min(5, height/15.0, height/(2*num_outcomes+3)), 15)


        margin = min(width, height) / 8.0
        self._write_string(name, pos_x + margin, pos_y + height - margin, height / 8.0, depth=depth+0.01)

        glPopName()
        return (id, outcome_pos)

    def draw_transition(self, name, from_pos_x, from_pos_y, to_pos_x, to_pos_y, active=False, depth=0):
        # "Generate" unique ID for each object
        id = self.name_counter
        self.name_counter += 1

        glPushName(id)
        self._set_closest_line_width(4)

        if active:
            glColor4f(0.7, 0, 0, 0.8)
        else:
            glColor4f(0.6, 0.6, 0.6, 0.8)


        glBegin(GL_LINES)
        glVertex3f(from_pos_x, from_pos_y, depth)
        glVertex3f(to_pos_x, to_pos_y, depth)
        glEnd()
        glPopName()

        return id

    def _write_string(self, string, pos_x, pos_y, height, line_width=1, depth=0):
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
        glSelectBuffer(64)
        viewport = glGetInteger(GL_VIEWPORT)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()

        glRenderMode(GL_SELECT)

        glLoadIdentity()
        # The system y axis is inverse to the OpenGL y axis
        gluPickMatrix(pos_x, viewport[3] - pos_y + viewport[1], 3, 3, viewport)

        self._apply_ortogonal_view()

    @staticmethod
    def find_selection():
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
        glBegin(GL_LINE_LOOP)
        segments = max(4, segments)
        for i in range(0, segments):
            angle = 2 * pi / segments * i
            x = pos_x + cos(angle) * radius
            y = pos_y + sin(angle) * radius
            glVertex3f(x, y, depth)
        glEnd()