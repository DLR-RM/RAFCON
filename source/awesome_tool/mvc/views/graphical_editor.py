
from utils import log
logger = log.get_logger(__name__)

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

        self.win = gtk.Window()
        self.win.set_title("Graphical Editor")
        self.v_box = gtk.VBox()
        self.test_label = gtk.Label("Hallo")
        self.editor = GraphicalEditor(glconfig)
        self.editor.set_size_request(200, 200)

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
        logger.debug("configure")
        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        glViewport(0, 0, self.allocation.width, self.allocation.height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = self.allocation.width/float(self.allocation.height)

        if self.allocation.width <= self.allocation.height:
            glOrtho(-100, 100, -100/aspect, 100/aspect, 1, -1)
        else:
            glOrtho(-100*aspect, 100*aspect, -100, 100, 1, -1)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # OpenGL end
        gldrawable.gl_end()

        return False

    def expose_init(self, *args):
        # Obtain a reference to the OpenGL drawable
        # and rendering context.
        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        # OpenGL begin
        if not gldrawable.gl_begin(glcontext):
            return False

        logger.debug("expose_init")

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

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

        logger.debug("expose_finish")

    def draw_container(self, name, width, height, activated=False):
        if activated:
            glColor3f(1, 1, 1)
        else:
            glColor3f(0.9, 0.9, 0.9)

        glRectf(0, 0, width, height)

        self._write_string(name, 10, height - 10, 20)


    def _write_string(self, string, pos_x, pos_y, height):
        glColor3f(0, 0.5, 0.5)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        pos_y = pos_y - height
        glTranslatef(pos_x, pos_y, 0)
        font_height = 119.5  # According to https://www.opengl.org/resources/libraries/glut/spec3/node78.html
        scale_factor = height / font_height
        glScalef(scale_factor, scale_factor, scale_factor)
        for c in string:
            #glTranslatef(0, 0, 0)
            glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
            width = glutStrokeWidth(GLUT_STROKE_ROMAN, ord(c))

        glPopMatrix()