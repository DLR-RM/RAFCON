from awesome_tool.utils import log
logger = log.get_logger(__name__)

import gtk
import gobject

from gtkmvc import View

from gaphas import GtkView
from gaphas import tool
from awesome_tool.mvc.controllers.gap.tools import MyConnectHandleTool, MyHoverTool, MyDeleteTool, MyItemTool


class GraphicalEditorView(View, gobject.GObject):
    top = 'main_frame'

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        gobject.GObject.__init__(self)
        View.__init__(self)

        self.v_box = gtk.VBox()
        self.scroller = gtk.ScrolledWindow()
        self.editor = GtkView()
        self.editor.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#272c36'))
        self.editor.tool = tool.ToolChain(self.editor).\
            append(MyHoverTool()).\
            append(MyConnectHandleTool()).\
            append(tool.PanTool()).\
            append(tool.ZoomTool()).\
            append(MyItemTool(self)).\
            append(tool.RubberbandTool()).\
            append(MyDeleteTool())
        self.scroller.add(self.editor)
        self.v_box.pack_end(self.scroller)

        self['main_frame'] = self.v_box

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(500, 500)

gobject.type_register(GraphicalEditorView)
gobject.signal_new('new_state_selection', GraphicalEditorView, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_PYOBJECT, ))