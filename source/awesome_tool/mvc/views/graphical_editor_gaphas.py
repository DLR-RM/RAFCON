from awesome_tool.utils import log
logger = log.get_logger(__name__)

import gtk

from gtkmvc import View

from gaphas import GtkView
from gaphas import tool
from awesome_tool.mvc.views.gap.tools import MyConnectHandleTool


class GraphicalEditorView(View):
    top = 'main_frame'

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        View.__init__(self)

        self.v_box = gtk.VBox()
        self.scroller = gtk.ScrolledWindow()
        self.editor = GtkView()
        self.editor.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#272c36'))
        self.editor.tool = tool.ToolChain(self.editor).\
            append(tool.HoverTool()).\
            append(MyConnectHandleTool()).\
            append(tool.PanTool()).\
            append(tool.ZoomTool()).\
            append(tool.ItemTool()).\
            append(tool.TextEditTool()).\
            append(tool.RubberbandTool())
        self.scroller.add(self.editor)
        self.v_box.pack_end(self.scroller)

        self['main_frame'] = self.v_box

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(500, 500)