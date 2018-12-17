# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from gi.repository import GObject

from gtkmvc3.view import View

from gaphas import tool
from gaphas import painter

from rafcon.gui.mygaphas.view import ExtendedGtkView
from rafcon.gui.mygaphas.tools import HoverItemTool, ConnectionCreationTool, ConnectionModificationTool, \
    MoveItemTool, MultiSelectionTool, RightClickTool, MoveHandleTool, ZoomTool, PanTool, ToolChain
from rafcon.gui.mygaphas.painter import HoveredItemPainter

from rafcon.gui.config import global_gui_config
from rafcon.utils import log

logger = log.get_logger(__name__)


class GraphicalEditorView(View, GObject.GObject):

    def __init__(self, selection_m):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        GObject.GObject.__init__(self)
        View.__init__(self)

        self.v_box = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.scroller = Gtk.ScrolledWindow()
        self.scroller.set_name('graphical_editor_scroller')
        self.editor = ExtendedGtkView(self, selection_m)
        self.editor.tool = ToolChain(self.editor). \
            append(HoverItemTool()). \
            append(MoveHandleTool()). \
            append(ConnectionCreationTool()). \
            append(ConnectionModificationTool()). \
            append(PanTool()). \
            append(ZoomTool()). \
            append(MoveItemTool()). \
            append(MultiSelectionTool()). \
            append(RightClickTool())
        self.editor.painter = painter.PainterChain(). \
            append(painter.ItemPainter()). \
            append(HoveredItemPainter()). \
            append(painter.FocusedItemPainter()). \
            append(painter.ToolPainter())
        self.scroller.add(self.editor)
        self.v_box.pack_end(self.scroller, True, True, 0)

        self['main_frame'] = self.v_box
        self.top = 'main_frame'

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(0, 0)


GObject.type_register(GraphicalEditorView)
GObject.signal_new('meta_data_changed', GraphicalEditorView, GObject.SignalFlags.RUN_FIRST, None,
                   (GObject.TYPE_PYOBJECT, GObject.TYPE_STRING, GObject.TYPE_BOOLEAN,))
