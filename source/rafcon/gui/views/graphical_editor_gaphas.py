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

from rafcon.design_patterns.mvc.view import View

from gaphas.painter import PainterChain
from gaphas.tool.rubberband import RubberbandPainter

from rafcon.gui.mygaphas.view import ExtendedGtkView
from rafcon.gui.mygaphas.tools import add_tools_to_view
from rafcon.gui.mygaphas.painter import RAFCONItemPainter, BoundingBoxItemPainter, HoveredItemPainter, GuidePainter
# noinspection PyUnresolvedReferences
from rafcon.gui.mygaphas import guide  # registers the guided handle-in-motion aspects

from rafcon.utils import log

logger = log.get_logger(__name__)


class GraphicalEditorView(View, GObject.GObject):

    def __init__(self, selection_m):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class of the actual editor is the
        gaphas-based ExtendedGtkView.
        """
        GObject.GObject.__init__(self)
        View.__init__(self, parent='main_frame')

        self.v_box = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.scroller = Gtk.ScrolledWindow()
        self.scroller.set_name('graphical_editor_scroller')
        self.scroller.set_hexpand(True)
        self.scroller.set_vexpand(True)
        self.editor = ExtendedGtkView(self, selection_m)

        # Attach all interaction tools (creates editor.rubberband_state as well)
        add_tools_to_view(self.editor)

        self.editor.painter = PainterChain(). \
            append(RAFCONItemPainter(self.editor)). \
            append(HoveredItemPainter(self.editor)). \
            append(GuidePainter(self.editor)). \
            append(RubberbandPainter(self.editor.rubberband_state))
        self.editor.bounding_box_painter = BoundingBoxItemPainter(self.editor)

        self.scroller.set_child(self.editor)
        self.v_box.append(self.scroller)

        self['main_frame'] = self.v_box

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(0, 0)


GObject.type_register(GraphicalEditorView)
GObject.signal_new('meta_data_changed', GraphicalEditorView, GObject.SignalFlags.RUN_FIRST, None,
                   (GObject.TYPE_PYOBJECT, GObject.TYPE_STRING, GObject.TYPE_BOOLEAN,))
