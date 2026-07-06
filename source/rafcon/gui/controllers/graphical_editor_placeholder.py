# Copyright (C) 2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html

"""
.. module:: graphical_editor_placeholder
   :synopsis: Placeholder for the gaphas graphical editor until it is ported to gaphas 5 (Stage 4
     of the GTK4 migration). Lets the application boot and open state machine tabs; the canvas
     area shows a hint instead of the state machine.
"""

from gi.repository import Gtk

from rafcon.design_patterns.mvc.view import View
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log

logger = log.get_logger(__name__)

_PLACEHOLDER_TEXT = ("The graphical state machine editor is not available yet:\n"
                     "rafcon.gui.mygaphas has not been ported to gaphas 5/GTK4 (Stage 4 of the "
                     "GTK4 migration).")


class GraphicalEditorView(View):
    """Placeholder view exposing the 'main_frame' widget used as notebook page"""

    def __init__(self, state_machine_m=None):
        View.__init__(self, parent='main_frame')

        label = Gtk.Label(label=_PLACEHOLDER_TEXT)
        label.set_wrap(True)
        label.set_justify(Gtk.Justification.CENTER)
        label.set_hexpand(True)
        label.set_vexpand(True)

        main_frame = Gtk.Frame()
        main_frame.set_child(label)
        self['main_frame'] = main_frame

        self.editor = None

    def setup_canvas(self, *args, **kwargs):
        pass


class GraphicalEditorController(ExtendedController):
    """Placeholder controller; ignores all canvas related requests"""

    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)
        self.canvas = None
        self.zoom = 1.
        logger.warning("Graphical editor unavailable: mygaphas not yet ported to gaphas 5 (Stage 4); "
                       "showing placeholder canvas.")

    def register_view(self, view):
        super(GraphicalEditorController, self).register_view(view)

    def register_actions(self, shortcut_manager):
        super(GraphicalEditorController, self).register_actions(shortcut_manager)

    def update_item(self, *args, **kwargs):
        pass

    def update_view(self, *args, **kwargs):
        pass

    def miniature_view(self, *args, **kwargs):
        pass
