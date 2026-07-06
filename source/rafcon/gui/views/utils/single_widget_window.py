# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View

from rafcon.utils import log

logger = log.get_logger(__name__)


class SingleWidgetWindowView(View):
    def __init__(self, view_class, width=500, height=500, title=None, pos=None):
        super().__init__(parent='main_frame')

        w = Gtk.Window()
        if title is None:
            w.set_title(str(view_class))
        else:
            w.set_title(title)
        w.set_default_size(width, height)
        # GTK4 removed programmatic window positioning; 'pos' is ignored
        self.widget_view = view_class()
        w.set_child(self.widget_view.get_parent_widget())
        w.present()

        self['main_frame'] = self.widget_view
        self.top = 'main_frame'
        self['main_window'] = w
