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
from gtkmvc3.view import View
from builtins import str

from rafcon.utils import log

logger = log.get_logger(__name__)


class SingleWidgetWindowView(View):
    def __init__(self, view_class, width=500, height=500, title=None, pos=None):
        View.__init__(self)

        w = Gtk.Window()
        if title is None:
            w.set_title(str(view_class))
        else:
            w.set_title(title)
        w.resize(width=width, height=height)
        if pos is not None:
            w.set_position(pos)
        self.widget_view = view_class()
        w.add(self.widget_view.get_top_widget())
        w.show_all()

        self['main_frame'] = self.widget_view
        self.top = 'main_frame'
        self['main_window'] = w

    pass  # class end
