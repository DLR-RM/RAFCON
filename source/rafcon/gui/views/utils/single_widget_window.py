# Copyright

import gtk
from gtkmvc import View

from rafcon.utils import log

logger = log.get_logger(__name__)


class SingleWidgetWindowView(View):
    def __init__(self, view_class, width=500, height=500, title=None, pos=None):
        View.__init__(self)

        w = gtk.Window()
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
        self['main_window'] = w

    pass  # class end
