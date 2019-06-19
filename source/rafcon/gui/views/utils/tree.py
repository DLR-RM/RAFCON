# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

from gi.repository import Gtk
from gi.repository import Pango

from gtkmvc3.view import View


class TreeView(View):
    builder = None
    top = None

    def __init__(self):
        super(TreeView, self).__init__()
        self.scrollbar_widget = None

        for column in self.get_top_widget().get_columns():
            self.make_column_title_elllipsable(column)

    @staticmethod
    def make_column_title_elllipsable(column, title=None, tooltip=None):
        """Replaces the column title widget with a similar title widget allowing to be ellipsized

        :param Gtk.TreeViewColumn column: The column to be modified
        :param str title: Optional new title, if not set, the current title will be used
        :param str tooltip: Optional tooltip, if not set, no tooltip will be set
        """
        title = title or column.get_title()
        label = Gtk.Label(label=title)
        if tooltip:
            label.set_tooltip_text(tooltip)
        label.set_ellipsize(Pango.EllipsizeMode.END)
        label.set_width_chars(1)
        label.show()
        column.set_widget(label)
        column.set_expand(True)
