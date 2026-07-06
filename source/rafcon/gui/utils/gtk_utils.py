# Copyright (C) 2015-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>

from gi.repository import Gdk


def iter_children(widget):
    """Iterates over the direct children of a widget

    GTK4 removed Gtk.Container and with it get_children(); children are traversed via the
    first-child/next-sibling chain instead.
    """
    child = widget.get_first_child()
    while child is not None:
        yield child
        child = child.get_next_sibling()


def set_all_margins(widget, margin):
    """Sets the same margin on all four sides of a widget

    Replacement for the GTK3 Gtk.Container.set_border_width(), which GTK4 removed.
    """
    widget.set_margin_start(margin)
    widget.set_margin_end(margin)
    widget.set_margin_top(margin)
    widget.set_margin_bottom(margin)


def is_point_on_screen(x, y):
    """Checks whether the given coordinate is in the visible area of any monitor

    GTK4 removed the point-to-monitor and workarea queries (meaningless under Wayland), so this
    checks the plain monitor geometries.
    """
    display = Gdk.Display.get_default()
    if display is None:
        return False
    monitors = display.get_monitors()
    for index in range(monitors.get_n_items()):
        geometry = monitors.get_item(index).get_geometry()
        if geometry.x <= x <= geometry.x + geometry.width and geometry.y <= y <= geometry.y + geometry.height:
            return True
    return False
