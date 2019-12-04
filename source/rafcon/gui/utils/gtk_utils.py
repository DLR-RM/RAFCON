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

from gi.repository import Gdk, Gtk


def get_monitor_workarea_at_point(x, y):
    """Returns the workarea (Gdk.Rectangle) for the monitor at the given point"""
    display = Gdk.Display.get_default()
    if Gtk.get_minor_version() >= 22:
        monitor = display.get_monitor_at_point(x, y)
        return monitor.get_workarea()
    screen = display.get_default_screen()
    monitor_number = screen.get_monitor_at_point(x, y)
    return screen.get_monitor_workarea(monitor_number)


def is_point_on_screen(x, y):
    """Checks whether the given coordinate is in the visible area of the screen"""
    workarea = get_monitor_workarea_at_point(x, y)
    return workarea.x <= x <= workarea.x + workarea.width and workarea.y <= y <= workarea.y + workarea.height
