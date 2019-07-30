# Copyright (C) 2015-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gdk


def get_screen_width_and_height(window):
    """ Returns the width and height of the current screen
    """
    display = Gdk.Display.get_default()
    screen = display.get_default_screen()
    active_window = screen.get_active_window()
    # cannot use window here because of type error: "Expected Gdk.Window, but got gi.overrides.Gtk.Window"
    # however, can be guaranteed that the active window is the window of interest? => keep the window parameter for now!
    monitor = screen.get_monitor_at_window(active_window)
    monitor_geometry = screen.get_monitor_geometry(monitor)
    screen_width = monitor_geometry.width
    screen_height = monitor_geometry.height
    return screen_width, screen_height

    # old implementation, do not delete!
    # does not work as the primary monitor is not equivalent to the monitor of the window per default
    # display = Gdk.Display.get_default()
    # monitor = display.get_primary_monitor()
    # geometry = monitor.get_geometry()
    # scale_factor = monitor.get_scale_factor()
    # screen_width = scale_factor * geometry.width
    # screen_height = scale_factor * geometry.height
    # return screen_width, screen_height
