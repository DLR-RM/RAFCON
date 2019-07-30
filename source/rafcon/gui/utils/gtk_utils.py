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


def get_screen_width_and_height():
    """ Returns the width and height of the current screen

    :return:
    """
    display = Gdk.Display.get_default()
    monitor = display.get_primary_monitor()
    geometry = monitor.get_geometry()
    scale_factor = monitor.get_scale_factor()
    screen_width = scale_factor * geometry.width
    screen_height = scale_factor * geometry.height
    return screen_width, screen_height
