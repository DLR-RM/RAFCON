# Copyright (C) 2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>

import threading
from gtkmvc3.view import View

from rafcon.gui import glade
from rafcon.gui.config import global_gui_config
from rafcon.gui.views.logging_console import LoggingConsoleView
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class DebugConsoleView(View):
    builder = glade.get_glade_path("debug_console.glade")
    top = 'console_container'

    def __init__(self):
        View.__init__(self)

        self._lock = threading.Lock()

        ######################################################
        # Logging text view
        ######################################################
        self.logging_console_view = LoggingConsoleView()
        self['console'].pack_start(self.logging_console_view.get_top_widget(), True, True, 0)
        self.logging_console_view.get_top_widget().show()

        ######################################################
        # initial configuration of the console
        ######################################################
        self['button_follow_logging'].set_active(global_gui_config.get_config_value('CONSOLE_FOLLOW_LOGGING', True))
        self['button_show_verbose'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_VERBOSE', True))
        self['button_show_debug'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True))
        self['button_show_info'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_INFO', True))
        self['button_show_warning'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True))
        self['button_show_error'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True))

        self['undock_console_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_UNDOCK))
        self['undock_console_button'].set_tooltip_text("Undock debug console widget")
        self['console_hide_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_DOWNA))

        gui_helper_label.ellipsize_labels_recursively(self['debug_console_button_hbox'])
