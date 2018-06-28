# Copyright (C) 2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>

import rafcon.gui.singleton as gui_singletons
from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.models.config_model import ConfigModel
from rafcon.gui.views.debug_console import DebugConsoleView
from rafcon.gui.controllers.logging_console import LoggingConsoleController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log, log_helpers
logger = log.get_logger(__name__)


class DebugConsoleController(ExtendedController):
    """Controller handling the updates and modifications of the debug console widget.

    :param rafcon.gui.models.config_model.ConfigModel: Gui config model holding and observing the global gui config.
    :param rafcon.gui.views.logging_console.DebugConsoleView view: The GTK view showing the logging messages.
    """

    def __init__(self, model, view):
        assert isinstance(model, ConfigModel)
        assert isinstance(view, DebugConsoleView)
        super(DebugConsoleController, self).__init__(model, view)

        self.handler_ids = {}

        ######################################################
        # logging console
        ######################################################
        logging_console_controller = LoggingConsoleController(gui_singletons.gui_config_model,
                                                              view.logging_console_view)
        self.add_controller('logging_console_controller', logging_console_controller)

        view['debug_console_button_hbox'].reorder_child(view['button_show_error'], 0)
        view['debug_console_button_hbox'].reorder_child(view['button_show_warning'], 1)
        view['debug_console_button_hbox'].reorder_child(view['button_show_info'], 2)
        view['debug_console_button_hbox'].reorder_child(view['button_show_debug'], 3)
        view['debug_console_button_hbox'].reorder_child(view['button_show_verbose'], 4)

    def register_view(self, view):
        super(DebugConsoleController, self).register_view(view)

        # Connect Debug console buttons' signals to their corresponding methods
        for level in ["verbose", "debug", "info", "warning", "error"]:
            self.connect_button_to_function("button_show_{}".format(level), "toggled", self.on_log_button_toggled,
                                            "LOGGING_SHOW_{}".format(level.upper()))
        self.connect_button_to_function("button_follow_logging", "toggled", self.on_log_button_toggled,
                                        "CONSOLE_FOLLOW_LOGGING")
        self.update_log_button_state()

    def destroy(self):
        self.view.quit_flag = True
        log_helpers.LoggingViewHandler.remove_logging_view('main')
        super(DebugConsoleController, self).destroy()

    def connect_button_to_function(self, view_index, button_state, function, *args):
        handler_id = self.view[view_index].connect(button_state, function, *args)
        self.handler_ids[view_index] = handler_id

    @ExtendedController.observe('config', after=True)
    def on_config_value_changed(self, config_m, prop_name, info):
        """Callback when a config value has been changed

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'config'
        :param dict info: Information e.g. about the changed config key
        """
        config_key = info['args'][1]
        if "LOGGING" in config_key:
            self.update_log_button_state()

    def on_log_button_toggled(self, log_button, config_key):
        gui_config.set_config_value(config_key, log_button.get_active())

    def update_log_button_state(self):
        for level in ["verbose", "debug", "info", "warning", "error"]:
            active = gui_config.get_config_value("LOGGING_SHOW_{}".format(level.upper()))
            self.view["button_show_{}".format(level)].set_active(active)
        active = gui_config.get_config_value("CONSOLE_FOLLOW_LOGGING")
        self.view["button_follow_logging"].set_active(active)
