from rafcon.gui.config import global_gui_config as gui_config

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Disable history")
    gui_config.set_config_value("HISTORY_ENABLED", False)
    return 0
