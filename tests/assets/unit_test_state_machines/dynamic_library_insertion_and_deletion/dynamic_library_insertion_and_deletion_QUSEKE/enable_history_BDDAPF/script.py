from rafcon.gui.config import global_gui_config as gui_config

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Enable GUI")
    #gui_config.set_config_value("HISTORY_ENABLED", True)
    return 0
