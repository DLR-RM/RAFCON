from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.singleton import state_machine_manager_model

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Enable history")
    #gui_config.set_config_value("HISTORY_ENABLED", True)
    sm = self.get_state_machine()
    sm_id = sm.state_machine_id
    sm_model = state_machine_manager_model.state_machines[sm_id]
    if not sm_model.history:
        return 0
    #sm_model.history.fake = False
    sm_model.history.busy = False
    #sm_model.history.observe_model(sm_model)
    #sm_model.history.observe_model(sm_model.root_state)
    return 0
