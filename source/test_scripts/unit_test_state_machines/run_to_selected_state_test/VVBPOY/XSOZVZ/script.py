def execute(self, inputs, outputs, gvm):
    from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
    import os
    
    gvm.set_variable('tmp_path', RAFCON_TEMP_PATH_BASE + "/test_file")
    self.logger.info(RAFCON_TEMP_PATH_BASE)
    with open(str(gvm.get_variable('tmp_path')), 'w') as f:
        f.write("")
    with open(str(gvm.get_variable('tmp_path')), 'a') as f:
        f.write("'" + self.state_id + " wrote to file!'\n")
    return 0
