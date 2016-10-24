def execute(self, inputs, outputs, gvm):
    
    import os
    
    try:
        "RAFCON_TEMP_PATH_BASE" in os.environ
        if not os.environ.get("RAFCON_TEMP_PATH_BASE"):
            os.environ["RAFCON_TEMP_PATH_BASE"] = '/tmp/'
    except KeyError:
        self.logger.error("Enviroment variable path not set")    
    
    gvm.set_variable('tmp_path', os.environ["RAFCON_TEMP_PATH_BASE"] + 'test_file')
    
    with open(str(gvm.get_variable('tmp_path')), 'w') as f:
        f.write("")
    with open(str(gvm.get_variable('tmp_path')), 'a') as f:
        f.write("'" + self.state_id + " wrote to file!'\n")
    return 0
