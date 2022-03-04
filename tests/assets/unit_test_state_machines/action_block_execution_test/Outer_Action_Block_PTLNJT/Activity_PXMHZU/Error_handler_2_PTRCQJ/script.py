

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Error handler 2")
    gvm.set_variable("error_handler_2", True)
    return "aborted"

