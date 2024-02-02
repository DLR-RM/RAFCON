

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Error handler")
    gvm.set_variable("error_handler", True)
    return "aborted"

