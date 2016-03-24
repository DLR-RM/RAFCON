

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Inner error handler")
    gvm.set_variable("inner_error_handler", True)
    return "aborted"

