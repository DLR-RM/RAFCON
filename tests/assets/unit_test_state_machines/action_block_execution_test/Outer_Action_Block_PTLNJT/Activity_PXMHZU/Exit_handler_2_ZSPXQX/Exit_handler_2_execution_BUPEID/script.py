

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Exit handler 2")
    gvm.set_variable("exit_handler_2", True)
    return "preempted"

