

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Inner action: wait forever")
    self.preemptive_wait()
    return

