import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world: p1")
    time.sleep(1.0)
    self.logger.debug("Finished executing: p1")
    return 0

