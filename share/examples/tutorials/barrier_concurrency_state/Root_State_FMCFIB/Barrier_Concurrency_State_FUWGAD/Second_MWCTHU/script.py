import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world2")
    time.sleep(2.0)
    number = 1/0 # create an error here that can be handled in the decider state
    return 0