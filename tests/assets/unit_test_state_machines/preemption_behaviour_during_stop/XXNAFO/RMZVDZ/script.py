import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world: s1")
    gvm.set_variable("s1", 1)
    while True:
        time.sleep(0.5)
        if self.preempted:
           return 0
    gvm.set_variable("s1", 0)
    return 0
