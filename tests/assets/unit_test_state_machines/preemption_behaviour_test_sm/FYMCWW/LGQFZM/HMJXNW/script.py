import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world: s1")
    for i in range(10):
        time.sleep(0.5)
        if self.preempted:
           break
    return 0

