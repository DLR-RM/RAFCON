

from builtins import range
def execute(self, inputs, outputs, gvm):
    import time
    number_of_bugs = inputs["Bugs"]
    for i in range(0, 2):
        self.logger.info("%d little bugs in the code." % number_of_bugs)
    self.logger.info("Take one down, patch it around.")
    time.sleep(1)
    return 0

