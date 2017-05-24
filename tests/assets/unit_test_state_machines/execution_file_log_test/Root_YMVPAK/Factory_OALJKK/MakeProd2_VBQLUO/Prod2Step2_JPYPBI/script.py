import numpy as np
import pandas
def execute(self, inputs, outputs, gvm):
    self.logger.info("crafting prod 2 part 2")
    outputs['part2'] = pandas.DataFrame(np.eye(8))
    return 0
