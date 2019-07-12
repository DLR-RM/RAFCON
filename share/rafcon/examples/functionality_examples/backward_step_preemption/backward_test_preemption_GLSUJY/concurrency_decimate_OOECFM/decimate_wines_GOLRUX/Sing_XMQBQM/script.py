from __future__ import print_function
# State "Sing"
from builtins import str
def execute(self, inputs, outputs, gvm):
    import time   
 
    bottles = inputs['number']
    bottle_type = inputs['type']
    if bottles is None:
        return -1
 
    self.logger.info(str(bottles) + " bottles of " + str(bottle_type) + " on the wall, " + str(bottles) + " bottles of beer.")
    bottles -= 1
    self.logger.info("Take one down and pass it around, " + str(bottles) + " bottles of beer on the wall.")
    time.sleep(1) 
 
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    print("Backward execution")
    return execute(self, inputs, outputs, gvm)