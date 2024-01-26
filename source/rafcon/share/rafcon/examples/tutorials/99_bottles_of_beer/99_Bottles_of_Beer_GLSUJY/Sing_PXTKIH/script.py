def execute(self, inputs, outputs, gvm):
    import time
    
    bottles = inputs['bottles']
    if bottles is None:
        return -1
        
    self.logger.info(str(bottles) + " bottles of beer on the wall, " + str(bottles) + " bottles of beer.")
    bottles -= 1
    self.logger.info("Take one down and pass it around, " + str(bottles) + " bottles of beer on the wall.")
    time.sleep(1)
    
    return "done"