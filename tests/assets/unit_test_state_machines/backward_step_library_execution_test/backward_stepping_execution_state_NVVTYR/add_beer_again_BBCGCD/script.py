
def execute(self, inputs, outputs, gvm):
    self.logger.debug("add beer again")
    outputs['beer_value'] = inputs['beer_value'] + 40
    return 0
    
    
def backward_execute(self, inputs, outputs, gvm):
    print "Backward execute ", self.name
