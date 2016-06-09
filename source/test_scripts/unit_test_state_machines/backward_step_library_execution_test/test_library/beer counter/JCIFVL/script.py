
def execute(self, inputs, outputs, gvm):
    self.logger.debug("count beers")
    outputs['counted beers'] = inputs['beers']
    return 0
    
    
def backward_execute(self, inputs, outputs, gvm):
    print "Backward execute ", self.name
