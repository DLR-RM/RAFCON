
def execute(self, inputs, outputs, gvm):
    self.logger.debug("add beer")
    outputs['beer_value'] = inputs['beer_value'] + 20
    return 0
    

def backward_execute(self, inputs, outputs, gvm):
    print "Backward execute ", self.name
