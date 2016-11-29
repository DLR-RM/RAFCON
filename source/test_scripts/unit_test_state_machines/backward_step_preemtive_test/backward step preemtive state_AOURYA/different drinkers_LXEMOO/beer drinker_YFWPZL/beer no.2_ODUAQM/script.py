
def execute(self, inputs, outputs, gvm):
    gvm.set_variable("beers", 2)
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    beers = gvm.get_variable("beers")
    gvm.set_variable("beers", beers -1)
    return 0
