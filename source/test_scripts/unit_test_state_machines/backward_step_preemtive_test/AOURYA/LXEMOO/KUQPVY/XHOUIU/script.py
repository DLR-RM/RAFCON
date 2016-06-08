
def execute(self, inputs, outputs, gvm):
    whiskey = gvm.get_variable("whiskey")
    gvm.set_variable("whiskey", whiskey +1)
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    whiskey = gvm.get_variable("whiskey")
    gvm.set_variable("whiskey", whiskey -1)
    return 0