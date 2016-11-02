import time

def execute(self, inputs, outputs, gvm):
    while True: 
        time.sleep(0.010)   
        beer = gvm.get_variable('beers')
        if beer >= 2:
            break
    gvm.set_variable("whiskey",2)
    return 0

def backward_execute(self, inputs, outputs, gvm):
    whiskey = gvm.get_variable("whiskey")
    gvm.set_variable("whiskey", whiskey -1)
    return 0