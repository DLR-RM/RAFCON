import time

def execute(self, inputs, outputs, gvm):
    gvm.set_variable("beers", 3)
    while True:
        time.sleep(0.010)
        whiskey = gvm.get_variable('whiskey')
        if not whiskey:  # whiskey no.1 not executed yet!
            continue
        if whiskey == 3:  # whiskey no.3 was started
            break
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    beers = gvm.get_variable("beers")
    gvm.set_variable("beers", beers -1)
    return 0