

def execute(self, inputs, outputs, gvm):
    import time
    time.sleep(0.2)
    if inputs['bottles'] > 0:
        return 1
    return 0

