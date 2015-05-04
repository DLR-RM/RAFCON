

def execute(self, inputs, outputs, gvm):
    self.preemptive_wait(inputs['time'])
    return 0

