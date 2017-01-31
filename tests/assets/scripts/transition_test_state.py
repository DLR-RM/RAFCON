from random import randint

def execute(self, inputs, outputs, gvm):
    print "Executing transition test state %s" % self.name 
    r = randint(3,4)
    print "Outcome: ", r
    return r
