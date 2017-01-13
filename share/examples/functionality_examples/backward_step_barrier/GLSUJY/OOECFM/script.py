

def enter(self, scoped_variables, gvm):
    pass

def exit(self, scoped_variables, gvm):
    pass

def backward_enter(self, scoped_variables, gvm):
    print "Backward enter ", self.name

def backward_exit(self, scoped_variables, gvm):
    print "Backward exit ", self.name