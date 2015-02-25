def enter(self, scoped_variables, gvm):
    print "library hierarchy state: Enter"
    gvm.set_variable("varX", 10)

def exit(self, scoped_variables, gvm):
    print "library hierarchy state: Exit"
