def enter(self, scoped_variables, external_modules, gvm):
    print "library hierarchy state: Enter"
    gvm.set_variable("varX", 10)

def exit(self, scoped_variables, external_modules, gvm):
    print "library hierarchy state: Exit"
