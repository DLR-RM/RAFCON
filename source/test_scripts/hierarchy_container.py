def enter(self, scoped_variables, external_modules, gvm):
    print "Hierarchy container: Enter"
    print "Set the varX global variable to 10"
    gvm.set_variable("varX", 10)


def exit(self, scoped_variables, external_modules, gvm):
    print "Hierarchy container: Exit"
