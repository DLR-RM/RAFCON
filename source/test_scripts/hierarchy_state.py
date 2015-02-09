def enter(self, scoped_variables, external_modules, gvm):
    print "Hierarchy container: Enter"
    print "Set the varX global variable to 10"
    gvm.set_variable("varX", 10)  # do not edit this variable, else you will destroy several test cases
    if "scopeVar1" in scoped_variables:
        scoped_variables["scopeVar1"] = "edited by ENTRY script of hierarchy container"

def exit(self, scoped_variables, external_modules, gvm):
    print "Hierarchy container: Exit"
    if "scopeVar1" in scoped_variables:
        print "value of : scoped_variables[scopeVar1] before editing it: ", scoped_variables["scopeVar1"]
        scoped_variables["scopeVar1"] = "edited by EXIT script of hierarchy container"
