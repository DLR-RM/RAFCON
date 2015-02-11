def enter(self, scoped_variables, external_modules, gvm):
    print "Scoped variable hierarchy state: Enter"
    print "scoped variables: ", scoped_variables
    if "scoped_variable1" in scoped_variables:
        scoped_variables["scoped_variable1"] = 12.0
        print "scoped variables: ", scoped_variables

def exit(self, scoped_variables, external_modules, gvm):
    print "Scoped variable hierarchy state: Enter"
    if "scoped_variable1" in scoped_variables:
        print "value of : scoped_variables[scoped_variable1] before editing it: ", scoped_variables["scoped_variable1"]
