def execute(self, inputs, outputs, external_modules, gvm):
    print "Hello world: Execute"
    print inputs
    print outputs
    outputs["MyFirstDataOutputPort"] = 10.0
    self.print_state_information()
    print "This is the value of the varX global variable: " + str(gvm.get_variable("varX"))
    return 3
