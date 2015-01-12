def execute(self, inputs, outputs, external_modules, gvm):
    print "Hello world: Execute first_state"
    print inputs
    print outputs
    outputs["MyFirstDataOutputPort"] = 10.0
    self.print_state_information()
    return 3
