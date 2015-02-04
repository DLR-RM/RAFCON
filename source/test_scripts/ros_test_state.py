def execute(self, inputs, outputs, external_modules, gvm):
    print "Hello world: Execute"
    print inputs
    print outputs
    outputs["MyFirstDataOutputPort"] = 10.0
    self.print_state_information()
    #external_modules["ros"].instance.add_two_ints_client()
    external_modules["ros"].instance.move_turtle("turtle1", 2, 2, 1)
    return 3
