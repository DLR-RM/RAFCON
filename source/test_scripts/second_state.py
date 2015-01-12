def execute(self, inputs, outputs, external_modules, gvm):
    print "Hello world: Execute second_state"
    print inputs
    print outputs
    outputs["DataOutput1"] = inputs["DataInput1"] + 11.0
    print "DataOutput1 is %s " % str(outputs["DataOutput1"])
    self.print_state_information()
    return 3
