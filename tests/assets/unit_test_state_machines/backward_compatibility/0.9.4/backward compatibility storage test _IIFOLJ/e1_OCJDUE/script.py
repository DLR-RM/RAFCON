
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world")
    if not gvm.variable_exists(inputs["gvm_variable"]):
        gvm.set_variable(inputs["gvm_variable"], 1)
    else:
        current_value = gvm.get_variable(inputs["gvm_variable"])
        gvm.set_variable(inputs["gvm_variable"], current_value + 1)
    return 0
