
def execute(self, inputs, outputs, gvm):
    gvm.set_variable("ros_initialized", True)
    return "success"
