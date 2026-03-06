
def execute(self, inputs, outputs, gvm):
    outputs["output_1"] = inputs["input_1"] + 1
    outputs["scoped_0"] = inputs["scoped_0"] + 1
    return "success"
