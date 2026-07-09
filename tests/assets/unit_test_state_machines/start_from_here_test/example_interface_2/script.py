
def execute(self, inputs, outputs, gvm):
    outputs["output_1"] = inputs["input_0"] + 1
    return "success"
