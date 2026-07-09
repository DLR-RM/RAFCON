
def execute(self, inputs, outputs, gvm):
    outputs["output_2"] = "some manual output"
    outputs["output_3"] = [0, 1, 2, 3, 4]
    outputs["output_4"] = 7
    return "success"
