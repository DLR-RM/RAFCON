from __future__ import print_function
def execute(self, inputs, outputs, gvm):
    print("Executing default data port test state ...")
    print(inputs)
    outputs["output_data_port1"] = inputs["input_data_port1"]
    print(outputs)
    return 3
