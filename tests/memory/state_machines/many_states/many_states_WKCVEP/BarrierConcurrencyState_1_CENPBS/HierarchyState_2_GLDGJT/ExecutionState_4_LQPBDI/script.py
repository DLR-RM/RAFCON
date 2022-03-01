import base64
from os.path import dirname, join, abspath

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    
    
    base64string = {}
    state_machine_dir = self.get_state_machine().file_system_path
    with open(join(state_machine_dir, 'image.gif'), 'rb') as imagefile:
        base64string["data"] = base64.b64encode(imagefile.read()).decode('ascii')
    outputs["output_0"] = base64string
    outputs["output_1"] = 34
    outputs["output_2"] = 5.4
    outputs["output_3"] = 'small data!'
    outputs["output_4"] = False
    
    self.preemptive_wait(0.01)
    return "success"
