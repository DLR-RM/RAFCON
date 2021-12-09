import base64
from os.path import dirname, join, abspath

def execute(self, inputs, outputs, gvm):
    # Loads Huge File and Passes it to loop
    base64string = {}
    state_machine_dir = self.get_state_machine().file_system_path
    
    with open(join(state_machine_dir, 'image.gif'), 'rb') as imagefile:
        base64string["data"] = base64.b64encode(imagefile.read()).decode('ascii')
    outputs["output_0"] = base64string
    outputs["output_1"] = 0
    self.logger.info("Hello {}".format(self.name))
    #self.preemptive_wait(2)
    return "success"
