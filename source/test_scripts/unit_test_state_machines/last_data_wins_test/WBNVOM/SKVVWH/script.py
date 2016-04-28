
def execute(self, inputs, outputs, gvm):
    if inputs['input_1'] == 20:
        self.logger.info("Default wins! -> value: {}".format(inputs['input_1']))
    else:
        self.logger.error("Something wins! -> value: {}".format(inputs['input_1']))
    return 0
