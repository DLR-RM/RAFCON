
def execute(self, inputs, outputs, gvm):
    if inputs['input_1'] == 0:
        self.logger.info("Last wins! -> value: {}".format(inputs['input_1']))
    elif inputs['input_1'] == 5:
        self.logger.error("Not Last wins! -> value: {}".format(inputs['input_1']))
    else:
        self.logger.error("Something wins! -> value: {}".format(inputs['input_1']))
    return 0
