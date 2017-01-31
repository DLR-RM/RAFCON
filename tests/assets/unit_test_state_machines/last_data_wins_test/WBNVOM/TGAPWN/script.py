
def execute(self, inputs, outputs, gvm):
    if inputs['input_1'] > 0:
        self.logger.info("state '{}' wins (take the longest).".format(inputs['input_1']))
    else:
        self.logger.error("wrong output '{}' wins.".format(inputs['input_1']))
    return 0
