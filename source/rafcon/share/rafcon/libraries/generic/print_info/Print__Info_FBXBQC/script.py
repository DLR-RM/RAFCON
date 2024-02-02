
def execute(self, inputs, outputs, gvm):
    message_to_print = inputs['info_message']
    self.logger.info(message_to_print)
    return 0