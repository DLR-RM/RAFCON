
def execute(self, inputs, outputs, gvm):
    self.logger.info('prod_type: %s' % str(inputs['prod_type']))
    if inputs['prod_type'] % 2 == 0:
        self.logger.info('Decided to make prod 1')
        return 0
    else:
        self.logger.info('Decided to make prod 2')
        return 1
