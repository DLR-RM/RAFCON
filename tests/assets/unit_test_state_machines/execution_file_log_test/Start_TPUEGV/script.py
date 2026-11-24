
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Starting factory")
    self.logger.info('updating product type')
    outputs['prod_type'] = inputs['prod_type']+ 1
    if inputs['prod_type'] > 1:
        return 1
    return 0
