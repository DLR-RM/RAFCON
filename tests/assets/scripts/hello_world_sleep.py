def execute(self, inputs, outputs, gvm):
    self.logger.info("Executing performance test hello world sleep state ...")
    import time
    time.sleep(0.2)
    return 0
