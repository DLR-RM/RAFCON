

def execute(self, inputs, outputs, gvm):
    from time import sleep, time
    self.logger.info("Start")
    start_time = time()
    wait_time = gvm.get_variable(self.name + '_wait')
    preempted = self.preemptive_wait(wait_time)
    wait_time = time() - start_time
    gvm.set_variable(self.name + '_wait_time', wait_time)
    gvm.set_variable(self.name + '_preempted', preempted)
    self.logger.info("waited " + str(time()-start_time) + " sec - Preempted: " + str(preempted))
    return 3

