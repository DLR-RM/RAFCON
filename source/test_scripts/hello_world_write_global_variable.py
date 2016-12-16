import threading

def execute(self, inputs, outputs, gvm):
    self.logger.info("Executing performance test hello world sleep state ...")
    # import time
    # time.sleep(0.2)
    print "+++++++++++++++++++++++", id(threading.currentThread()), "+++++++++++++++++++++++"
    print "+++++++++++++++++++++++", threading.current_thread(), "+++++++++++++++++++++++"
    print "+++++++++++++++++++++++", threading.current_thread().name, "+++++++++++++++++++++++"
    if not gvm.variable_exist("test_counter"):
        gvm.set_variable("test_counter", 1)
    else:
        image_id = gvm.get_variable("test_counter")
        gvm.set_variable("test_counter", image_id + 1)
    return 0
