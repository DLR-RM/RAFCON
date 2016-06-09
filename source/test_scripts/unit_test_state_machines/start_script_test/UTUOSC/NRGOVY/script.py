
def execute(self, inputs, outputs, gvm):
    self.logger.debug("enter state_1")
    
    test_file = open("/tmp/rafcon_unit_tests/test_start_script.txt", "w+")
    test_file.write("start, ")
    test_file.close()
    
    return 0