
def execute(self, inputs, outputs, gvm):
    self.logger.debug("enter state_2")
    
    test_file = open("/tmp/rafcon_unit_tests/test_start_script.txt", "a")
    test_file.write("state, ")
    test_file.close()
    
    return 0
