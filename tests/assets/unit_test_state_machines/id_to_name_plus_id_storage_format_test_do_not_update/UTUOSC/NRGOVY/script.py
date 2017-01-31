from testing_utils import RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE
FILE_TO_MODIFY = RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE + "/test_start_script.txt"


def execute(self, inputs, outputs, gvm):
    self.logger.debug("enter state_1 -> write into: " + FILE_TO_MODIFY)
    
    with open(FILE_TO_MODIFY, "w+") as f:
        f.write("start, ")
    
    return 0
