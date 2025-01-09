import pytest
import time
import sys
import os


@pytest.mark.timeout(300)
@pytest.mark.skipif(sys.version_info < (3, 4), reason="requires python3.4 or higher")
def test_memory(test_iterations=15):
    """
    RAFCON memory test launcher

    Parameters:
    test_iterations (int): Number of times every test state machine is to be run

   """
    from tests.memory.memory_test_helper import MemoryTestHelper

    if "Memory_Test_Iterations" in os.environ:
        test_iterations = int(os.environ["Memory_Test_Iterations"])

    # Specify machines to be tested
    machines_to_be_tested = ["sequential", "concurrent", "big_data"]
    # machines_to_be_tested = ["big_data"]

    # Initialize Variables
    tests = []

    test_successful = False
    # Create test objects and run tests
    for k in range(len(machines_to_be_tested)):
        tests.append(MemoryTestHelper(machines_to_be_tested[k], test_iterations))
        results = tests[k].run(assert_during_execution=True)


if __name__ == '__main__':
    test_memory()

