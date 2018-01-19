import os
import sys
import pytest
import traceback
from multiprocessing import Process, Queue


def run_pytest_on_module(filename, unit_test_message_queue):
    return_value = -1
    try:
        return_value = pytest.main(" -x -s --verbose " + filename)
    except Exception as e:
        sys.stderr.write(
            "Error: Unit test {0} failed with an uncaught exception: {1}\nTraceback: {2}".format(
                filename, e, str(traceback.format_exc()))
        )
    print "return value of unit test " + filename + ": " + str(return_value)
    # successful processes return 0
    if return_value != 0:
        sys.stderr.write("Error: Unit test {0} failed\nTraceback: {1}".format(
            filename, str(traceback.format_exc()))
        )
    unit_test_message_queue.put([filename, return_value])


def run_unit_tests_in_processes(path_with_test_modules):
    """
    This is the actual unit test
    :return:
    """
    # my_path, my_file = os.path.split(__file__)
    test_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    unit_test_message_queue = Queue()
    process_list = []

    def run_tests_in_path(path):
        print os.listdir(path)
        for file_name in os.listdir(path):
            abs_path = os.path.join(path, file_name)
            if os.path.isfile(abs_path) and file_name.startswith("test_") and file_name.endswith(".py"):
                # Run test in new process
                test_process = Process(target=run_pytest_on_module, args=(abs_path, unit_test_message_queue))
                process_list.append(test_process)
                test_process.start()
                print "Started unit test: " + abs_path
                # as some tests are not completely runtime independent, they are executed sequentially
                test_process.join()
                return_value = unit_test_message_queue.get()
                assert return_value[1] == 0

            elif os.path.isdir(abs_path) and file_name != "all":
                # Recursively go through all directories to search for tests
                run_tests_in_path(abs_path)

    run_tests_in_path(os.path.join(test_path, path_with_test_modules))

    # for process in process_list:
    #     return_value = unit_test_message_queue.get()
    #     # this assert checks if all unit tests executed successfully
    #     assert return_value[1] == 0
    #     process.join()

if __name__ == '__main__':
    # run_unit_tests_in_processes("network")
    run_unit_tests_in_processes("gui")
