import os
import imp
import sys
import time
import pytest
from multiprocessing import Process, Queue
from Queue import Empty


def run_pytest_on_module(filename, unit_test_message_queue):
    return_value = -1
    try:
        return_value = pytest.main(" -x -v " + filename)
    except Exception as e:
        print "Unit test " + filename + " failed and execption was caught!" + str(e)
    print "return value of unit test " + filename + ": " + str(return_value)
    # successful processes return 0
    if return_value != 0:
        sys.stderr.write("Error: Unit test " + filename + " failed and exception was caught!\n")
    unit_test_message_queue.put([filename, return_value])


def test_run_unit_tests_in_processes():
    """
    This is the actual unit test
    :return:
    """
    my_path, my_file = os.path.split(__file__)
    test_path = os.path.abspath(os.path.join(my_path, '..'))

    unit_test_message_queue = Queue()
    process_list = []

    #####################################################
    # execute all common tests
    #####################################################
    common_path = test_path + "/common"
    for file in os.listdir(common_path):
        if file == "test_all.py":
            continue
        if file.endswith(".pyc"):
            continue
        if file.startswith("."):
            continue
        test_process = Process(target=run_pytest_on_module, args=("common/"+file, unit_test_message_queue))
        process_list.append(test_process)
        test_process.start()
        # test_process.join()

    #####################################################
    # execute all network tests
    #####################################################
    network_path = test_path + "/network"
    for file in os.listdir(network_path):
        if file == "test_all_server.py":
            continue
        if file.endswith(".pyc"):
            continue
        if file.endswith(".yaml"):
            continue
        if file.startswith("."):
            continue
        test_process = Process(target=run_pytest_on_module, args=("network/"+file, unit_test_message_queue))
        process_list.append(test_process)
        test_process.start()
        # test_process.join()

    for process in process_list:
        return_value = unit_test_message_queue.get()
        # this assert checks if all unit tests executed successfully
        assert return_value[1] == 0
        process.join()

if __name__ == '__main__':
    # test_run_unit_tests_in_processes()
    pytest.main([__file__])
