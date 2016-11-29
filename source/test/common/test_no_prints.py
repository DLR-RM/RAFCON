import pytest
from os.path import realpath, dirname
import rafcon
import os


def search_for_print_statements(path):
    # subprocess output retrieval somehow does not work with grep commands
    command = "grep -R -v '# print' " + str(dirname(realpath(rafcon.__file__))) + path + \
              "  | grep -v '# .* print' | grep ' print '"
    output = os.popen(command).read()
    # print output
    single_lines = output.split("\n")
    return single_lines


def test_number_of_whitespaces():
    mvc_print_lines = search_for_print_statements("/gui")
    state_machine_print_lines = search_for_print_statements("/core")
    utils_print_lines = search_for_print_statements("/utils")
    print len(mvc_print_lines)
    assert len(mvc_print_lines) == 36
    print len(state_machine_print_lines)
    assert len(state_machine_print_lines) == 12
    print len(utils_print_lines)
    assert len(utils_print_lines) == 1


if __name__ == '__main__':
    test_number_of_whitespaces()
    # test_start_script_state()
    # test_start_script_valid_config()