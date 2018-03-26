import pytest
from os.path import realpath, dirname, join
import rafcon
import os


def search_for_print_statements_in_python_files(path):
    # subprocess output retrieval somehow does not work with grep commands
    command = "grep -Rn -v '# print' --include '*.py' " + join(dirname(realpath(rafcon.__file__)), path) + \
              "  | grep -v '# .* print' | grep ' print '"
    output = os.popen(command).read()
    # print output
    single_lines = output.split("\n")

    # remove empty line
    if '' in single_lines:
        single_lines.remove('')

    return single_lines


def test_number_of_whitespaces():
    gui_print_lines = search_for_print_statements_in_python_files("gui")
    core_print_lines = search_for_print_statements_in_python_files("core")
    utils_print_lines = search_for_print_statements_in_python_files("utils")
    print "\n".join([str(line) for line in gui_print_lines])
    print len(gui_print_lines)
    assert len(gui_print_lines) == 5
    print "\n".join([str(line) for line in core_print_lines])
    print len(core_print_lines)
    assert len(core_print_lines) == 0
    print "\n".join([str(line) for line in utils_print_lines])
    print len(utils_print_lines)
    assert len(utils_print_lines) == 1


if __name__ == '__main__':
    test_number_of_whitespaces()
    # test_start_script_state()
    # test_start_script_valid_config()
