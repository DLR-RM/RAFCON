import pytest


import run_tests_in_processes_template


def test_run_unit_tests_common():
    run_tests_in_processes_template.run_unit_tests_in_processes("common")


if __name__ == '__main__':
    # test_run_unit_tests_in_processes()
    pytest.main([__file__])