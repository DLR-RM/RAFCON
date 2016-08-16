
import pytest
import testing_utils
from rafcon.statemachine.global_variable_manager import GlobalVariableManager
from rafcon.utils import type_helpers
from pytest import raises


def test_type_check(caplog):
    # valid
    gvm = GlobalVariableManager()
    gvm.set_variable("a", 1, "int")
    a = gvm.get_variable("a")
    assert a == 1

    gvm.set_variable("b", "test", "str")
    b = gvm.get_variable("b")
    assert b == "test"

    gvm.set_variable("c", 12.0, "float")
    c = gvm.get_variable("c")
    assert c == 12.0

    # invalid
    with raises(AttributeError):
        gvm.set_variable("d", "test", "int")
    testing_utils.assert_logger_warnings_and_errors(caplog)

    with raises(AttributeError):
        gvm.set_variable("e", "test", "float")
    testing_utils.assert_logger_warnings_and_errors(caplog)

    with raises(ValueError):
        gvm.set_variable("f", 12.0, "string")
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])
    # test_type_check()
