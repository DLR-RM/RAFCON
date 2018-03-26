from rafcon.core.global_variable_manager import GlobalVariableManager
import pytest
import testing_utils
from pytest import raises


def test_references(caplog):
    gvm = GlobalVariableManager()
    d = {'a': 1, 'b': 2}

    # Test access by reference
    gvm.set_variable('d', d, per_reference=True)
    _d = gvm.get_variable('d', per_reference=True)
    d['a'] = 3
    assert d['a'] == _d['a'] == 3
    __d = gvm.get_variable('d', per_reference=True)
    assert d['a'] == __d['a'] == 3
    ___d = gvm.get_variable('d')
    d['a'] = 4
    assert d['a'] == __d['a'] == ___d['a'] == 4

    # Test set by reference, get by copy
    gvm.set_variable('x', d, per_reference=True)
    cd = gvm.get_variable('x', per_reference=False)
    d['a'] = 5
    assert d['a'] == 5 and cd['a'] == 4

    # Test access by copy
    dc = gvm.get_variable('d', per_reference=False)
    d['b'] = 5
    assert d['a'] == dc['a']
    assert d['b'] != dc['b']

    gvm.set_variable('c', d)
    cc = gvm.get_variable('c')
    d['a'] = 10
    assert d['a'] != cc['a']

    with raises(RuntimeError):
        gvm.get_variable('c', per_reference=True)
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_locks(caplog):
    gvm = GlobalVariableManager()
    gvm.set_variable('a', 1)
    a = gvm.get_variable('a')
    assert a == 1

    access_key = gvm.lock_variable('a')
    gvm.lock_variable('a')
    a = gvm.get_variable('a', access_key=access_key)
    assert a == 1
    gvm.set_variable('a', 2, access_key=access_key)
    assert gvm.get_variable('a', access_key=access_key) == 2
    gvm.unlock_variable('a', access_key)
    gvm.unlock_variable('a', access_key)
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors=2)


def test_type_check(caplog):
    # valid
    gvm = GlobalVariableManager()
    gvm.set_variable("a", 1, data_type=int)
    a = gvm.get_variable("a")
    assert a == 1

    gvm.set_variable("b", "test", data_type=str)
    b = gvm.get_variable("b")
    assert b == "test"

    gvm.set_variable("c", 12.0, data_type=float)
    c = gvm.get_variable("c")
    assert c == 12.0

    gvm.set_variable("d", True, data_type=bool)
    d = gvm.get_variable("d")
    assert d

    e_list = [1, 2, 3]
    gvm.set_variable("e", e_list, data_type=list)
    e = gvm.get_variable("e")
    assert e == e_list

    f_dict = {'a': 1, 'b': 2}
    gvm.set_variable("f", f_dict, data_type=dict)
    f = gvm.get_variable("f")
    assert f == f_dict

    # invalid
    with raises(TypeError):
        gvm.set_variable("g", "test", data_type=int)
    testing_utils.assert_logger_warnings_and_errors(caplog)

    with raises(TypeError):
        gvm.set_variable("g", "test", data_type=float)
    testing_utils.assert_logger_warnings_and_errors(caplog)

    # overwriting
    gvm.set_variable("a", 3, data_type=int)
    a = gvm.get_variable("a")
    assert a == 3

    # invalid overwriting
    with raises(TypeError):
        gvm.set_variable("a", "string", data_type=int)
    a = gvm.get_variable("a")
    assert a == 3

    # invalid overwriting
    with raises(TypeError):
        gvm.set_variable("a", "any_string")
    a = gvm.get_variable("a")
    assert a == 3

    # backward compatibility
    gvm.delete_variable("a")
    gvm.set_variable("a", "test")
    a = gvm.get_variable("a")
    assert a == "test"

    gvm.set_variable("a", 123)
    a = gvm.get_variable("a")
    assert a == 123


if __name__ == '__main__':
    # test_locks(None)
    # test_references(None)
    test_type_check(None)
    # pytest.main([__file__])
