from rafcon.core.global_variable_manager import GlobalVariableManager
import pytest
from tests import utils as testing_utils
from pytest import raises


def test_references(caplog):
    gvm = GlobalVariableManager.instance()
    gvm.reset()
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
    gvm = GlobalVariableManager.instance()
    gvm.reset()
    gvm.set_variable('a', 1)
    a = gvm.get_variable('a')
    assert a == 1

    assert (not gvm.is_locked("a"))

    access_key = gvm.lock_variable('a')
    gvm.lock_variable('a')
    assert (gvm.is_locked("a"))
    a = gvm.get_variable('a', access_key=access_key)
    assert a == 1
    gvm.set_variable('a', 2, access_key=access_key)
    assert gvm.get_variable('a', access_key=access_key) == 2

    gvm.set_locked_variable('a', access_key=access_key, value=4)
    assert gvm.get_locked_variable('a', access_key=access_key) == 4

    gvm.unlock_variable('a', access_key)
    gvm.unlock_variable('a', access_key)
    assert (not gvm.is_locked("a"))
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_warnings=1, expected_errors=1)


def test_type_check(caplog):
    # valid
    gvm = GlobalVariableManager.instance()
    gvm.reset()
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

    # If variable doesnt exist
    var = gvm.get_variable("Fakevar", default=-1)
    assert (var == -1)

    # If variable exist
    assert (gvm.data_type_exist("a"))


def test_keys(caplog):
    gvm = GlobalVariableManager.instance()
    gvm.reset()
    gvm.set_variable('a', 1)
    gvm.set_variable('ab', 2)
    gvm.set_variable('abc', 3)
    gvm.set_variable('def', 4)

    key_start_a = gvm.get_all_keys_starting_with('a')
    assert (len(gvm.get_all_keys()) == 4)
    assert (len(key_start_a) == 3)

    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_locks(None)
    test_references(None)
    test_type_check(None)
    pytest.main([__file__])
