from rafcon.statemachine.global_variable_manager import GlobalVariableManager
import pytest
import test_utils
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
    # test_utils.assert_logger_warnings_and_errors(caplog)


def test_locks(caplog):
    gvm = GlobalVariableManager()
    gvm.set_variable('a', 1)
    a = gvm.get_variable('a')
    assert a == 1

    access_key = gvm.lock_variable('a')
    a = gvm.get_variable('a', access_key=access_key)
    assert a == 1
    gvm.set_variable('a', 2, access_key=access_key)
    assert gvm.get_variable('a', access_key=access_key) == 2
    # test_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_locks(None)
    test_references(None)
    # pytest.main([__file__])