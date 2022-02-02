import pytest

from rafcon.design_patterns.singleton import Singleton


@Singleton
class TestSingleton:
    def __init__(self):
        pass


def test_1():
    with pytest.raises(TypeError):
        TestSingleton()
    test_singleton = TestSingleton.instance()
    test_singleton2 = TestSingleton.instance()
    assert test_singleton == test_singleton2
