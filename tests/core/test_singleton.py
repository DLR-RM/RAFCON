import pytest

from rafcon.design_patterns.singleton import Singleton


@Singleton
class TestSingleton:
    def __init__(self):
        pass


def test_singleton():
    with pytest.raises(TypeError):
        TestSingleton()
    test_singleton_instance = TestSingleton.instance()
    test_singleton_instance2 = TestSingleton.instance()
    assert test_singleton_instance is test_singleton_instance2
