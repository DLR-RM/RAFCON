import pytest

from rafcon.design_patterns.observer.observer import Observer
from rafcon.design_patterns.observer.observable import Observable, ObservableMetaclass
from rafcon.design_patterns.observer.wrappers import ListWrapper, SetWrapper, DictWrapper

from tests import utils as testing_utils


class ObservableTest(Observable):

    def __init__(self):
        Observable.__init__(self)
        self.__first_var = None
        self.observable_test_var = 0

    @property
    def first_var(self):
        return self.__first_var

    @first_var.setter
    @Observable.observed
    def first_var(self, first_var):
        self.__first_var = first_var

    @Observable.observed
    def complex_method(self, param1, param2, param3):
        print(param3)
        self.observable_test_var = param1 + param2
        return 20


class ObserverTest(object):

    def __init__(self):
        self.test_observable = ObservableTest()
        self.test_observable.add_observer(self, "first_var", self.on_first_var_changed_before,
                                          self.on_first_var_changed_after)
        self.test_observable.add_observer(self, "complex_method",
                                          self.on_complex_method_changed_before,
                                          self.on_complex_method_changed_after)
        self.test_value = 0
        self.test_value2 = 0
        self.test_value3 = 0

    def on_first_var_changed_before(self, observable, args):
        self.test_value = args[1]

    def on_first_var_changed_after(self, observable, return_value, args):
        pass

    def on_complex_method_changed_before(self, observable, args):
        self.test_value2 = args[1] + args[2]

    def on_complex_method_changed_after(self, observable, return_value, args):
        self.test_value3 = return_value + 10


@ObservableMetaclass.add(ObservableMetaclass)
class TestModel(Observer):
    a = 0
    passed = False

    __observables__ = ('a',)

    def __init__(self):
        Observer.__init__(self)


class TestObserver(Observer):
    def __init__(self, model):
        super().__init__(model)
        self.model = model

    @Observer.observe('a', assign=True)
    def b_on_changed(self, _, attribute, info):
        self.model.passed = True


def test_observer(caplog):
    observer_test = ObserverTest()
    observer_test.test_observable.first_var = 20.0
    assert observer_test.test_value == 20

    observer_test.test_observable.complex_method(1, 3, "Hello world")
    assert observer_test.test_observable.observable_test_var == 4
    assert observer_test.test_value2 == 4
    assert observer_test.test_value3 == 30

    test_model = TestModel()
    TestObserver(test_model)

    test_model.a += 1

    assert test_model.passed

    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_wrapper():
    list_feedbacks = []
    list_instance = []
    list_wrapper = ListWrapper(list_instance)
    list_wrapper.notify_before = lambda instance, name, args, kwargs: list_feedbacks.append(name)
    list_wrapper.notify_after = lambda instance, name, value, args, kwargs: list_feedbacks.append(name)

    list_wrapper.append(12)
    list_wrapper.append(1)
    list_wrapper.extend([12])
    list_wrapper.clear()

    assert list_feedbacks.count('append') == 4
    assert list_feedbacks.count('extend') == 2
    assert list_feedbacks.count('clear') == 2

    set_feedbacks = []
    set_instance = set()
    set_wrapper = SetWrapper(set_instance)
    set_wrapper.notify_before = lambda instance, name, args, kwargs: set_feedbacks.append(name)
    set_wrapper.notify_after = lambda instance, name, value, args, kwargs: set_feedbacks.append(name)

    set_wrapper.add(5)
    set_wrapper.add(1)
    set_wrapper.add(3)
    set_wrapper.remove(1)
    set_wrapper.clear()

    assert set_feedbacks.count('add') == 6
    assert set_feedbacks.count('remove') == 2
    assert set_feedbacks.count('clear') == 2

    dict_feedbacks = []
    dict_instance = {}
    dict_wrapper = DictWrapper(dict_instance)
    dict_wrapper.notify_before = lambda instance, name, args, kwargs: dict_feedbacks.append(name)
    dict_wrapper.notify_after = lambda instance, name, value, args, kwargs: dict_feedbacks.append(name)

    dict_wrapper.clear()
    dict_wrapper['test_key'] = 'test_value'

    assert dict_feedbacks.count('clear') == 2
    assert dict_feedbacks.count('__setitem__') == 2


if __name__ == '__main__':
    pytest.main(['-s', __file__])
