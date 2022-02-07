from rafcon.design_patterns.mvc.model import Model
from rafcon.design_patterns.observer.observer import Observer
from rafcon.design_patterns.observer.observable import Observable
from tests import utils as testing_utils
import pytest


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


class TestModel(Model):
    a = 0
    passed = False

    __observables__ = ('a',)

    def __init__(self):
        super().__init__()


class TestObserver(Observer):
    def __init__(self, model):
        super().__init__(model)
        self.model = model

    @Observer.observe('a', assign=True)
    def b_on_changed(self, _, attribute, info):
        self.model.passed = True


def test_observer(caplog):
    testing_utils.dummy_gui(None)
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


if __name__ == '__main__':
    pytest.main(['-s', __file__])
