from collections import Counter

from rafcon.design_patterns.mvc.model import Model
from rafcon.design_patterns.observer.observer import Observer
from rafcon.design_patterns.observer.observable import Observable, Signal


def test_1():
    feedbacks = {}

    class TestObservable(Observable):
        @Observable.observed
        def change(self):
            pass

        @Observable.observed
        def change2(self):
            pass

    class TestModel(Model):
        a = 0
        b = []
        c = set()
        d = TestObservable()
        e = Signal()

        __observables__ = ('a', 'b', 'c', 'd', 'e')

        def __init__(self):
            super().__init__()

    class TestObserver(Observer):
        def __init__(self, model):
            super().__init__(model)

        @Observer.observe('a', assign=True)
        @Observer.observe('b', before=True, after=True)
        @Observer.observe('c', after=True)
        @Observer.observe('d', before=True, after=True)
        @Observer.observe('e', signal=True)
        def b_on_changed(self, _, attribute, info):
            if attribute not in feedbacks:
                feedbacks[attribute] = []
            if 'assign' in info:
                feedbacks[attribute].append('assign')
            elif 'before' in info:
                feedbacks[attribute].append('before')
            elif 'after' in info:
                feedbacks[attribute].append('after')
            elif 'signal' in info:
                feedbacks[attribute].append('signal')

    test_model = TestModel()
    TestObserver(test_model)

    test_model.a = 12
    test_model.a += 1
    test_model.b.append('g')
    test_model.b.remove('g')
    test_model.c.add(1)
    test_model.c.add(2)
    test_model.c.add(3)
    test_model.c.add(4)
    test_model.d.change()
    test_model.d.change2()
    test_model.d.change2()
    test_model.e.emit()
    test_model.e.emit()

    a_feedback = Counter(feedbacks['a'])
    b_feedback = Counter(feedbacks['b'])
    c_feedback = Counter(feedbacks['c'])
    d_feedback = Counter(feedbacks['d'])
    e_feedback = Counter(feedbacks['e'])

    assert a_feedback['assign'] == 2
    assert a_feedback['before'] == 0
    assert a_feedback['after'] == 0
    assert b_feedback['assign'] == 0
    assert b_feedback['before'] == 2
    assert b_feedback['after'] == 2
    assert c_feedback['assign'] == 0
    assert c_feedback['before'] == 0
    assert c_feedback['after'] == 4
    assert d_feedback['before'] == 3
    assert d_feedback['after'] == 3
    assert e_feedback['signal'] == 2
