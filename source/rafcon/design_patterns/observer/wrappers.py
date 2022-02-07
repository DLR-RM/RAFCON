class WrapperBase:
    def __init__(self):
        self._models = set()
        self._observers = {}

    @property
    def models(self):
        return self._models

    def add_model(self, model, observable):
        self._models.add((model, observable))

    def remove_model(self, model, observable):
        self._models.remove((model, observable))

    def add_observer(self, instance, observable, notify_before_function=None, notify_after_function=None):
        self._observers[(instance, observable)] = (notify_before_function, notify_after_function)

    def notify_before(self, instance, name, args, kwargs):
        for (observer, observable), (before_function, after_function) in self._observers.items():
            if observable == name and before_function:
                before_function(instance, args)
        for model, observable in self.models:
            model.notify_before(observable, instance, name, args, kwargs)

    def notify_after(self, instance, name, value, args, kwargs):
        for (observer, prop_name), (before_function, after_function) in self._observers.items():
            if prop_name == name and after_function:
                after_function(instance, value, args)
        for model, observable in self.models:
            model.notify_after(observable, instance, name, value, args, kwargs)


class Wrapper(WrapperBase):
    def __init__(self, obj, methods):
        super().__init__()
        self._obj = obj
        self.__class__ = type(self.__class__.__name__, (self.__class__,), {method: Wrapper._wrapper(method) for method in methods})

    @staticmethod
    def _wrapper(name):
        def wrapper(self, *args, **kwargs):
            self.notify_before(self._obj, name, args, kwargs)
            value = getattr(self._obj, name)(*args, **kwargs)
            self.notify_after(self._obj, name, value, args, kwargs)
            return value
        return wrapper

    def __getattr__(self, name):
        return getattr(self._obj, name)


class IterableWrapper(Wrapper):
    def __init__(self, obj, methods):
        Wrapper.__init__(self, obj, methods)
        for method in 'eq ge gt iter le len lt ne'.split():
            method = '__%s__' % method
            setattr(self.__class__, method, getattr(self._obj, method))

    def __getitem__(self, key):
        return self._obj.__getitem__(key)

    def __setitem__(self, key, value):
        self.notify_before(self._obj, '__setitem__', (key, value), {})
        result = self._obj.__setitem__(key, value)
        self.notify_after(self._obj, '__setitem__', result, (key, value), {})
        return result

    def __delitem__(self, key):
        self.notify_before(self._obj, '__delitem__', (key,), {})
        result = self._obj.__delitem__(key)
        self.notify_after(self._obj, '__delitem__', result, (key,), {})
        return result


class TupleWrapper(Wrapper):
    def __init__(self, obj, methods):
        Wrapper.__init__(self, obj, methods)


class ListWrapper(IterableWrapper):
    def __init__(self, obj):
        IterableWrapper.__init__(self, obj, ('append', 'extend', 'insert', 'pop', 'remove', 'reverse', 'sort'))


class SetWrapper(IterableWrapper):
    def __init__(self, obj):
        IterableWrapper.__init__(self, obj, ('add', 'clear', 'discard', 'pop', 'remove'))


class DictWrapper(IterableWrapper):
    def __init__(self, obj):
        IterableWrapper.__init__(self, obj, ('clear', 'pop', 'popitem', 'setdefault', 'update'))
