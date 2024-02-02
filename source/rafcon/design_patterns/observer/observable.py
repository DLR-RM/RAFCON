from rafcon.design_patterns.observer.wrappers import WrapperBase, ListWrapper, SetWrapper, DictWrapper


OBSERVABLES = '__observables__'
OBSERVABLE_NAME_TEMPLATE = '_observable_%s'
GET_OBSERVABLE_NAME_TEMPLATE = 'get_observable_%s'
SET_OBSERVABLE_NAME_TEMPLATE = 'set_observable_%s'


class ObservableMetaclass(type):
    """
    The Observable Metaclass class that every observable class must use it in order to be known as an observable class.
    """

    @staticmethod
    def add(observable_model):
        """
        Adds the Observable Metaclass to a class
        """

        def wrapper(cls):
            info = cls.__dict__.copy()
            info.pop('__dict__', None)
            return observable_model(cls.__name__, cls.__bases__, info)
        return wrapper

    @staticmethod
    def _get_getter(observable):
        return lambda self: getattr(self, OBSERVABLE_NAME_TEMPLATE % observable)

    @staticmethod
    def _get_setter(observable):
        def wrapper(self, value):
            observable_name = OBSERVABLE_NAME_TEMPLATE % observable
            old_value = getattr(self, observable_name)
            new_value = ObservableMetaclass._create_value(observable, value, self)
            setattr(self, observable_name, new_value)
            if type(old_value) != type(new_value) or (isinstance(old_value, WrapperBase) and old_value != new_value):
                self._reset_notifications(observable, old_value)
            self.notify_assign(observable, old_value, value)
        return wrapper

    @staticmethod
    def _create_value(observable_name, value, model=None):
        result = None
        if isinstance(value, list):
            result = ListWrapper(value)
        elif isinstance(value, set):
            result = SetWrapper(value)
        elif isinstance(value, dict):
            result = DictWrapper(value)
        if result is not None:
            if model:
                result.add_model(model, observable_name)
            return result
        return value

    def __init__(cls, _, bases, info):
        observables = set()
        # Iterates over all observables and creates getter and setter for them
        for observable in set(cls.__dict__.get(OBSERVABLES, set())):
            observable_name = OBSERVABLE_NAME_TEMPLATE % observable
            get_observable_name = GET_OBSERVABLE_NAME_TEMPLATE % observable
            set_observable_name = SET_OBSERVABLE_NAME_TEMPLATE % observable
            setattr(cls, observable_name, ObservableMetaclass._create_value(observable_name, info[observable]))
            setattr(cls, get_observable_name, ObservableMetaclass._get_getter(observable))
            setattr(cls, set_observable_name, ObservableMetaclass._get_setter(observable))
            setattr(cls, observable, property(getattr(cls, get_observable_name), getattr(cls, set_observable_name)))
            observables.add(observable)
        for base in bases:
            observables |= getattr(base, OBSERVABLES, set())
        setattr(cls, OBSERVABLES, observables)


class Observable(WrapperBase):
    """
    The Observable class allows a functions or a method to be observable.
    """

    @staticmethod
    def observed(function):
        """
        Tags a function or a method to be observable
        """

        def wrapper(*args, **kwargs):
            self = args[0]
            try:
                self.notify_before(self, function.__name__, args, kwargs)
                result = function(*args, **kwargs)
                self.notify_after(self, function.__name__, result, args, kwargs)
            except Exception as e:
                self.notify_after(self, function.__name__, e, args, kwargs)
                raise
            return result
        return wrapper

    def __init__(self):
        super().__init__()


class Signal(Observable):
    """
    The Signal class
    """

    def __init__(self):
        super().__init__()

    def emit(self, args=None):
        """
        Emits the signal
        """

        for model, name in self.models:
            model.notify_signal(name, args)
