import inspect


from rafcon.design_patterns.observer.observable import Signal, OBSERVABLES, OBSERVABLE_NAME_TEMPLATE
from rafcon.design_patterns.observer.wrappers import WrapperBase


class NotifyInfo(dict):
    """
    The NotifyInfo class holds the information of a specific notification.
    """

    def __init__(self, flag, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)
        for flag_ in ['assign', 'before', 'after', 'signal']:
            if flag_ != flag and flag_ in self:
                del self[flag_]

    def __getattr__(self, name):
        return self[name]


class Observer:
    """
    The Observer class can observe the observable attributes and functions (or methods) and calls the defined callbacks
    when the observables change.
    """

    @classmethod
    def observe(cls, *args, **kwargs):
        """
        Observes the observable
        """

        def wrapper(function):
            setattr(function, OBSERVABLES, getattr(function, OBSERVABLES, list()) + [(args[0], kwargs)])
            return function
        return wrapper

    def __init__(self, model=None):
        self._observable_to_methods = {}
        # Iterates over all observe callbacks and add them to 'observable_to_methods'
        for cls in inspect.getmro(type(self)):
            for name, method in cls.__dict__.items():
                if inspect.isfunction(method) and hasattr(method, OBSERVABLES):
                    for observable, kwargs in getattr(method, OBSERVABLES):
                        if observable not in self._observable_to_methods:
                            self._observable_to_methods[observable] = []
                        self._observable_to_methods[observable].append((getattr(self, name), kwargs))
        self._observers = []
        if model:
            self.observe_model(model)
        self._notifications = {'assign': {}, 'before': {}, 'after': {}, 'signal': {}}
        self._assign_notifications = self._notifications['assign']
        self._before_notifications = self._notifications['before']
        self._after_notifications = self._notifications['after']
        self._signal_notifications = self._notifications['signal']
        for observable in self.get_observables():
            self._register_observable(observable)

    @property
    def observable_to_methods(self):
        return self._observable_to_methods

    @observable_to_methods.setter
    def observable_to_methods(self, value):
        self._observable_to_methods = value

    def get_observables(self):
        """
        Returns the observables
        """

        return getattr(self, OBSERVABLES, set())

    def observe_model(self, model):
        """
        Observes a model
        """

        return model.register_observer(self)

    def relieve_model(self, model):
        """
        Relieves a model
        """

        return model.unregister_observer(self)

    def register_observer(self, observer):
        """
        Registers an observer
        """

        if observer in self._observers:
            return
        for observable in self.get_observables():
            self._add_notification(observer, observable)
        self._observers.append(observer)

    def unregister_observer(self, observer):
        """
        Unregisters an observer
        """

        if observer not in self._observers:
            return
        for observable in self.get_observables():
            self._remove_notification(observer, observable)
        self._observers.remove(observer)

    def notify_observer(self, observer, method, *args, **kwargs):
        """
        Notifies an observer
        """

        return method(*args, **kwargs)

    def notify_assign(self, observable, old_value, new_value):
        """
        Sends the 'assign' notification
        """

        for method, kwargs in self._assign_notifications[observable][:]:
            if old_value != new_value:
                info = NotifyInfo('assign', kwargs, model=self, prop_name=observable, old=old_value, new=new_value)
                self.notify_observer(method.__self__, method, self, observable, info)

    def notify_before(self, observable, instance, method_name, args, kwargs):
        """
        Sends the 'before' notification
        """

        for method, kwargs_ in self._before_notifications[observable][:]:
            info = NotifyInfo('before',
                              kwargs_,
                              model=self,
                              prop_name=observable,
                              instance=instance,
                              method_name=method_name,
                              args=args,
                              kwargs=kwargs)
            self.notify_observer(method.__self__, method, self, observable, info)

    def notify_after(self, observable, instance, method_name, result, args, kwargs):
        """
        Sends the 'after' notification
        """

        for method, kwargs_ in self._after_notifications[observable][:]:
            info = NotifyInfo('after',
                              kwargs_,
                              model=self,
                              prop_name=observable,
                              instance=instance,
                              method_name=method_name,
                              result=result,
                              args=args,
                              kwargs=kwargs)
            self.notify_observer(method.__self__, method, self, observable, info)

    def notify_signal(self, observable, args):
        """
        Sends the 'signal' notification
        """

        for method, kwargs in self._signal_notifications[observable][:]:
            info = NotifyInfo('signal', kwargs, model=self, prop_name=observable, arg=args)
            self.notify_observer(method.__self__, method, self, observable, info)

    def _register_observable(self, observable):
        if observable not in self._assign_notifications:
            self._assign_notifications[observable] = []
        value = getattr(self, OBSERVABLE_NAME_TEMPLATE % observable, None)
        if isinstance(value, WrapperBase):
            value.add_model(self, observable)
            if isinstance(value, Signal):
                if observable not in self._signal_notifications:
                    self._signal_notifications[observable] = []
            else:
                if observable not in self._before_notifications:
                    self._before_notifications[observable] = []
                if observable not in self._after_notifications:
                    self._after_notifications[observable] = []

    def _add_notification(self, observer, observable):
        value = getattr(self, OBSERVABLE_NAME_TEMPLATE % observable, None)
        for method, kwargs in observer.observable_to_methods.get(observable, set()):
            pair = (method, kwargs)
            if 'assign' in kwargs and pair not in self._assign_notifications[observable]:
                self._assign_notifications[observable].append(pair)
            if 'before' in kwargs and isinstance(value, WrapperBase) and pair not in self._before_notifications[observable]:
                self._before_notifications[observable].append(pair)
            if 'after' in kwargs and isinstance(value, WrapperBase) and pair not in self._after_notifications[observable]:
                self._after_notifications[observable].append(pair)
            if 'signal' in kwargs and isinstance(value, Signal) and pair not in self._signal_notifications[observable]:
                self._signal_notifications[observable].append(pair)

    def _remove_notification(self, observer, observable):
        for notification in self._notifications.values():
            sequences = notification.get(observable, ())
            for method, kwargs in reversed(sequences):
                if method.__self__ == observer:
                    sequences.remove((method, kwargs))

    def _reset_notifications(self, observable, value):
        if isinstance(value, WrapperBase):
            value.remove_model(self, observable)
        self._register_observable(observable)
        for observer in self._observers:
            self._remove_notification(observer, observable)
            self._add_notification(observer, observable)
