import threading

from gi.repository import GLib

from rafcon.design_patterns.observer.observable import ObservableModel
from rafcon.design_patterns.observer.observer import Observer


@ObservableModel.add(ObservableModel)
class Model(Observer):
    """
    Model class of the MVC pattern. It holds the references to the data. The data can be either in the memory, database,
    etc.
    """

    def __init__(self):
        Observer.__init__(self)


@ObservableModel.add(ObservableModel)
class ModelMT(Model):
    def __init__(self):
        Model.__init__(self)
        self._threads = {}
        self._lock = threading.Lock()

    def register_observer(self, observer):
        """
        Registers an observer
        """

        Model.register_observer(self, observer)
        self._threads[observer] = threading.currentThread()

    def unregister_observer(self, observer):
        """
        Unregisters an observer
        """

        Model.unregister_observer(self, observer)
        del self._threads[observer]

    def notify_observer(self, observer, method, *args, **kwargs):
        if self._threads[observer] == threading.currentThread():
            return Model.notify_observer(self, observer, method, *args, **kwargs)
        GLib.idle_add(self._idle_notify_observer, method, args, kwargs)

    def _idle_notify_observer(self, method, args, kwargs):
        with self._lock:
            method(*args, **kwargs)
