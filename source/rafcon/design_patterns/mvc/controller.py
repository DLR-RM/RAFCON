import inspect

from gi.repository import GLib

from rafcon.design_patterns.observer.observer import Observer


class Controller(Observer):
    """
    The Controller class of the MVC pattern. It holds the logic and makes the connection between the model and the view.
    """

    def __init__(self, model, view):
        Observer.__init__(self, model)
        self.model = model
        self.view = view
        GLib.idle_add(self._idle_init, priority=GLib.PRIORITY_HIGH)

    def _idle_init(self):
        self.view.connect_signals({event: [callback] for event, callback in inspect.getmembers(self.__class__, predicate=inspect.ismethod)})
        self.register_view(self.view)

    def register_view(self, view):
        """
        Registers a specific view (must be overwritten)
        """

        pass
