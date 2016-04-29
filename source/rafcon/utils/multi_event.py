"""
.. module:: multi_event
   :platform: Unix, Windows
   :synopsis: A module for creating multi_events, that listens to several threading.Events

.. moduleauthor:: Sebastain Brunner


"""
import threading


# adapted from:
# http://stackoverflow.com/questions/12317940/python-threading-can-i-sleep-on-two-threading-events-simultaneously


def or_set(self):
    """
    A function to overwrite the default set function of threading.Events
    :return:
    """
    self._set()
    self.changed()


def or_clear(self):
    """
    A function to overwrite the default clear function of the threading.Event
    :return:
    """
    self._clear()
    self.changed()


def orify(e, changed_callback):
    """Add another event to the multi_event

    :param e: the event to be added to the multi_event
    :param changed_callback: a method to call if the event status changes, this method has access to the multi_event
    :return:
    """
    e._set = e.set
    e._clear = e.clear
    e.changed = changed_callback
    e.set = lambda: or_set(e)
    e.clear = lambda: or_clear(e)


def create_multi_event(*events):
    """
    Creates a new multi_event, that listens to all events passed in the "events" parameter
    :param events: a list of threading.Events
    :return:
    """
    or_event = threading.Event()

    def changed():
        bools = [e.is_set() for e in events]
        if any(bools):
            or_event.set()
        else:
            or_event.clear()

    for e in events:
        orify(e, changed)

    changed()
    return or_event

