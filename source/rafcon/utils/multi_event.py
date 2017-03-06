# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: multi_event
   :synopsis: A module for creating multi_events that listen to several `threading.Event`s

"""
import threading


# adapted from:
# http://stackoverflow.com/questions/12317940/python-threading-can-i-sleep-on-two-threading-events-simultaneously


def or_set(self):
    """A function to overwrite the default set function of threading.Events

    :param self: Reference to the event
    """
    self._set()
    for callback in self.callbacks:
        callback()


def or_clear(self):
    """ A function to overwrite the default clear function of the threading.Event

    :param self: Reference to the event
    """
    self._clear()
    for callback in self.callbacks:
        callback()


def orify(e, changed_callback):
    """Add another event to the multi_event

    :param e: the event to be added to the multi_event
    :param changed_callback: a method to call if the event status changes, this method has access to the multi_event
    :return:
    """
    if not hasattr(e, "callbacks"):  # Event has not been orified yet
        e._set = e.set
        e._clear = e.clear
        e.set = lambda: or_set(e)
        e.clear = lambda: or_clear(e)
        e.callbacks = list()
    # Keep track of one callback per multi event
    e.callbacks.append(changed_callback)


def create(*events):
    """Creates a new multi_event

    The multi_event listens to all events passed in the "events" parameter.

    :param events: a list of threading.Events
    :return: The multi_event
    :rtype: threading.Event
    """
    or_event = threading.Event()

    def changed():
        if any([event.is_set() for event in events]):
            or_event.set()
        else:
            or_event.clear()

    for e in events:
        orify(e, changed)

    changed()
    return or_event

