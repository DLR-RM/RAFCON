# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: gui_functions
   :synopsis: A module holding utility functions for gtk

"""


# This function is intentionally placed here, as placing it inside rafcon.gui will call rafcon.gui.__init__.py
# and create the singletons, which is not desired inside the unit tests
def call_gui_callback(callback, *args, **kwargs):
    """Wrapper method for GLib.idle_add

    This method is intended as replacement for idle_add. It wraps the method with a callback option. The advantage is
    that this way, the call is blocking. The method return, when the callback method has been called and executed.

    :param callback: The callback method, e.g. on_open_activate
    :param args: The parameters to be passed to the callback method
    """
    from threading import Condition
    import sys
    from rafcon.utils import log
    from gi.repository import GLib
    # concerning sharing data with inner functions see: https://stackoverflow.com/a/11987516
    communication_dict = dict()
    communication_dict["exception_info"] = None
    communication_dict["result"] = None

    condition = Condition()

    @log.log_exceptions()
    def fun():
        """Call callback and notify condition variable
        """
        try:
            communication_dict["result"] = callback(*args, **kwargs)
        except:
            # Exception within this asynchronously called function won't reach pytest. This is why we have to store
            # the information about the exception to re-raise it at the end of the synchronous call.
            communication_dict["exception_info"] = sys.exc_info()
        finally:  # Finally is also executed in the case of exceptions
            condition.acquire()
            condition.notify()
            condition.release()

    if "priority" in kwargs:
        priority = kwargs.pop("priority")
    else:
        priority = GLib.PRIORITY_LOW

    with condition:
        GLib.idle_add(fun, priority=priority)
        # Wait for the condition to be notified
        # TODO: implement timeout that raises an exception
        condition.wait()
    if communication_dict["exception_info"]:
        e_class, e_instance, e_traceback = communication_dict["exception_info"]
        raise e_instance.with_traceback(e_traceback)
    return communication_dict["result"]
