# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: decorators
   :synopsis: A module to hold all decorators needed for the RAFCON core

"""

from builtins import filter
import functools
import itertools


def wraps_safely(obj, attr_names=functools.WRAPPER_ASSIGNMENTS):
    # Solves problem with missing attributes: http://stackoverflow.com/a/28752007
    return functools.wraps(obj, assigned=list(filter(functools.partial(hasattr, obj), attr_names)))

global_lock_counter = 0


def lock_state_machine(func):
    @wraps_safely(func)
    def func_wrapper(*args, **kwargs):
        """ Decorate method to observable core edit methods. If the core method of rafcon core object is called
        the respective state machine object edition will be locked by the respective thread until the handed function
        execution is finished.
        """
        from rafcon.core.state_elements.state_element import StateElement
        from rafcon.core.states.state import State
        global global_lock_counter
        self_reference = args[0]
        target_state_machine = None
        if isinstance(self_reference, State):
            target_state_machine = self_reference.get_state_machine()
        elif isinstance(self_reference, StateElement):
            if self_reference.parent:
                target_state_machine = self_reference.parent.get_state_machine()

        if target_state_machine:
            target_state_machine.acquire_modification_lock()
            global_lock_counter += 1
        try:
            return_value = func(*args, **kwargs)
        except Exception:
            # logger.debug("Exception occurred during execution of function {0}. ".format(str(func)))
            raise
        finally:
            if target_state_machine:
                target_state_machine.release_modification_lock()
                global_lock_counter -= 1
        return return_value
    return func_wrapper
