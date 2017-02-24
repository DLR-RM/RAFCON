# Copyright

"""
.. module:: decorators
   :synopsis: A module to hold all decorators needed for the RAFCON core

"""

global_lock_counter = 0


def lock_state_machine(func):
    def func_wrapper(*args, **kwargs):
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