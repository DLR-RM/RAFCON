# Copyright (C) 2015-2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Mohamed ElSherbini <mohamed.elsherbini@dlr.de>
# Johannes Ernst <j.ernst@dlr.de>

"""
A file containing the breakpoint_manager module.
It is used to store breakpoints for state machines set by the user. 
All breakpoints are deleted when the GUI or the core is closed.
"""

import os
import weakref
from rafcon.utils import log

logger = log.get_logger(__name__)


class BreakpointManager:

    def __init__(self):
        self._breakpoints = {}
        self._listeners = []

    def add_listener(self, callback):
        if callback not in self._listeners:
            self._listeners.append(weakref.WeakMethod(callback))

    def is_same_callback(self, ref, callback):
        # Resolve the saved weak reference and check if it matches with the callback
        func = ref()
        if func is None:
            return False
        return (
            func.__self__ is callback.__self__ and
            func.__func__ is callback.__func__
        )

    def remove_listener(self, callback):
        self._listeners = [
            ref for ref in self._listeners if not self.is_same_callback(ref, callback)
        ]

    def _notify(self):
        # Notify callback functions all listeners and check if they are alive
        alive = []
        for ref in self._listeners:
            # Resolve the weak reference
            listener = ref()
            if listener is not None:
                listener()
                alive.append(ref)
        self._listeners = alive

    @staticmethod
    def _get_state_id(state):
        if state is None or state.file_system_path is None:
            return None
        return os.path.basename(state.file_system_path)

    def add_breakpoint(self, state, display_name):
        state_id = self._get_state_id(state)
        self._breakpoints[state_id] = {
            'enabled': True,
            'name': display_name,
            'display_path': state.file_system_path
        }
        logger.info(f"✓ Breakpoint: {display_name}")
        logger.info(f"  Path: {state.file_system_path}")
        self._notify()

    def remove_breakpoint(self, state):
        state_id = self._get_state_id(state)
        self.remove_breakpoint_by_id(state_id)

    def remove_breakpoint_by_id(self, state_id):
        if state_id in self._breakpoints:
            del self._breakpoints[state_id]
            self._notify()

    def toggle_breakpoint(self, state_id):
        if state_id in self._breakpoints:
            self._breakpoints[state_id]['enabled'] = not self._breakpoints[state_id]['enabled']
            self._notify()

    def clear_all(self):
        self._breakpoints.clear()
        self._notify()

    def should_pause(self, state):
        state_id = self._get_state_id(state)
        if state_id is None:
            return False
        if state_id in self._breakpoints:
            return self._breakpoints[state_id]['enabled']
        return False

    def get_all_breakpoints(self):
        return dict(self._breakpoints)

    def disable_all(self):
        for bp in self._breakpoints.values():
            bp['enabled'] = False
        self._notify()

    def enable_all(self):
        for bp in self._breakpoints.values():
            bp['enabled'] = True
        self._notify()
