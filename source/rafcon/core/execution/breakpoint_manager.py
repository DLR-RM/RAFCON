import os
from rafcon.utils import log

logger = log.get_logger(__name__)


class BreakpointManager:

    def __init__(self):
        self._breakpoints = {}

    @staticmethod
    def _get_state_id(state):
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

    def remove_breakpoint(self, state):
        state_id = self._get_state_id(state)
        if state_id in self._breakpoints:
            del self._breakpoints[state_id]

    def toggle_breakpoint(self, state_id):
        if state_id in self._breakpoints:
            self._breakpoints[state_id]['enabled'] = not self._breakpoints[state_id]['enabled']

    def clear_all(self):
        self._breakpoints.clear()

    def should_pause(self, state):
        state_id = self._get_state_id(state)
        if state_id in self._breakpoints:
            return self._breakpoints[state_id]['enabled']
        return False

    def get_all_breakpoints(self):
        return dict(self._breakpoints)

    def disable_all(self):
        for bp in self._breakpoints.values():
            bp['enabled'] = False

    def enable_all(self):
        for bp in self._breakpoints.values():
            bp['enabled'] = True
