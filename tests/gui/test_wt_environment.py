from __future__ import print_function

import pytest

import threading

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback


@pytest.mark.parametrize('gui', [{"runtime_config": {
    'LEFT_BAR_HIDDEN': False,
    'RIGHT_BAR_HIDDEN': False,
    'CONSOLE_HIDDEN': False,
}}], indirect=True, ids=["with visible sidebars"])
def test_thread_observer_creation_list(gui):
    """ By searching for the WT thread number can been seen, that
    """
    # The side bars have been hidden in the state_resize test; here we show them again
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine
    import rafcon.core.singleton
    import rafcon.gui.singleton

    print("WT_ident: ", threading.currentThread().ident)
    print("CORE_singleton_init_thread_ident: ", gui.core_singletons.thread_identifier)
    print("GUI_singleton_init_thread_ident: ", gui.singletons.thread_identifier)
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller
    gui(menubar_ctrl.on_new_activate, None)
    # negative test
    # menubar_ctrl.on_new_activate(None)
    print("\n"*3, "WT generated object", "\n"*3)
    gui(gui.core_singletons.state_machine_manager.add_state_machine, StateMachine(HierarchyState("new root state")))

if __name__ == '__main__':
    test_thread_observer_creation_list(None)
