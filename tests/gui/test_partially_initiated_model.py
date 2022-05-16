# general tool elements
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback

import pytest

logger = log.get_logger(__name__)


@pytest.mark.parametrize('gui', [{"gui_config": {
    'MAX_VISIBLE_LIBRARY_HIERARCHY': 1,
    'NOT_FULLY_RECURSIVE_LIBRARY_MODEL': True}
}], indirect=True, ids=["with limited library visibility depth"])
def test_gui(gui):
    """The function triggers and test basic functions of the menu bar and configuration model to test partially 
    initiated models.

    - test partial initiated generic/dialog/Dialog [3 options] -> covers bug issue #658
        
    """
    from os.path import join

    from rafcon.core.states.library_state import LibraryState
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    gui(menubar_ctrl.on_new_activate, None)

    lib_state = LibraryState(join("generic", "dialog"), "Dialog [3 options]", "0.1", "Dialog [3 options]")
    gui(gui_helper_state_machine.insert_state_into_selected_state, lib_state, False)

    lib_tree_ctrl = main_window_controller.get_controller('library_controller')
    from rafcon.gui.controllers.library_tree import LibraryTreeController
    assert isinstance(lib_tree_ctrl, LibraryTreeController)

    ##############################################################
    print("\n" * 3 + "#" * 60 + "\n# Insert a library with may library depth level 1 " + "\n" * 3)
    ##############################################################
    library_path, library_name = ('generic/dialog', 'Show dialog')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.insert_button_clicked, None, False)

    ##############################################################
    print("\n" * 3 + "#" * 60 + "\n# Insert a library with may library depth level 2 " + "\n" * 3)
    ##############################################################
    library_path, library_name = ('generic/dialog', 'Dialog [3 options]')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.insert_button_clicked, None, False)

    ##############################################################
    print("\n" * 3 + "#" * 60 + "\n# Insert a library with may library depth level 1 as template " + "\n" * 3)
    ##############################################################
    library_path, library_name = ('generic/dialog', 'Show dialog')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.insert_button_clicked, None, True)

    ##############################################################
    print("\n" * 3 + "#" * 60 + "\n# Insert a library with may library depth level 2 as template " + "\n" * 3)
    ##############################################################
    library_path, library_name = ('generic/dialog', 'Dialog [3 options]')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.insert_button_clicked, None, True)


if __name__ == '__main__':
    # test_gui(None)
    pytest.main(['-s', __file__])
