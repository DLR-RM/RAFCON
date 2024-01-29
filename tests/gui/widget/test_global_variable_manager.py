import pytest

from tests import utils as testing_utils

from rafcon.utils import log

logger = log.get_logger(__name__)


# TODO this needs a test run from the widget view site

def test_gui(gui):
    gui(testing_utils.remove_all_gvm_variables)
    gvm = gui.core_singletons.global_variable_manager
    # gvm_controller = gui.gui_singletons.main_window_controller.get_controller('global_variable_manager_ctrl')
    gvm_controller = gui.singletons.main_window_controller.global_variable_manager_ctrl

    view = gvm_controller.view['global_variable_tree_view']
    gui(view.grab_focus)

    gui(gvm.set_variable, 'new_0', 0)

    # use gui callback to wait for gv row generation
    gui(gvm_controller.apply_new_global_variable_value, 0, '2')

    def run_gvm_method0(method, return_list):
        return_list.append(method())

    def run_gvm_method1(method, gv_arg1, return_list):
        return_list.append(method(gv_arg1))

    def run_gvm_method2(method, gv_arg1, gv_arg2, return_list):
        return_list.append(method(gv_arg1, gv_arg2))

    return_list = list()
    gui(run_gvm_method1, gvm.get_variable, 'new_0', return_list)
    assert return_list[-1] == 2

    gui(gvm_controller.apply_new_global_variable_name, 0, 'changed_global_0')
    gui(run_gvm_method1, gvm.get_variable, 'changed_global_0', return_list)
    assert return_list[-1]

    gui(gvm_controller.apply_new_global_variable_type, 0, 'float')
    gui(run_gvm_method1, gvm.get_data_type, 'changed_global_0', return_list)
    assert return_list[-1] is float

    gui(run_gvm_method1, gvm.lock_variable, 'changed_global_0', return_list)
    access_key = return_list[-1]
    assert not gvm_controller.global_variable_is_editable('changed_global_0', 'testing...')
    gui.expected_errors += 1

    gui(run_gvm_method2, gvm.unlock_variable, 'changed_global_0', access_key, return_list)

    gui(gvm_controller.on_add, view)
    gui(run_gvm_method0, gvm.get_all_keys, return_list)
    assert len(return_list[-1]) == 2

    gui(gvm_controller.remove_core_element, 'changed_global_0')
    gui(run_gvm_method0, gvm.get_all_keys, return_list)
    assert len(return_list[-1]) == 1


if __name__ == '__main__':
    # test_gui(None)
    import pytest
    pytest.main(['-s', __file__])
