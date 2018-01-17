# general tool elements
from testing_utils import call_gui_callback
import testing_utils

from rafcon.utils import log

logger = log.get_logger(__name__)


@log.log_exceptions(None, gtk_quit=True)
def trigger_gvm_signals():
    # core elements
    import rafcon.core.singleton
    import rafcon.gui.singleton
    call_gui_callback(testing_utils.remove_all_gvm_variables)
    gvm = rafcon.core.singleton.global_variable_manager
    gvm_controller = rafcon.gui.singleton.main_window_controller.get_controller('global_variable_manager_ctrl')

    view = gvm_controller.view['global_variable_tree_view']
    view.grab_focus()

    call_gui_callback(gvm.set_variable, 'new_0', 0)

    # use gui callback to wait for gv row generation
    call_gui_callback(gvm_controller.apply_new_global_variable_value, 0, '2')

    def run_gvm_method0(method, return_list):
        return_list.append(method())

    def run_gvm_method1(method, gv_arg1, return_list):
        return_list.append(method(gv_arg1))

    def run_gvm_method2(method, gv_arg1, gv_arg2, return_list):
        return_list.append(method(gv_arg1, gv_arg2))

    return_list = list()
    call_gui_callback(run_gvm_method1, gvm.get_variable, 'new_0', return_list)
    assert return_list[-1] == 2

    call_gui_callback(gvm_controller.apply_new_global_variable_name, 0, 'changed_global_0')
    call_gui_callback(run_gvm_method1, gvm.get_variable, 'changed_global_0', return_list)
    assert return_list[-1]

    call_gui_callback(gvm_controller.apply_new_global_variable_type, 0, 'float')
    call_gui_callback(run_gvm_method1, gvm.get_data_type, 'changed_global_0', return_list)
    assert return_list[-1] is float


    call_gui_callback(run_gvm_method1, gvm.lock_variable, 'changed_global_0', return_list)
    access_key = return_list[-1]
    assert not gvm_controller.global_variable_is_editable('changed_global_0', 'testing...')

    call_gui_callback(run_gvm_method2, gvm.unlock_variable, 'changed_global_0', access_key, return_list)

    call_gui_callback(gvm_controller.on_add, view)
    call_gui_callback(run_gvm_method0, gvm.get_all_keys, return_list)
    assert len(return_list[-1]) is 2

    call_gui_callback(gvm_controller.remove_core_element, 'changed_global_0')
    call_gui_callback(run_gvm_method0, gvm.get_all_keys, return_list)
    assert len(return_list[-1]) is 1

# TODO this needs a test run from the widget view site


def test_gui(caplog):

    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        trigger_gvm_signals()
    except:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=1)


if __name__ == '__main__':
    # test_gui(None)
    import pytest
    pytest.main(['-s', __file__])
