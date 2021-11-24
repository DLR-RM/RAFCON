import os

# general tool elements
from rafcon.utils import filesystem
import rafcon.utils.log as log

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback

logger = log.get_logger(__name__)


def grep_process_ids(process_name):
    import psutil
    gedit_instances = []
    for process in psutil.process_iter():
        if process_name in process.name():
            # print process_name, process.name(), process.pid
            gedit_instances.append(process.pid)
    return gedit_instances


def find_running_process(process_name):
    return len(grep_process_ids(process_name)) > 0


def kill_running_processes(process_name):

    instances = grep_process_ids(process_name)

    for pid in instances:
        os.system('kill ' + str(pid))


def check_for_editor(editor):
    import subprocess
    try:
        subprocess.Popen(editor)
        kill_running_processes(editor)
        return True
    except OSError:
        return False


def compile_script(gui, state_m):
    try:
        gui(state_m.state.script.compile_module)
    except Exception as e:  # dedicated exception needed as setting the script text does not compile the script
        logger.error("Error in script")


def test_gui(gui):
    # queue = Queue.Queue() # TODO think about to use this to get call_back methods return value by a generic scheme
    # thread = threading.Thread(target=lambda q, arg1: q.put(trigger_source_editor_signals(arg1)), args=(queue, main_window_controller))
    errors = 3

    import rafcon
    import rafcon.gui.config as gui_config
    import psutil

    # ---setup---
    menubar_ctrl = gui.singletons.main_window_controller.menu_bar_controller

    # ---setup new statemachine and add a new state---
    gui(menubar_ctrl.on_new_activate, None)
    gui(menubar_ctrl.on_add_state_activate, None)

    # ---focus the newly added state and get the source controller---
    sm_m = menubar_ctrl.model.get_selected_state_machine_model()
    root_state_m = sm_m.root_state
    state_m = list(root_state_m.states.values())[0]
    states_editor_controller = gui.singletons.main_window_controller.states_editor_ctrl
    state_identifier = states_editor_controller.get_state_identifier(state_m)
    gui(states_editor_controller.activate_state_tab, state_m)
    tab_list = states_editor_controller.tabs
    source_editor_controller = tab_list[state_identifier]['controller'].get_controller('source_ctrl')

    # ---check if the default text equals the default_script.py
    default_content = filesystem.read_file(os.path.join(rafcon.__path__[0], 'core', 'default_script.py'))
    content = source_editor_controller.source_text
    assert content == default_content

    # ---check if the source text can be changed---
    content = 'Test'
    # This adds an additional error, as this script cannot be compiled
    gui(source_editor_controller.set_script_text, content)
    compile_script(gui, state_m)
    assert content == source_editor_controller.source_text
    gui.expected_errors += 1  # NameError: name 'Test' is not defined

    # ---check if a wrong shell command returns false by the append_...() method
    assert not gui(source_editor_controller.execute_shell_command_with_path, 'gibberish', 'foo.txt')
    gui.expected_errors += 1  # No such file or directory (gibberish)

    source_view = source_editor_controller.view
    test_text = 'apply_test'

    # get the textview buffer and replace the buffer text with another
    test_buffer = source_view.get_buffer()
    # This adds an additional error, as this script cannot be compiled
    gui(test_buffer.set_text, test_text, -1)

    # ---check if a new buffer doesn't change the source text
    gui(source_view.textview.set_buffer, test_buffer)
    assert not source_editor_controller.source_text == test_text

    # ---check if the cancel button resets the buffer to the source text
    cancel_button = source_view['cancel_button']
    gui(source_editor_controller.cancel_clicked, cancel_button)
    assert source_view.get_buffer().get_text(test_buffer.get_start_iter(), test_buffer.get_end_iter(),
                                             include_hidden_chars=True) == content

    # test buffer now contains the source_text which equals content so test_buffer is again set to contain test_text
    gui(test_buffer.set_text, test_text, -1)

    # ---check if changing the buffer and applying the changes has an impact on the source text
    gui(source_view.textview.set_buffer, test_buffer)
    print("test_buffer " + test_buffer.get_text(test_buffer.get_start_iter(), test_buffer.get_end_iter(),
                                                include_hidden_chars=True))
    apply_button = source_view['apply_button']
    gui(source_editor_controller.apply_clicked, apply_button)
    compile_script(gui, state_m)
    assert source_editor_controller.source_text == test_text
    gui.expected_errors += 1  # NameError: name 'apply_test' is not defined

    # ----------- Test requiring psutil to work ------------
    try:
        psutil.process_iter()
    except psutil.AccessDenied:
        logger.warning("Cannot finish test: Cannot access process list")
        gui.expected_warnings += 1
        return

    # ----------- Test requiring gedit to work ------------
    try:
        if not check_for_editor('gedit'):
            logger.warning("Cannot finish test: Cannot find gedit")
            gui.expected_warnings += 1
            return
    except psutil.AccessDenied:
        logger.warning("Cannot finish test: Cannot access gedit")
        gui.expected_warnings += 1
        return


    # ---check if the open external button opens a gedit instance

    try:
        kill_running_processes('gedit')
    except psutil.AccessDenied:
        logger.warning("Cannot finish test: Cannot kill gedit")
        gui.expected_warnings += 1
        return

    gui(gui_config.global_gui_config.set_config_value, 'DEFAULT_EXTERNAL_EDITOR', 'gedit')
    button = source_view['open_external_button']

    gui(button.set_active, True)
    assert find_running_process('gedit')
    assert button.get_label() == 'Unlock'

    kill_running_processes('gedit')

    gui(button.set_active, False)
    assert button.get_label() == 'Open externally'


if __name__ == '__main__':
    # test_gui(None)
    import pytest
    pytest.main(['-s', __file__])
