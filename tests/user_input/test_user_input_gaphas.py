import os
import time

# core elements
import rafcon.core.config

# general tool elements
from rafcon.utils import log
import testing_utils
from testing_utils import call_gui_callback, run_gui, wait_for_gui_quit

logger = log.get_logger(__name__)


def resize_state(sm_model, sm_notebook_page, graphical_editor_controller):
    # Important: do not remove any information (including comments) of this function! Needed for debugging!
    import pymouse
    mouse = pymouse.PyMouse()
    from gaphas.item import Element, NW, NE, SE, SW

    from rafcon.gui.controllers.graphical_editor_gaphas import GraphicalEditorController
    assert isinstance(graphical_editor_controller, GraphicalEditorController)
    state_view_for_root_state = graphical_editor_controller.canvas.get_view_for_model(sm_model.root_state)
    from rafcon.gui.mygaphas.items.state import StateView
    assert isinstance(state_view_for_root_state, StateView)
    # print state_view_for_root_state
    # p = state_view_for_root_state.position
    # print state_view_for_root_state.position
    # print state_view_for_root_state.matrix
    # print state_view_for_root_state.handles()
    # print state_view_for_root_state.handles()[NW].pos.x
    # print state_view_for_root_state.handles()[NW].pos.y
    # print state_view_for_root_state.handles()[SE].pos.x
    # print state_view_for_root_state.handles()[SE].pos.y

    # self.view.get_matrix_i2v(self).transform_distance(width, height)
    from gaphas.view import View
    assert (state_view_for_root_state.view, View)

    v2i = state_view_for_root_state.view.get_matrix_v2i(state_view_for_root_state)
    i2v = state_view_for_root_state.view.get_matrix_i2v(state_view_for_root_state)
    c2i = state_view_for_root_state.canvas.get_matrix_c2i(state_view_for_root_state)
    i2c = state_view_for_root_state.canvas.get_matrix_i2c(state_view_for_root_state)

    # item_base_x, item_base_y = v2i.transform_point(p[0], p[1])
    # item_base_x, item_base_y = i2v.transform_point(p[0], p[1])
    #
    # item_base_x, item_base_y = i2c.tranform_point(v2i.transform_point(0, 0))
    # item_base_x, item_base_y = 0, 0

    se_x, se_y = i2v.transform_point(state_view_for_root_state.handles()[SE].pos.x.value,
                                     state_view_for_root_state.handles()[SE].pos.y.value)

    main_w = rafcon.gui.singleton.main_window_controller.view.get_top_widget()
    pos_main = main_w.get_position()
    rel_pos = sm_notebook_page.translate_coordinates(main_w, 0, 0)
    abs_pos_se = (pos_main[0] + rel_pos[0] + se_x, pos_main[1] + rel_pos[1] + se_y)

    mouse.move(*abs_pos_se)
    mouse.press(*abs_pos_se)
    mouse.release(abs_pos_se[0] + 20, abs_pos_se[1] + 20)


def south_east_coordinates_of_model(gaphas_view):
    from gaphas.item import Element, NW, NE, SE, SW
    i2v = gaphas_view.view.get_matrix_i2v(gaphas_view)
    x, y = i2v.transform_point(gaphas_view.handles()[SE].pos.x.value,
                               gaphas_view.handles()[SE].pos.y.value)
    return x, y


def create_and_resize_state():
    import gtk
    from rafcon.gui.singleton import main_window_controller
    # gvm = rafcon.core.singleton.global_variable_manager
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    # from rafcon.gui.controllers.menu_bar import MenuBarController
    # assert isinstance(menubar_ctrl, MenuBarController)
    # execution_engine = singletons.state_machine_execution_engine
    # state_machine_manager = singletons.state_machine_manager

    call_gui_callback(menubar_ctrl.on_new_activate, None)
    # time.sleep(1.0)
    time.sleep(0.5)

    testing_utils.wait_for_gui()

    sm_ctrls = main_window_controller.get_controller('state_machines_editor_ctrl')
    graphical_editor_controller = sm_ctrls.get_controller(1)
    # get first state machine page in state machines notebook
    sm_page = sm_ctrls.tabs.items()[0][1]['page']
    sm_model = sm_ctrls.tabs.items()[0][1]['state_machine_m']

    state_view_for_root_state = graphical_editor_controller.canvas.get_view_for_model(sm_model.root_state)
    x, y = south_east_coordinates_of_model(state_view_for_root_state)

    # print x, y

    call_gui_callback(resize_state, sm_model, sm_page, graphical_editor_controller)

    # testing_utils.wait_for_gui()
    # graphical_editor_controller.canvas.update()

    new_x, new_y = south_east_coordinates_of_model(state_view_for_root_state)
    sleep_time = 0.01
    max_counter = 10.0 / 0.01
    counter = 0
    while new_x == x and new_y == y:
        new_x, new_y = south_east_coordinates_of_model(state_view_for_root_state)
        counter += 1
        if counter > max_counter:
            break
        time.sleep(sleep_time)

    # print new_x, new_y

    assert x < new_x
    assert y < new_y

    # time.sleep(10.0)


def test_user_input_gaphas(caplog):
    run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
            libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")})
    try:
        create_and_resize_state()
    except Exception, e:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_user_input_gaphas(None)
    # pytest.main(['-s', __file__])
