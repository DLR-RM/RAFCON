# Copyright (C) 2020 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

from rafcon.utils import log

logger = log.get_logger(__name__)


def main_window2graphical_editor(main_window_coordinates):
    """
    Transforms a point relative to the main window, into a point relative to the graphical editor.
    (window coordinates --> view coordinates).
    :param (float, float) main_window_coordinates:
    A tuple containing x and y coordinates relative to the main window.
    :return: The same point relative to the graphical editor (the view coordinate system).
    :rtype: (float, float)
    """
    from rafcon.gui.singleton import main_window_controller
    main_window = main_window_controller.view.get_top_widget()
    sm_controllers = main_window_controller.state_machines_editor_ctrl
    sm_notebook_page = list(sm_controllers.tabs.items())[0][1]['page']
    return main_window.translate_coordinates(sm_notebook_page, main_window_coordinates[0], main_window_coordinates[1])


def graphical_editor2main_window(ge_coordinates):
    """
    Transforms a point relative to the graphical editor, into a point relative to the main window.
    (view coordinates --> window coordinates)
    :param (float, float) ge_coordinates:
    A tuple containing a x and a y coordinate relative to the state machine notebook.
    :return: The same point relative to the main window.
    :rtype: (float, float)
    """
    from rafcon.gui.singleton import main_window_controller
    main_window = main_window_controller.view.get_top_widget()
    sm_controllers = main_window_controller.state_machines_editor_ctrl
    sm_notebook_page = list(sm_controllers.tabs.items())[0][1]['page']
    return sm_notebook_page.translate_coordinates(main_window, ge_coordinates[0], ge_coordinates[1])


def graphical_editor2item(target_state_m, ge_coordinates):
    """
    Transforms a point relative to the graphical editor, into a point relative to the target state model.
    (view coordinates --> item coordinates)
    :param StateModel target_state_m: The state model, the resulting point should be relative to.
    :param (float,float) ge_coordinates: A tuple containing a x and a y coordinate relative to the graphical editor.
    :return: The same point relative to the target state model.
    :rtype: (float,float)
    """
    from rafcon.gui.singleton import main_window_controller
    from rafcon.gui.singleton import state_machine_manager_model
    state_machine_m = state_machine_manager_model.get_state_machine_model(target_state_m)
    sm_id = state_machine_m.state_machine.state_machine_id
    sm_controllers = main_window_controller.state_machines_editor_ctrl
    graphical_editor_controller = sm_controllers.get_controller(sm_id)
    root_state_m = state_machine_m.root_state
    state_view_for_root_state = graphical_editor_controller.canvas.get_view_for_model(root_state_m)
    state_view_for_target_state = graphical_editor_controller.canvas.get_view_for_model(target_state_m)
    v2i = state_view_for_root_state.view.get_matrix_v2i(state_view_for_target_state)
    item_coordinates = v2i.transform_point(ge_coordinates[0], ge_coordinates[1])
    return item_coordinates


def item2graphical_editor(target_state_m, item_coordinates):
    """
    Transforms a point relative to given target state model, into a point relative to the graphical editor.
    (item coordinates --> view coordinates)
    :param StateModel target_state_m: The state model, the given point is relative to.
    :param (float,float) item_coordinates:
    A tuple containing a x and a y coordinate relative to the given target state model.
    :return: The same point relative to the graphical editor.
    :rtype: (float,float)
    """
    from rafcon.gui.singleton import main_window_controller
    from rafcon.gui.singleton import state_machine_manager_model
    state_machine_m = state_machine_manager_model.get_state_machine_model(target_state_m)
    sm_id = state_machine_m.state_machine.state_machine_id
    sm_controllers = main_window_controller.state_machines_editor_ctrl
    graphical_editor_controller = sm_controllers.get_controller(sm_id)
    root_state_m = state_machine_m.root_state
    state_view_for_root_state = graphical_editor_controller.canvas.get_view_for_model(root_state_m)
    state_view_for_target_state = graphical_editor_controller.canvas.get_view_for_model(target_state_m)

    i2v = state_view_for_root_state.view.get_matrix_i2v(state_view_for_target_state)
    ge_coordinates = i2v.transform_point(item_coordinates[0], item_coordinates[1])
    return ge_coordinates


def screen2main_window(screen_coordinates):
    """
    Transforms a point in absolute screen coordinates, into a point relative to the main window.
    :param (float,float) screen_coordinates: A tuple of x and y coordinate in absolute screen coordinates.
    :return: The same point relative to the main window.
    :rtype: (float,float)
    """
    from rafcon.gui.singleton import main_window_controller
    main_window = main_window_controller.view.get_top_widget()
    main_window_pos = main_window.get_position()
    return screen_coordinates[0] - main_window_pos[0], screen_coordinates[1] - main_window_pos[1]


def main_window2screen(main_window_coordinates):
    """
    Transforms a point relative to the main window, into a point in absolute screen coordinates.
    :param (float, float) main_window_coordinates: A tuple of x and y coordinate relative to the main window.
    :return: The same point in absolute screen coordinates.
    :rtype: (float, float)
    """
    from rafcon.gui.singleton import main_window_controller
    main_window = main_window_controller.view.get_top_widget()
    main_window_pos = main_window.get_position()
    return main_window_pos[0] + main_window_coordinates[0], main_window_pos[1] + main_window_coordinates[1]
