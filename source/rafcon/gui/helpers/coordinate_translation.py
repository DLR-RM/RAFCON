# Copyright (C) 2020 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

from rafcon.gui.singleton import main_window_controller
from rafcon.utils import log

logger = log.get_logger(__name__)


class CoordinateTranslator:
    """
    The CoordinateTranslator is a helper class, helping to transform coordinates between different coordinate systems
    (e.g screen, item, window, and view coordinates) in the GUI.
    """
    main_window = None
    sm_notebook_page = None
    graphical_editor_controller = None

    def __init__(self):
        self.main_window = main_window_controller.view.get_top_widget()
        sm_controllers = main_window_controller.state_machines_editor_ctrl
        self.sm_notebook_page = list(sm_controllers.tabs.items())[0][1]['page']
        self.graphical_editor_controller = sm_controllers.get_controller(1)

    def main_window_coordinates_to_notebook_coordinates(self, main_window_coordinates):
        """
        Transforms a point relative to the main window, into a point relative to the state machine notebook.
        (window coordinates --> view coordinates).
        :param (float, float) main_window_coordinates: A tuple containing a x and a y coordinate relative to the main window.
        :return: The same point, but relative to the state machine notebook (That's the view coordinate system).
        :rtype: (float, float)
        """
        return self.main_window.translate_coordinates(self.sm_notebook_page, main_window_coordinates[0],
                                                      main_window_coordinates[1])

    def notebook_coordinates_to_main_window_coordinates(self, notebook_coordinates):
        """
        Transforms a point relative to the state machine notebook, into a point relative to the main window.
        (view coordinates --> window coordinates)
        :param (float, float) notebook_coordinates: A tuple containing a x and a y coordinate relative to the state machine notebook.
        :return: The same point, but relative to the main window.
        :rtype: (float, float)
        """
        return self.sm_notebook_page.translate_coordinates(self.main_window, notebook_coordinates[0],
                                                           notebook_coordinates[1])

    def notebook_coordinates_to_item_coordinates(self, state_machine_m, target_state_m, notebook_coordinates):
        """
        Transforms a point relative to the state machine notebook, into a point relative to the target state model.
        (view coordinates --> item coordinates)
        :param StateMachineModel state_machine_m: The state machine model, the target state model belongs to.
        :param StateModel target_state_m: The state model, the resulting point should be relative to.
        :param (float,float) notebook_coordinates: A tuple containing a x and a y coordinate relative to the state machine notebook.
        :return: The same point, relative to the target state model.
        :rtype: (float,float)
        """
        root_state = self.__get_root_state(target_state_m.state)
        root_state_m = state_machine_m.get_state_model_by_path(root_state.get_path())
        state_view_for_root_state = self.graphical_editor_controller.canvas.get_view_for_model(root_state_m)
        state_view_for_target_state = self.graphical_editor_controller.canvas.get_view_for_model(target_state_m)
        v2i = state_view_for_root_state.view.get_matrix_v2i(state_view_for_target_state)
        item_coordinates = v2i.transform_point(notebook_coordinates[0], notebook_coordinates[1])
        return item_coordinates

    def item_coordinates_to_notebook_coordinates(self, state_machine_m, target_state_m, item_coordinates):
        """
        Transforms a point relative to given target state model, into a point relative to the state machine notebook.
        (item coordinates --> view coordinates)
        :param StateMachineModel state_machine_m: The state machine model, the target state model belongs to.
        :param StateModel target_state_m: The state model, the given point is relative to.
        :param (float,float) item_coordinates: A tuple containing a x and a y coordinate relative to the given target state model.
        :return: The same point, but relative to the state machine notebook.
        :rtype: (float,float)
        """
        root_state = self.__get_root_state(target_state_m.state)
        root_state_m = state_machine_m.get_state_model_by_path(root_state.get_path())
        state_view_for_root_state = self.graphical_editor_controller.canvas.get_view_for_model(root_state_m)
        state_view_for_target_state = self.graphical_editor_controller.canvas.get_view_for_model(target_state_m)

        i2v = state_view_for_root_state.view.get_matrix_i2v(state_view_for_target_state)
        notebook_coordinates = i2v.transform_point(item_coordinates[0], item_coordinates[1])
        return notebook_coordinates

    def screen_coordinates_to_main_window_coordiantes(self, screen_coordinates):
        """
        Transforms a point in absolute screen coordinates, into a point relative to the main window.
        :param (float,float) screen_coordinates: A tuple of x and y coordinate in absolute screen coordinates.
        :return: The same point, but relative to the main window.
        :rtype: (float,float)
        """
        main_window_pos = self.main_window.get_position()
        return screen_coordinates[0] - main_window_pos[0], screen_coordinates[1] - main_window_pos[1]

    def main_window_coordinates_to_screen_coordinates(self, main_window_coordinates):
        """
        Transforms a point relative to the main window, into a point in absolute screen coordinates.
        :param (float, float) main_window_coordinates: A tuple of x and y coordinate relative to the main window.
        :return: The same point, but in absoulte screen coordinates.
        :rtype: (float, float)
        """
        main_window_pos = self.main_window.get_position()
        return main_window_pos[0] + main_window_coordinates[0], main_window_pos[1] + main_window_coordinates[1]

    def __get_root_state(self, state):
        """
        :param State state: A state of a state machine.
        :return: The root state of state machine, the given state belongs to. The state itself, if the state is the root state.
        :rtype: State
        """
        root_state = state
        while not root_state.is_root_state:
            root_state = root_state.parent
        return root_state
