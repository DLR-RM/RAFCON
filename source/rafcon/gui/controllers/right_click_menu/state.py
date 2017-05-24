# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_machine_right_click_menu
   :synopsis: The module provides a menu by right click to edit states or its elements.

"""

import gtk
from functools import partial

import rafcon.core.singleton as core_singletons
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
import rafcon.gui.singleton as gui_singletons
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.config import global_gui_config
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state as gui_helper_state
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import create_image_menu_item, create_check_menu_item, append_sub_menu_to_parent_menu
from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.container_state import ContainerStateModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)

# TODO module needs refactoring, right naming, to separate into capsuled modules and re-arrangement
# TODO  -> to be more useful for more right click extension


class StateMachineRightClickMenu(object):

    def __init__(self, state_machine_manager_model=None):
        if state_machine_manager_model is None:
            from rafcon.gui.singleton import state_machine_manager_model
        self.state_machine_manager_model = state_machine_manager_model
        from rafcon.gui.singleton import main_window_controller
        shortcut_manager = main_window_controller.shortcut_manager
        self.shortcut_manager = shortcut_manager
        self.accel_group = gtk.AccelGroup()

    def generate_right_click_menu_state(self):
        menu = gtk.Menu()
        accel_group = self.accel_group
        shortcuts_dict = global_gui_config.get_config_value('SHORTCUTS')

        self.insert_is_start_state_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_execution_sub_menu_in_menu(menu, shortcuts_dict, accel_group)

        menu.append(gtk.SeparatorMenuItem())

        add_sub_menu_item, add_sub_menu = append_sub_menu_to_parent_menu("Add", menu, constants.BUTTON_ADD)

        add_state_sub_menu_item, add_state_sub_menu = append_sub_menu_to_parent_menu("State", add_sub_menu,
                                                                                     constants.BUTTON_ADD)

        add_state_sub_menu.append(create_image_menu_item("Execution State", constants.BUTTON_ADD,
                                                         self.on_add_execution_state_activate,
                                                         accel_code=shortcuts_dict['add_execution_state'][0],
                                                         accel_group=accel_group))
        add_state_sub_menu.append(create_image_menu_item("Hierarchy State", constants.BUTTON_ADD,
                                                         self.on_add_hierarchy_state_activate,
                                                         accel_code=shortcuts_dict['add_hierarchy_state'][0],
                                                         accel_group=accel_group))
        add_state_sub_menu.append(create_image_menu_item("Preemptive State", constants.BUTTON_ADD,
                                                         self.on_add_preemptive_state_activate,
                                                         accel_code=shortcuts_dict['add_preemptive_state'][0],
                                                         accel_group=accel_group))
        add_state_sub_menu.append(create_image_menu_item("Barrier State", constants.BUTTON_ADD,
                                                         self.on_add_barrier_state_activate,
                                                         accel_code=shortcuts_dict['add_barrier_state'][0],
                                                         accel_group=accel_group))

        add_sub_menu.append(gtk.SeparatorMenuItem())

        add_sub_menu.append(create_image_menu_item("Outcome", constants.BUTTON_ADD, self.on_add_outcome,
                                                   accel_code=shortcuts_dict['add_outcome'][0], accel_group=accel_group))
        add_sub_menu.append(create_image_menu_item("Output Port", constants.BUTTON_ADD, self.on_add_output,
                                                   accel_code=shortcuts_dict['add_output'][0], accel_group=accel_group))
        add_sub_menu.append(create_image_menu_item("Input Port", constants.BUTTON_ADD, self.on_add_input,
                                                   accel_code=shortcuts_dict['add_input'][0], accel_group=accel_group))
        add_sub_menu.append(create_image_menu_item("Scoped Variable", constants.BUTTON_ADD, self.on_add_scoped_variable,
                                                   accel_code=shortcuts_dict['add_scoped_variable'][0],
                                                   accel_group=accel_group))
        menu.append(gtk.SeparatorMenuItem())

        self.insert_copy_cut_paste_in_menu(menu, shortcuts_dict, accel_group)

        from rafcon.core.states.barrier_concurrency_state import DeciderState
        state_m_list = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
        all_m_list = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_all()
        if all([isinstance(elem, (AbstractStateModel, ScopedVariableModel)) for elem in all_m_list]) and \
                not any([state_m.state.is_root_state for state_m in state_m_list]):
            menu.append(create_image_menu_item("Group states", constants.BUTTON_GROUP, self.on_group_states_activate,
                                               accel_code=shortcuts_dict['group'][0], accel_group=accel_group))
        if len(state_m_list) == 1 and isinstance(state_m_list[0], AbstractStateModel):
            if isinstance(state_m_list[0], ContainerStateModel) and not state_m_list[0].state.is_root_state:
                menu.append(create_image_menu_item("Ungroup states", constants.BUTTON_UNGR,
                                                   self.on_ungroup_state_activate,
                                                   accel_code=shortcuts_dict['ungroup'][0], accel_group=accel_group))
            if not isinstance(state_m_list[0].state, DeciderState) and not state_m_list[0].state.is_root_state:
                menu.append(create_image_menu_item("Substitute state", constants.BUTTON_REFR,
                                                   self.on_substitute_state_activate,
                                                   accel_code=shortcuts_dict['substitute_state'][0],
                                                   accel_group=accel_group))

            from rafcon.gui.controllers.state_editor.overview import StateOverviewController
            state_type_class_dict = StateOverviewController.change_state_type_class_dict(state_m_list[0].state)
            if len(state_type_class_dict) > 1:
                change_type_sub_menu_item, change_type_sub_menu = append_sub_menu_to_parent_menu("Change state type",
                                                                                                 menu,
                                                                                                 constants.BUTTON_EXCHANGE)
                for class_key, item in state_type_class_dict.iteritems():
                    class_item = create_image_menu_item(class_key, constants.SIGN_LIB,
                                                        partial(self.on_type_change_activate,
                                                                target_class=item['class']),
                                                                accel_code=None,
                                                                accel_group=accel_group)
                    if isinstance(state_m_list[0].state, item['class']):
                        class_item.set_sensitive(False)
                    change_type_sub_menu.append(class_item)
        menu.append(gtk.SeparatorMenuItem())

        save_as_sub_menu_item, save_as_sub_menu = append_sub_menu_to_parent_menu("Save state as", menu,
                                                                                 constants.BUTTON_SAVE)

        save_as_sub_menu.append(create_image_menu_item("State machine", constants.BUTTON_SAVE,
                                                       self.on_save_state_as_state_machine_activate,
                                                       accel_code=shortcuts_dict['save_state_as'][0],
                                                       accel_group=accel_group))
        save_as_library_sub_menu_item, save_as_library_sub_menu = append_sub_menu_to_parent_menu("Library",
                                                                                                 save_as_sub_menu,
                                                                                                 constants.SIGN_LIB)
        library_root_paths = core_singletons.library_manager.library_root_paths
        for library_root_key in library_root_paths.iterkeys():
            save_as_library_sub_menu.append(create_image_menu_item(library_root_key, constants.SIGN_LIB,
                                                                   partial(self.on_save_state_as_state_machine_activate,
                                                                           path=library_root_paths[library_root_key]),
                                                                   accel_code=None, accel_group=accel_group))

        return menu

    def insert_is_start_state_in_menu(self, menu, shortcuts_dict, accel_group):

        state_m_list = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()
        has_no_start_state_state_types = (BarrierConcurrencyState, PreemptiveConcurrencyState)
        if len(state_m_list) == 1 and isinstance(state_m_list[0], AbstractStateModel) and \
                not state_m_list[0].state.is_root_state and \
                not isinstance(state_m_list[0].parent.state, has_no_start_state_state_types):
            menu.append(create_check_menu_item("Is start state", state_m_list[0].is_start, self.on_toggle_is_start_state,
                                               accel_code=shortcuts_dict['is_start_state'][0], accel_group=accel_group))

    def insert_execution_sub_menu_in_menu(self, menu, shortcuts_dict, accel_group):
        execution_sub_menu_item, execution_sub_menu = append_sub_menu_to_parent_menu("Execution", menu,
                                                                                 constants.BUTTON_EXP)
        execution_sub_menu.append(create_image_menu_item("from here", constants.BUTTON_START_FROM_SELECTED_STATE,
                                                         self.on_run_from_selected_state_activate,
                                                         accel_code=shortcuts_dict['start_from_selected'][0],
                                                         accel_group=accel_group))
        execution_sub_menu.append(create_image_menu_item("stop here", constants.BUTTON_RUN_TO_SELECTED_STATE,
                                                         self.on_run_to_selected_state_activate,
                                                         accel_code=shortcuts_dict['run_to_selected'][0],
                                                         accel_group=accel_group))

    def insert_copy_cut_paste_in_menu(self, menu, shortcuts_dict, accel_group, no_paste=False):
        menu.append(create_image_menu_item("Copy selection", constants.BUTTON_COPY, self.on_copy_activate,
                                           accel_code=shortcuts_dict['copy'][0], accel_group=accel_group))
        menu.append(create_image_menu_item("Paste selection", constants.BUTTON_PASTE, self.on_paste_activate,
                                           accel_code=shortcuts_dict['paste'][0], accel_group=accel_group))
        if not no_paste:
            menu.append(create_image_menu_item("Cut selection", constants.BUTTON_CUT, self.on_cut_activate,
                                               accel_code=shortcuts_dict['cut'][0], accel_group=accel_group))

    def generate_right_click_menu_library(self):
        menu = gtk.Menu()
        accel_group = self.accel_group
        shortcuts_dict = global_gui_config.get_config_value('SHORTCUTS')

        self.insert_is_start_state_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_execution_sub_menu_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_copy_cut_paste_in_menu(menu, shortcuts_dict, accel_group, no_paste=True)

        menu.append(create_image_menu_item("Group states", constants.BUTTON_GROUP, self.on_group_states_activate,
                                           accel_code=shortcuts_dict['group'][0], accel_group=accel_group))

        menu.append(create_image_menu_item("Open separately", constants.BUTTON_OPEN,
                                           self.on_open_activate,
                                           accel_code=None, accel_group=accel_group))

        menu.append(create_image_menu_item("Substitute state", constants.BUTTON_REFR,
                                           self.on_substitute_state_activate,
                                           accel_code=shortcuts_dict['substitute_state'][0], accel_group=accel_group))
        menu.append(create_image_menu_item("Substitute library with template", constants.BUTTON_REFR,
                                           self.on_substitute_library_with_template_activate))

        return menu

    def on_toggle_is_start_state(self, widget, data=None):
        self.shortcut_manager.trigger_action("is_start_state", None, None)

    def on_add_execution_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_execution_state', None, None)

    def on_add_hierarchy_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_hierarchy_state', None, None)

    def on_add_preemptive_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_preemptive_state', None, None)

    def on_add_barrier_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_barrier_state', None, None)

    def on_add_outcome(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_outcome', None, None)

    def on_add_output(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_output', None, None)

    def on_add_input(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_input', None, None)

    def on_add_scoped_variable(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_scoped_variable', None, None)

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger default copy")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.copy(active_sm_m.selection)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger default paste")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.paste(active_sm_m.selection.get_states()[0])

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger default cut")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.cut(active_sm_m.selection)

    def on_group_states_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("group", None, None)

    def on_ungroup_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("ungroup", None, None)

    def on_save_as_state_machine_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("save_as_sm", None, None)

    def on_run_from_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('start_from_selected', None, None)

    def on_run_to_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('run_to_selected', None, None)

    @staticmethod
    def on_save_state_as_state_machine_activate(widget, data=None, path=None):
        # TODO if the state is a root state the state machine will get chosen path a physical system path?!
        # workaround to set the initial path in the 'choose folder' dialog to the handed one
        old_last_path_open = gui_singletons.global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
        try:
            if path is not None:
                gui_singletons.global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
                gui_helper_state.save_selected_state_as()
            else:
                gui_helper_state.save_selected_state_as()
        except Exception:
            raise
        finally:
            # secure that the last path open is reset
            gui_singletons.global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', old_last_path_open)

    @staticmethod
    def on_open_activate(widget, data=None):
        state_m = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_states()[0]
        path, _, _ = gui_singletons.library_manager.get_os_path_to_library(state_m.state.library_path,
                                                                           state_m.state.library_name)
        gui_helper_state_machine.open_state_machine(path)

    def on_substitute_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('substitute_state', None, None)

    def on_substitute_library_with_template_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('substitute_library_with_template', None, None)

    def on_type_change_activate(self, widget, data=None, target_class=None):
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection.get_all()) == 1 and len(selection.get_states()) == 1:
            gui_helper_state.change_state_type(selection.get_states()[0], target_class)

    def mouse_click(self, widget, event=None):
        from rafcon.gui.models.library_state import LibraryStateModel
        # logger.info("Single right click -> selection is \n{0}"
        #             "".format(gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_all()))
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
            if len(selection.get_all()) == 1 and len(selection.get_states()) == 1 and \
                    isinstance(selection.get_states()[0], LibraryStateModel):
                menu = self.generate_right_click_menu_library()
            else:
                menu = self.generate_right_click_menu_state()
            menu.show_all()
            return self.activate_menu(event, menu)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        menu.popup(None, None, None, event.button, event.time)
        return True


class StateMachineRightClickMenuController(ExtendedController, StateMachineRightClickMenu):

    def __init__(self, model=None, view=None):
        ExtendedController.__init__(self, model, view)
        StateMachineRightClickMenu.__init__(self)

    def register_view(self, view):
        raise NotImplementedError


class StateMachineTreeRightClickMenuController(StateMachineRightClickMenuController):

    def register_view(self, view):
        view.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        pthinfo = self.view.get_path_at_pos(int(event.x), int(event.y))

        if pthinfo is not None:
            path, col, cellx, celly = pthinfo
            self.view.grab_focus()
            self.view.set_cursor(path, col, 0)

            menu.popup(None, None, None, event.button, event.time)
        return True


class StateRightClickMenuControllerOpenGLEditor(StateMachineRightClickMenuController):

    def register_view(self, view):
        from rafcon.gui.views.graphical_editor import GraphicalEditorView
        assert isinstance(view, GraphicalEditorView)
        view.editor.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if selection.get_num_states() > 0 or selection.get_num_scoped_variables() > 0:
            menu.popup(None, None, None, event.button, event.time)
            return True
        else:
            return False

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger opengl copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger opengl paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger opengl cut")
        self.shortcut_manager.trigger_action("cut", None, None)


class StateRightClickMenuGaphas(StateMachineRightClickMenu):

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if selection.get_num_states() > 0 or selection.get_num_scoped_variables() > 0:
            menu.popup(None, None, None, event.button, event.time)
            return True
        else:
            return False

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger gaphas copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger gaphas paste")
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger gaphas cut")
        self.shortcut_manager.trigger_action("cut", None, None)
