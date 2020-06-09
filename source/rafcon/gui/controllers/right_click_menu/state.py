# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_machine_right_click_menu
   :synopsis: The module provides a menu by right click to edit states or its elements.

"""

from builtins import object
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
from functools import partial

import rafcon.core.singleton as core_singletons
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
import rafcon.gui.singleton as gui_singletons
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.config import global_gui_config
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import create_menu_item, create_check_menu_item, append_sub_menu_to_parent_menu
from rafcon.gui.models import AbstractStateModel, ContainerStateModel, LibraryStateModel, ScopedVariableModel, \
    TransitionModel, DataFlowModel
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)

# TODO module needs refactoring, right naming, to separate into capsuled modules and re-arrangement
# TODO  -> to be more useful for more right click extension


class StateMachineRightClickMenu(object):

    """ A class for handling all right click menus inside rafcon.
    Examples are the right click menu of the state machine tree or the gaphas editor.

    """

    menu_position = None

    def __init__(self, state_machine_manager_model=None):
        if state_machine_manager_model is None:
            from rafcon.gui.singleton import state_machine_manager_model
        self.state_machine_manager_model = state_machine_manager_model
        from rafcon.gui.singleton import main_window_controller
        shortcut_manager = main_window_controller.shortcut_manager
        self.shortcut_manager = shortcut_manager
        self.accel_group = Gtk.AccelGroup()

    def generate_right_click_menu_state(self):
        menu = Gtk.Menu()
        accel_group = self.accel_group
        shortcuts_dict = global_gui_config.get_config_value('SHORTCUTS')

        self.insert_is_start_state_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_execution_sub_menu_in_menu(menu, shortcuts_dict, accel_group)

        menu.append(Gtk.SeparatorMenuItem())

        add_sub_menu_item, add_sub_menu = append_sub_menu_to_parent_menu("Add", menu, constants.BUTTON_ADD)

        add_state_sub_menu_item, add_state_sub_menu = append_sub_menu_to_parent_menu("State", add_sub_menu,
                                                                                     constants.BUTTON_ADD)

        add_state_sub_menu.append(create_menu_item("Execution State", constants.BUTTON_ADD,
                                                   self.on_add_execution_state_activate,
                                                   accel_code=shortcuts_dict['add_execution_state'][0],
                                                   accel_group=accel_group))
        add_state_sub_menu.append(create_menu_item("Hierarchy State", constants.BUTTON_ADD,
                                                   self.on_add_hierarchy_state_activate,
                                                   accel_code=shortcuts_dict['add_hierarchy_state'][0],
                                                   accel_group=accel_group))
        add_state_sub_menu.append(create_menu_item("Preemptive State", constants.BUTTON_ADD,
                                                   self.on_add_preemptive_state_activate,
                                                   accel_code=shortcuts_dict['add_preemptive_state'][0],
                                                   accel_group=accel_group))
        add_state_sub_menu.append(create_menu_item("Barrier State", constants.BUTTON_ADD,
                                                   self.on_add_barrier_state_activate,
                                                   accel_code=shortcuts_dict['add_barrier_state'][0],
                                                   accel_group=accel_group))

        add_sub_menu.append(Gtk.SeparatorMenuItem())

        add_sub_menu.append(create_menu_item("Outcome", constants.BUTTON_ADD, self.on_add_outcome,
                                             accel_code=shortcuts_dict['add_outcome'][0], accel_group=accel_group))
        add_sub_menu.append(create_menu_item("Output Port", constants.BUTTON_ADD, self.on_add_output,
                                             accel_code=shortcuts_dict['add_output'][0], accel_group=accel_group))
        add_sub_menu.append(create_menu_item("Input Port", constants.BUTTON_ADD, self.on_add_input,
                                             accel_code=shortcuts_dict['add_input'][0], accel_group=accel_group))
        add_sub_menu.append(create_menu_item("Scoped Variable", constants.BUTTON_ADD, self.on_add_scoped_variable,
                                             accel_code=shortcuts_dict['add_scoped_variable'][0],
                                             accel_group=accel_group))
        menu.append(Gtk.SeparatorMenuItem())

        self.insert_copy_cut_paste_in_menu(menu, shortcuts_dict, accel_group)

        from rafcon.core.states.barrier_concurrency_state import DeciderState
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        selected_state_m = selection.get_selected_state()
        all_m_list = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_all()
        # logger.info("Element in selection: " + str(selected_state_m) + " : " + str([elem for elem in all_m_list]))
        if any([isinstance(elem, (AbstractStateModel, ScopedVariableModel)) for elem in all_m_list]) and \
                all([isinstance(elem, (AbstractStateModel, ScopedVariableModel, TransitionModel, DataFlowModel))
                     for elem in all_m_list]) and \
                all([not state_m.state.is_root_state for state_m in selection.states]):
            menu.append(create_menu_item("Group states", constants.BUTTON_GROUP, self.on_group_states_activate,
                                         accel_code=shortcuts_dict['group'][0], accel_group=accel_group))
        if len(selection.states) == 1:
            if isinstance(selected_state_m, ContainerStateModel) and not selected_state_m.state.is_root_state:
                menu.append(create_menu_item("Ungroup states", constants.BUTTON_UNGR,
                                             self.on_ungroup_state_activate,
                                             accel_code=shortcuts_dict['ungroup'][0], accel_group=accel_group))
            if not isinstance(selected_state_m.state, DeciderState) and not selected_state_m.state.is_root_state:
                menu.append(create_menu_item("Substitute state", constants.BUTTON_REFR,
                                             self.on_substitute_state_activate,
                                             accel_code=shortcuts_dict['substitute_state'][0],
                                             accel_group=accel_group))

            from rafcon.gui.controllers.state_editor.overview import StateOverviewController
            allowed_state_classes = StateOverviewController.get_allowed_state_classes(selected_state_m.state)
            if len(allowed_state_classes) > 1:
                change_type_sub_menu_item, change_type_sub_menu = append_sub_menu_to_parent_menu("Change state type",
                                                                                                 menu,
                                                                                                 constants.BUTTON_EXCHANGE)
                for state_class in allowed_state_classes:
                    callback_function = partial(self.on_type_change_activate, target_class=state_class)
                    class_item = create_menu_item(state_class.__name__, constants.SIGN_LIB,
                                                  callback_function,
                                                  accel_code=None, accel_group=accel_group)

                    if isinstance(selected_state_m.state, state_class):
                        class_item.set_sensitive(False)
                    change_type_sub_menu.append(class_item)
        menu.append(Gtk.SeparatorMenuItem())

        # save state as but not for root state, therefore the user should use save state machine as
        if len(selection.states) == 1 and not selected_state_m.state.is_root_state:
            save_as_sub_menu_item, save_as_sub_menu = append_sub_menu_to_parent_menu("Save state as", menu,
                                                                                     constants.BUTTON_SAVE)
            callback_function = partial(self.on_save_as_activate,
                                        save_as_function=gui_helper_state_machine.save_selected_state_as)
            save_as_sub_menu.append(create_menu_item("State machine", constants.BUTTON_SAVE,
                                                     callback_function,
                                                     accel_code=shortcuts_dict['save_state_as'][0],
                                                     accel_group=accel_group))
            save_as_library_sub_menu_item, save_as_library_sub_menu = append_sub_menu_to_parent_menu("Library",
                                                                                                     save_as_sub_menu,
                                                                                                     constants.SIGN_LIB)
            library_root_paths = core_singletons.library_manager.library_root_paths
            for library_root_key in library_root_paths.keys():
                callback_function = partial(self.on_save_as_activate,
                                            path=library_root_paths[library_root_key],
                                            save_as_function=gui_helper_state_machine.save_selected_state_as)
                save_as_library_sub_menu.append(create_menu_item(library_root_key, constants.SIGN_LIB,
                                                                 callback_function,
                                                                 accel_code=None, accel_group=accel_group))
        else:
            save_as_sub_menu_item, save_as_sub_menu = append_sub_menu_to_parent_menu("Save state machine as", menu,
                                                                                     constants.BUTTON_SAVE)

            callback_function = partial(self.on_save_as_activate,
                                        save_as_function=gui_helper_state_machine.save_state_machine_as)
            save_as_sub_menu.append(create_menu_item("State machine", constants.BUTTON_SAVE,
                                                     callback_function,
                                                     accel_code=shortcuts_dict['save_as'][0],
                                                     accel_group=accel_group))
            save_as_library_sub_menu_item, save_as_library_sub_menu = append_sub_menu_to_parent_menu("Library",
                                                                                                     save_as_sub_menu,
                                                                                                     constants.SIGN_LIB)
            library_root_paths = core_singletons.library_manager.library_root_paths
            for library_root_key in library_root_paths.keys():
                callback_function = partial(self.on_save_as_activate,
                                            path=library_root_paths[library_root_key],
                                            save_as_function=gui_helper_state_machine.save_state_machine_as)
                save_as_library_sub_menu.append(create_menu_item(library_root_key, constants.SIGN_LIB,
                                                                 callback_function,
                                                                 accel_code=None, accel_group=accel_group))

        return menu

    def insert_show_library_content_in_menu(self, menu, shortcuts_dict, accel_group):

        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        selected_state_m = selection.get_selected_state()
        if len(selection.states) == 1 and isinstance(selected_state_m, LibraryStateModel):
            menu.append(create_check_menu_item("Show library content", selected_state_m.meta['gui']['show_content'],
                                               partial(self.on_toggle_show_library_content, state_m=selected_state_m)))
            menu.append(create_menu_item("Show library tree element", constants.SIGN_LIB,
                                         partial(self.on_select_library_tree_element, state_m=selected_state_m)))

    def insert_is_start_state_in_menu(self, menu, shortcuts_dict, accel_group):

        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        selected_state_m = selection.get_selected_state()
        has_no_start_state_state_types = (BarrierConcurrencyState, PreemptiveConcurrencyState)
        if len(selection.states) == 1 and \
                not selected_state_m.state.is_root_state and \
                not isinstance(selected_state_m.parent.state, has_no_start_state_state_types):
            menu.append(create_check_menu_item("Is start state", selected_state_m.is_start, self.on_toggle_is_start_state,
                                               accel_code=shortcuts_dict['is_start_state'][0], accel_group=accel_group))

    def insert_execution_sub_menu_in_menu(self, menu, shortcuts_dict, accel_group):
        execution_sub_menu_item, execution_sub_menu = append_sub_menu_to_parent_menu("Execution", menu,
                                                                                 constants.BUTTON_START)
        execution_sub_menu.append(create_menu_item("from here", constants.BUTTON_START_FROM_SELECTED_STATE,
                                                   self.on_run_from_selected_state_activate,
                                                   accel_code=shortcuts_dict['start_from_selected'][0],
                                                   accel_group=accel_group))
        execution_sub_menu.append(create_menu_item("stop here", constants.BUTTON_RUN_TO_SELECTED_STATE,
                                                   self.on_run_to_selected_state_activate,
                                                   accel_code=shortcuts_dict['run_to_selected'][0],
                                                   accel_group=accel_group))

    def insert_copy_cut_paste_in_menu(self, menu, shortcuts_dict, accel_group, no_paste=False):
        menu.append(create_menu_item("Copy selection", constants.BUTTON_COPY, self.on_copy_activate,
                                     accel_code=shortcuts_dict['copy'][0], accel_group=accel_group))
        menu.append(create_menu_item("Paste selection", constants.BUTTON_PASTE, self.on_paste_activate,
                                     accel_code=shortcuts_dict['paste'][0], accel_group=accel_group))
        if not no_paste:
            menu.append(create_menu_item("Cut selection", constants.BUTTON_CUT, self.on_cut_activate,
                                         accel_code=shortcuts_dict['cut'][0], accel_group=accel_group))

    def generate_right_click_menu_library(self):
        menu = Gtk.Menu()
        accel_group = self.accel_group
        shortcuts_dict = global_gui_config.get_config_value('SHORTCUTS')

        self.insert_show_library_content_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_is_start_state_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_execution_sub_menu_in_menu(menu, shortcuts_dict, accel_group)

        self.insert_copy_cut_paste_in_menu(menu, shortcuts_dict, accel_group, no_paste=True)

        menu.append(create_menu_item("Group states", constants.BUTTON_GROUP, self.on_group_states_activate,
                                     accel_code=shortcuts_dict['group'][0], accel_group=accel_group))

        menu.append(create_menu_item("Open separately", constants.BUTTON_OPEN,
                                     self.on_open_library_state_separately_activate,
                                     accel_code=shortcuts_dict['open_library_state_separately'][0],
                                     accel_group=accel_group))

        menu.append(create_menu_item("Substitute state with library", constants.BUTTON_REFR,
                                     self.on_substitute_state_activate,
                                     accel_code=shortcuts_dict['substitute_state'][0], accel_group=accel_group))

        sub_menu_item, sub_menu = append_sub_menu_to_parent_menu("Substitute library with template", menu,
                                                                 constants.BUTTON_REFR)
        sub_menu.append(create_menu_item("Keep state name", constants.BUTTON_LEFTA,
                                         partial(self.on_substitute_library_with_template_activate, keep_name=True)))

        sub_menu.append(create_menu_item("Take name from library", constants.BUTTON_EXCHANGE,
                                         partial(self.on_substitute_library_with_template_activate, keep_name=False)))

        return menu

    @staticmethod
    def on_toggle_show_library_content(widget, date=None, state_m=None):
        if state_m is not None:
            gui_helper_state_machine.gui_helper_state.toggle_show_content_flag_of_library_state_model(state_m)


    def on_select_library_tree_element(widget, date=None, state_m=None):
        from rafcon.gui.singleton import main_window_controller
        library_tree_controller = main_window_controller.get_controller('library_controller')
        library_tree_controller.select_library_tree_element_of_library_state_model(state_m)

    def on_toggle_is_start_state(self, widget, data=None):
        self.shortcut_manager.trigger_action("is_start_state", None, None)

    def on_add_execution_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_execution_state', None, None, cursor_position=self.menu_position)

    def on_add_hierarchy_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_hierarchy_state', None, None, cursor_position=self.menu_position)

    def on_add_preemptive_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_preemptive_state', None, None, cursor_position=self.menu_position)

    def on_add_barrier_state_activate(self, widget=None, data=None):
        self.shortcut_manager.trigger_action('add_barrier_state', None, None, cursor_position=self.menu_position)

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
        global_clipboard.paste(active_sm_m.selection.get_selected_state())

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger default cut")
        active_sm_m = self.state_machine_manager_model.get_selected_state_machine_model()
        global_clipboard.cut(active_sm_m.selection)

    def on_group_states_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("group", None, None)

    def on_ungroup_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("ungroup", None, None)

    def on_run_from_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('start_from_selected', None, None)

    def on_run_to_selected_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('run_to_selected', None, None)

    @staticmethod
    def on_save_as_activate(widget, data=None, path=None, save_as_function=None):
        # workaround to set the initial path in the 'choose folder' dialog to the handed one
        old_last_path_open = gui_singletons.global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
        if save_as_function is None:
            logger.error("Hand a function for operation 'save_as' currently it is '{0}'".format(save_as_function))
            return

        try:
            if path is not None:
                gui_singletons.global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
            save_as_function()
        except Exception:
            raise
        finally:
            # secure that the last path open is reset
            gui_singletons.global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', old_last_path_open)

    @staticmethod
    def on_open_library_state_separately_activate(widget, data=None):
        gui_helper_state_machine.open_library_state_separately()

    def on_substitute_state_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action('substitute_state', None, None)

    @staticmethod
    def on_substitute_library_with_template_activate(widget, data=None, keep_name=False):
        gui_helper_state_machine.substitute_selected_library_state_with_template(keep_name)

    @staticmethod
    def on_type_change_activate(widget, data=None, target_class=None):
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection) == 1 and len(selection.states) == 1:
            gui_helper_state_machine.change_state_type_with_error_handling_and_logger_messages(selection.get_selected_state(),
                                                                                               target_class)

    def mouse_click(self, widget, event=None):
        from rafcon.gui.models.library_state import LibraryStateModel
        # logger.info("Single right click -> selection is \n{0}"
        #             "".format(gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection.get_all()))
        if event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 3:
            selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
            if len(selection) == 1 and len(selection.states) == 1 and \
                    isinstance(selection.get_selected_state(), LibraryStateModel):
                menu = self.generate_right_click_menu_library()
            else:
                menu = self.generate_right_click_menu_state()
            menu.show_all()
            return self.activate_menu(event, menu)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        menu.popup(None, None, None, None, event.get_button()[1], event.time)
        return True


class StateMachineRightClickMenuController(ExtendedController, StateMachineRightClickMenu):

    def __init__(self, model=None, view=None):
        ExtendedController.__init__(self, model, view)
        StateMachineRightClickMenu.__init__(self)

    def register_view(self, view):
        raise NotImplementedError


class StateMachineTreeRightClickMenuController(StateMachineRightClickMenuController):

    def register_view(self, view):
        ExtendedController.register_view(self, view)
        view.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        pthinfo = self.view.get_path_at_pos(int(event.x), int(event.y))

        if pthinfo is not None:
            path, col, cellx, celly = pthinfo
            self.view.grab_focus()
            self.view.set_cursor(path, col, 0)

            menu.popup(None, None, None, None, event.get_button()[1], event.time)
        return True


class StateRightClickMenuControllerOpenGLEditor(StateMachineRightClickMenuController):

    def register_view(self, view):
        ExtendedController.register_view(self, view)
        from rafcon.gui.views.graphical_editor import GraphicalEditorView
        assert isinstance(view, GraphicalEditorView)
        view.editor.connect('button_press_event', self.mouse_click)

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection.states) > 0 or len(selection.scoped_variables) > 0:
            menu.popup(None, None, None, None, event.get_button()[1], event.time)
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
    """ A class for handling the right click menu inside gaphas

    """

    def activate_menu(self, event, menu):
        # logger.info("activate_menu by " + self.__class__.__name__)
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection.states) > 0 or len(selection.scoped_variables) > 0:
            from rafcon.gui.helpers.coordinates import screen2main_window
            menu.popup(None, None, None, None, event.get_button()[1], event.time)
            # The pointer in screen coordinates.
            pointer = menu.get_root_window().get_pointer()
            self.menu_position = screen2main_window((pointer.x, pointer.y))
            return True
        else:
            return False

    def on_copy_activate(self, widget, data=None):
        # logger.info("trigger gaphas copy")
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_activate(self, widget, data=None):
        # logger.info("trigger gaphas paste")
        self.shortcut_manager.trigger_action("paste", None, None, cursor_position=self.menu_position)

    def on_cut_activate(self, widget, data=None):
        # logger.info("trigger gaphas cut")
        self.shortcut_manager.trigger_action("cut", None, None)
