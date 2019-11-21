# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import range
import os
from gi.repository import Pango

from gtkmvc3.view import View

import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui import glade
from rafcon.gui.utils import constants
from rafcon.gui.views.execution_history import ExecutionHistoryView
from rafcon.gui.views.global_variable_editor import GlobalVariableEditorView
from rafcon.gui.views.library_tree import LibraryTreeView
from rafcon.gui.views.notification_bar import NotificationBarView
from rafcon.gui.views.debug_console import DebugConsoleView
from rafcon.gui.views.menu_bar import MenuBarView
from rafcon.gui.views.modification_history import ModificationHistoryView
from rafcon.gui.views.state_icons import StateIconView
from rafcon.gui.views.state_machine_tree import StateMachineTreeView
from rafcon.gui.views.state_machines_editor import StateMachinesEditorView
from rafcon.gui.views.states_editor import StatesEditorView
from rafcon.gui.views.tool_bar import ToolBarView
from rafcon.gui.views.undocked_window import UndockedWindowView


class MainWindowView(View):
    builder = glade.get_glade_path("main_window.glade")
    top = 'main_window'

    def __init__(self):
        View.__init__(self)

        if os.getenv("RAFCON_START_MINIMIZED", False):
            self.get_top_widget().iconify()

        # Add gui components by removing their corresponding placeholders defined in the glade file first and then
        # adding the widgets.
        self.left_bar_notebooks = [self['upper_notebook'], self['lower_notebook']]

        ################################################
        # Undock Buttons
        ################################################
        self['undock_left_bar_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_UNDOCK))
        self['undock_left_bar_button'].set_tooltip_text("Undock left side bar widget")
        self['undock_right_bar_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_UNDOCK))
        self['undock_right_bar_button'].set_tooltip_text("Undock right side bar widget")
        self['collapse_tree_button'].set_image(gui_helper_label.create_button_label(constants.ICON_TREE_FOLD))
        self['collapse_tree_button'].set_tooltip_text("Collapse tree of widget")

        ######################################################
        # Library Tree
        ######################################################
        self.library_tree = LibraryTreeView()
        self.library_tree.show()
        self['libraries_scrolledwindow'].add(self.library_tree)

        ######################################################
        # State Icons
        ######################################################
        self.state_icons = StateIconView()
        self.state_icons.show()
        self["state_icons_box"].pack_start(self.state_icons.get_top_widget(), True, True, 0)

        ######################################################
        # State Machine Tree
        ######################################################
        self.state_machine_tree = StateMachineTreeView()
        self.state_machine_tree.show()
        self['states_tree_scrolledwindow'].add(self.state_machine_tree)

        ######################################################
        # Global Variable Manager
        ######################################################
        self.global_var_editor = GlobalVariableEditorView()
        self.global_var_editor.show()
        self['global_variables_eventbox'].add(self.global_var_editor.get_top_widget())

        ######################################################
        # State Machine History
        ######################################################
        self.state_machine_history = ModificationHistoryView()
        self.state_machine_history.show()
        self['history_alignment'].add(self.state_machine_history.get_top_widget())

        ######################################################
        # State Machine Execution History
        ######################################################
        self.execution_history = ExecutionHistoryView()
        self.execution_history.show()
        self['execution_history_alignment'].add(self.execution_history.get_top_widget())

        ######################################################
        # rotate all tab labels by 90 degrees and make detachable
        ######################################################
        self.rotate_and_detach_tab_labels()

        self['upper_notebook'].set_current_page(0)
        self['lower_notebook'].set_current_page(0)

        ######################################################
        # State-machines-editor (graphical)
        ######################################################
        self.state_machines_editor = StateMachinesEditorView()
        self.state_machines_editor.show()
        self['central_vbox'].pack_start(self.state_machines_editor.get_top_widget(), True, True, 0)
        self['central_vbox'].reorder_child(self.state_machines_editor.get_top_widget(), 1)

        ######################################################
        # Notification Bar
        ######################################################
        self.notification_bar = NotificationBarView()
        self.notification_bar.show()
        self['central_vbox'].pack_start(self.notification_bar.get_top_widget(), False, True, 0)
        self['central_vbox'].reorder_child(self.notification_bar.get_top_widget(), 2)

        ######################################################
        # States-editor
        ######################################################
        self.states_editor = StatesEditorView()
        self['state_editor_eventbox'].add(self.states_editor.get_top_widget())
        self.states_editor.show()

        ######################################################
        # Debug Console
        ######################################################
        self.debug_console_view = DebugConsoleView()
        self['debug_console_viewport'].add(self.debug_console_view.get_top_widget())
        self.debug_console_view.get_top_widget().show()
        # map hide and undock buttons within and debug widget to be usable from main window view with generic naming
        self['undock_console_button'] = self.debug_console_view['undock_console_button']
        self['console_hide_button'] = self.debug_console_view['console_hide_button']
        self['console_container'] = self.debug_console_view['console_container']
        self['console'] = self.debug_console_view['console']

        ##################################################
        # HeaderBar with MenuBar
        ##################################################

        self.menu_bar = MenuBarView(self)
        self.menu_bar.show()

        self['headerbar'].pack_start(self.menu_bar.get_top_widget())
        self['headerbar'].show()

        self.tool_bar = ToolBarView()
        self.tool_bar.show()
        self['top_level_vbox'].remove(self['tool_bar_placeholder'])
        self['top_level_vbox'].pack_start(self.tool_bar.get_top_widget(), expand=False, fill=True, padding=0)
        self['top_level_vbox'].reorder_child(self.tool_bar.get_top_widget(), 0)

        ################################################
        # Hide Buttons
        ################################################
        self['left_bar_hide_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_LEFTA))
        self['right_bar_hide_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_RIGHTA))

        ################################################
        # Return Buttons
        ################################################
        self['left_bar_return_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_RIGHTA))
        self['right_bar_return_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_LEFTA))
        self['console_return_button'].set_image(gui_helper_label.create_button_label(constants.BUTTON_UPA))

        # --------------------------------------------------------------------------
        # Edit graphical_editor_shortcuts
        # --------------------------------------------------------------------------

        button_start_shortcut = self['button_start_shortcut']
        button_start_shortcut.set_tooltip_text('Run')
        button_stop_shortcut = self['button_stop_shortcut']
        button_stop_shortcut.set_tooltip_text('Stop')
        button_pause_shortcut = self['button_pause_shortcut']
        button_pause_shortcut.set_tooltip_text('Pause')
        button_start_from_shortcut = self['button_start_from_shortcut']
        button_start_from_shortcut.set_tooltip_text('Run From Selected State')
        button_run_to_shortcut = self['button_run_to_shortcut']
        button_run_to_shortcut.set_tooltip_text('Run Until Selected State (Selected State Excluded)')
        button_step_mode_shortcut = self['button_step_mode_shortcut']
        button_step_mode_shortcut.set_tooltip_text('Enter Step Mode')
        button_step_in_shortcut = self['button_step_in_shortcut']
        button_step_in_shortcut.set_tooltip_text('Step Into (One Level In -> Child-State))')
        button_step_over_shortcut = self['button_step_over_shortcut']
        button_step_over_shortcut.set_tooltip_text('Step Over (the next Sibling-State))')
        button_step_out_shortcut = self['button_step_out_shortcut']
        button_step_out_shortcut.set_tooltip_text('Step Out (One Level Up -> Parent-State)')
        button_step_backward_shortcut = self['button_step_backward_shortcut']
        button_step_backward_shortcut.set_tooltip_text('Step Backward')

        button_start_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_START))
        button_stop_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_STOP))
        button_pause_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_PAUSE))
        button_start_from_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_START_FROM_SELECTED_STATE))
        button_run_to_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_RUN_TO_SELECTED_STATE))
        button_step_mode_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_STEPM))
        button_step_in_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_STEP_INTO))
        button_step_over_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_STEP_OVER))
        button_step_out_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_STEP_OUT))
        button_step_backward_shortcut.set_label_widget(gui_helper_label.create_button_label(constants.BUTTON_BACKW))

        # --------------------------------------------------------------------------

        # Gtk TODO: find replacement for methods set_tab_hborder and set_tab_vborder
        # self['upper_notebook'].set_tab_hborder(constants.TAB_BORDER_WIDTH * 2)
        # self['upper_notebook'].set_tab_vborder(constants.TAB_BORDER_WIDTH * 3)
        # if global_gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
        #     self['lower_notebook'].set_tab_hborder(int(constants.TAB_BORDER_WIDTH * 2 / 1.4))
        # else:
        #     self['lower_notebook'].set_tab_hborder(constants.TAB_BORDER_WIDTH * 2)
        # self['lower_notebook'].set_tab_vborder(constants.TAB_BORDER_WIDTH * 3)

        self.left_bar_window = UndockedWindowView('left_bar_window')
        self.right_bar_window = UndockedWindowView('right_bar_window')
        self.console_window = UndockedWindowView('console_window')

        gui_helper_label.ellipsize_labels_recursively(self['execution_ticker_text'], Pango.EllipsizeMode.START)

    def rotate_and_detach_tab_labels(self):
        """Rotates tab labels of a given notebook by 90 degrees and makes them detachable.

        :param notebook: GTK Notebook container, whose tab labels are to be rotated and made detachable
        """
        icons = {'Libraries': constants.SIGN_LIB, 'States Tree': constants.ICON_TREE,
                 'Global Variables': constants.ICON_GLOB, 'Modification History': constants.ICON_HIST,
                 'Execution History': constants.ICON_EHIST, 'network': constants.ICON_NET}
        for notebook in self.left_bar_notebooks:
            for i in range(notebook.get_n_pages()):
                child = notebook.get_nth_page(i)
                tab_label = notebook.get_tab_label(child)
                tab_label_text = tab_label.get_text()
                notebook.set_tab_label(child, gui_helper_label.create_tab_header_label(tab_label_text, icons))
                notebook.set_tab_reorderable(child, True)
                notebook.set_tab_detachable(child, True)

    def bring_tab_to_the_top(self, tab_label):
        """Find tab with label tab_label in list of notebooks and set it to the current page.

        :param tab_label: String containing the label of the tab to be focused
        """
        found = False
        for notebook in self.left_bar_notebooks:
            for i in range(notebook.get_n_pages()):
                if gui_helper_label.get_notebook_tab_title(notebook, i) == gui_helper_label.get_widget_title(tab_label):
                    found = True
                    break
            if found:
                notebook.set_current_page(i)
                break
