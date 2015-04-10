import traceback

import gtk

from awesome_tool.mvc.controllers import GlobalVariableManagerController, StateMachineTreeController, LibraryTreeController
import awesome_tool.statemachine.singleton
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers.states_editor import StatesEditorController
from awesome_tool.mvc.controllers.state_machines_editor import StateMachinesEditorController
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.selection import Selection
from awesome_tool.mvc.models.library_manager import LibraryManagerModel
from awesome_tool.mvc.shortcut_manager import ShortcutManager
from awesome_tool.mvc.views.state_machines_editor import StateMachinesEditorView
from awesome_tool.mvc.views.states_editor import StatesEditorView
from awesome_tool.utils import log
logger = log.get_logger(__name__)
import awesome_tool.statemachine.config
from awesome_tool.mvc.controllers.menu_bar_controller import MenuBarController
from awesome_tool.mvc.controllers.tool_bar_controller import ToolBarController
from awesome_tool.mvc.controllers.top_tool_bar_controller import TopToolBarController
from awesome_tool.utils import constants
from awesome_tool.statemachine.execution.statemachine_status import ExecutionMode

class MainWindowController(ExtendedController):

    def __init__(self, state_machine_manager_model, view, gvm_model, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        self.editor_type = editor_type
        self.shortcut_manager = None

        # state machine manager
        assert isinstance(state_machine_manager_model, StateMachineManagerModel)
        state_machine_manager = state_machine_manager_model.state_machine_manager
        active_state_machine_id = state_machine_manager.active_state_machine_id
        active_state_machine = None
        if len(state_machine_manager_model.state_machines) > 0:
            active_state_machine = state_machine_manager_model.state_machines[active_state_machine_id]

        if active_state_machine is None:
            logger.warn("No active state machine found")

        # execution engine
        self.state_machine_execution_engine = awesome_tool.statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        ######################################################
        # shortcut manager
        ######################################################
        self.shortcut_manager = ShortcutManager(view['main_window'])

        ######################################################
        # logging view
        ######################################################
        self.console_scroller = view['console_scroller']
        view['debug_console_vbox'].remove(self.console_scroller)
        view.logging_view.get_top_widget().show()
        view['debug_console_vbox'].pack_start(view.logging_view.get_top_widget(), True, True, 0)

        ######################################################
        # library tree
        ######################################################
        library_manager_model = LibraryManagerModel(awesome_tool.statemachine.singleton.library_manager)
        library_controller = LibraryTreeController(library_manager_model, view.library_tree, state_machine_manager_model)
        self.add_controller('library_controller', library_controller)
        view['library_vbox'].remove(view['library_tree_placeholder'])
        view['library_vbox'].pack_start(view.library_tree, True, True, 0)
        # view['add_link_button'].connect("clicked", library_controller.add_link_button_clicked,
        #                                 state_machine_manager_model)
        # view['add_template_button'].connect("clicked", library_controller.add_template_button_clicked,
        #                                     state_machine_manager_model)
        view['add_link_menu_entry'].connect("activate", library_controller.add_link_button_clicked,
                                        state_machine_manager_model)
        view['add_template_menu_entry'].connect("activate", library_controller.add_template_button_clicked,
                                            state_machine_manager_model)

        view['main_window'].add_events(gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK | gtk.gdk.BUTTON_MOTION_MASK |
                               gtk.gdk.KEY_PRESS_MASK | gtk.gdk.KEY_RELEASE_MASK | gtk.gdk.POINTER_MOTION_MASK)

        ######################################################
        # statemachine tree
        ######################################################
        # remove placeholder tab

        state_machine_tree_tab = view['state_machine_tree_placeholder']
        page_num = view["tree_notebook_1"].page_num(state_machine_tree_tab)
        view["tree_notebook_1"].remove_page(page_num)
        #append new tab
        #TODO: this is not always the active state machine
        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)
        state_machine_label = gtk.Label('STATE TREE')
        state_machine_event_box = gtk.EventBox()
        state_machine_event_box.set_border_width(constants.BORDER_WIDTH)
        state_machine_alignment = gtk.Alignment(0.0, 0.5, 0.0, 0.0)
        state_machine_alignment.add(state_machine_label)
        state_machine_event_box.add(state_machine_alignment)
        state_machine_tab_label = gtk.Label('State Tree')
        state_machine_vbox = gtk.VBox()
        state_machine_vbox.pack_start(state_machine_event_box, False, True, 0)
        state_machine_vbox.pack_start(view.state_machine_tree, True, True, 0)
        state_machine_vbox.show_all()
        view["tree_notebook_1"].insert_page(state_machine_vbox, state_machine_tab_label, page_num)

        ######################################################
        # state editor
        ######################################################
        states_editor_ctrl = StatesEditorController(state_machine_manager_model,  # or self.model,
                                                         view.states_editor,
                                                         editor_type)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)
        ######################################################
        # state machines editor
        ######################################################
        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                        view.state_machines_editor,
                                                                        states_editor_ctrl)
        self.add_controller('state_machines_editor_ctrl', state_machines_editor_ctrl)

        ######################################################
        # global variable editor
        ######################################################
        #remove placeholder tab
        global_variables_tab = view['global_variables_placeholder']
        page_num = view["tree_notebook_1"].page_num(global_variables_tab)
        view["tree_notebook_1"].remove_page(page_num)
        #append new tab
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)
        global_variables_label = gtk.Label('GLOBAL VARIABLES')
        global_variables_event_box = gtk.EventBox()
        global_variables_alignment = gtk.Alignment(0.0, 0.5, 0.0, 0.0)
        global_variables_alignment.add(global_variables_label)
        global_variables_event_box.add(global_variables_alignment)
        global_variables_event_box.set_border_width(constants.BORDER_WIDTH)
        global_variables_tab_label = gtk.Label('Global Variables')
        global_variables_vbox = gtk.VBox()
        global_variables_vbox.pack_start(global_variables_event_box, False, True, 0)
        global_variables_vbox.pack_start(view.global_var_manager_view.get_top_widget(), True, True, 0)
        global_variables_vbox.show_all()
        view["tree_notebook_1"].insert_page(global_variables_vbox, global_variables_tab_label, page_num)

        ######################################################
        # rotate all tab labels by 90 degrees and make detachable
        ######################################################

        for i in range(view["tree_notebook_1"].get_n_pages()):
            child = view["tree_notebook_1"].get_nth_page(i)
            tab_label = view["tree_notebook_1"].get_tab_label(child)
            tab_label.set_angle(90)
            view["tree_notebook_1"].set_tab_reorderable(child, True)
            view["tree_notebook_1"].set_tab_detachable(child, True)

        for i in range(view["tree_notebook_2"].get_n_pages()):
            child = view["tree_notebook_2"].get_nth_page(i)
            tab_label = view["tree_notebook_2"].get_tab_label(child)
            tab_label.set_angle(90)
            view["tree_notebook_2"].set_tab_reorderable(child, True)
            view["tree_notebook_2"].set_tab_detachable(child, True)

        ######################################################
        # menu bar
        ######################################################
        menu_bar_controller = MenuBarController(state_machine_manager_model,
                                                view,
                                                state_machines_editor_ctrl,
                                                states_editor_ctrl,
                                                view.logging_view,
                                                view.get_top_widget(),
                                                self.shortcut_manager)
        self.add_controller("menu_bar_controller", menu_bar_controller)

        ######################################################
        # tool bar
        ######################################################
        tool_bar_controller = ToolBarController(state_machine_manager_model,
                                                     view.tool_bar,
                                                     menu_bar_controller)
        self.add_controller("tool_bar_controller", tool_bar_controller)

        ######################################################
        # top tool bar
        ######################################################
        top_tool_bar_controller = TopToolBarController(state_machine_manager_model,
                                                     view.top_tool_bar,
                                                     view["main_window"],
                                                     menu_bar_controller)
        self.add_controller("top_tool_bar_controller", top_tool_bar_controller)

        ######################################################
        # setup correct sizes
        ######################################################
        view['top_level_h_pane'].set_position(1200)
        view['left_h_pane'].set_position(300)
        view['left_v_pane_1'].set_position(400)
        view['left_v_pane_2'].set_position(600)

    def register_view(self, view):
        self.register_actions(self.shortcut_manager)
        view['main_window'].connect('delete_event', self.get_controller("menu_bar_controller").on_delete_event)
        view['main_window'].connect('destroy', self.get_controller("menu_bar_controller").destroy)

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        label_string = str(awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        label_string = label_string.replace("EXECUTION_MODE.", "")
        self.view['execution_status_label'].set_text(label_string)

        if awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.RUNNING:
            self.set_button_active(True, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
        elif awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.PAUSED:
            self.set_button_active(True, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
        elif awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.STOPPED:
            self.on_button_stop_shortcut_clicked(None)
        elif awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.STEPPING:
            self.set_button_active(True, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)

    # Shortcut buttons

    def on_button_start_shortcut_toggled(self, widget, event=None):
        if awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not ExecutionMode.RUNNING:
            self.get_controller("menu_bar_controller").on_start_activate(None)

            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
        else:
            logger.info("Statemachine running")
            self.set_button_active(True, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)

    def on_button_pause_shortcut_toggled(self, widget, event=None):
        if awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not ExecutionMode.PAUSED:
            self.get_controller("menu_bar_controller").on_pause_activate(None)

            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
        else:
            logger.info("Statemachine paused")
            self.set_button_active(True, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)

    def on_button_stop_shortcut_clicked(self, widget, event=None):
        if awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not ExecutionMode.STOPPED:
            self.get_controller("menu_bar_controller").on_stop_activate(None)

        self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
        self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
        self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)

    def on_button_step_mode_shortcut_toggled(self, widget, event=None):
        if awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not ExecutionMode.STEPPING:
            self.get_controller("menu_bar_controller").on_step_mode_activate(None)

            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
        else:
            logger.info("Statemachine stepping")
            self.set_button_active(True, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)

    def on_button_step_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_step_activate(None)

    def on_button_step_backward_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_backward_step_mode_activate(None)

    def set_button_active(self, active, button, func):
        button.handler_block_by_func(func)
        button.set_active(active)
        button.handler_unblock_by_func(func)

    def on_debug_content_change(self, widget, data=None):
        info = self.view['button_show_info'].get_active()
        debug = self.view['button_show_debug'].get_active()
        warning = self.view['button_show_warning'].get_active()
        error = self.view['button_show_error'].get_active()
        self.view.logging_view.update_filtered_buffer(info, debug, warning, error)