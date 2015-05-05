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
from awesome_tool.mvc.controllers.network_connections import NetworkConnections
from awesome_tool.utils import constants
from awesome_tool.statemachine.execution.statemachine_status import ExecutionMode
from awesome_tool.mvc.controllers.execution_history import ExecutionHistoryTreeController
import threading


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
        #view['library_vbox'].remove(view['library_tree_placeholder'])

        #view['library_vbox'].pack_start(view.library_tree, True, True, 0)

        library_tree_tab = view['library_vbox']
        page_num = view["tree_notebook_1"].page_num(library_tree_tab)
        view["tree_notebook_1"].remove_page(page_num)

        library_tab_label = gtk.Label('Libraries')
        library_notebook_widget = self.create_notebook_widget('LIBRARIES', view.library_tree)
        view["tree_notebook_1"].insert_page(library_notebook_widget, library_tab_label, page_num)

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

        state_machine_tab_label = gtk.Label('State Tree')
        state_machine_notebook_widget = self.create_notebook_widget('STATE TREE', view.state_machine_tree)
        view["tree_notebook_1"].insert_page(state_machine_notebook_widget, state_machine_tab_label, page_num)

        ######################################################
        # network controller
        ######################################################
        network_connections_ctrl = NetworkConnections(state_machine_manager_model,
                                                      view.network_connections_view)
        self.add_controller('network_connections_ctrl', network_connections_ctrl)

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
                                                                   states_editor_ctrl,
                                                                   network_connections_ctrl)
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

        global_variables_tab_label = gtk.Label('Global Variables')
        global_variables_notebook_widget = self.create_notebook_widget('GLOBAL VARIABLES',
                                                                       view.global_var_manager_view.get_top_widget())
        view["tree_notebook_1"].insert_page(global_variables_notebook_widget, global_variables_tab_label, page_num)

        view["tree_notebook_1"].set_current_page(0)

        ######################################################
        # history
        ######################################################
        #remove placeholder tab
        history_tab = view['history_vbox_placeholder']
        page_num = view["tree_notebook_2"].page_num(history_tab)
        view["tree_notebook_2"].remove_page(page_num)
        #append new tab

        history_tab_label = gtk.Label('History')
        history_notebook_widget = self.create_notebook_widget('HISTORY', gtk.Label("Placeholder"))
        view["tree_notebook_2"].insert_page(history_notebook_widget, history_tab_label, page_num)

        ######################################################
        # execution history
        ######################################################
        #remove placeholder tab
        execution_history_tab = view['execution_history_placeholder']
        page_num = view["tree_notebook_2"].page_num(execution_history_tab)
        view["tree_notebook_2"].remove_page(page_num)
        #append new tab

        execution_history_ctrl = ExecutionHistoryTreeController(state_machine_manager_model,
                                                                view.execution_history_view,
                                                                state_machine_manager)
        self.add_controller('execution_history_ctrl', execution_history_ctrl)

        execution_history_tab_label = gtk.Label('Execution History')
        execution_history_notebook_widget = self.create_notebook_widget('EXECUTION HISTORY',
                                                                        view.execution_history_view.get_top_widget())
        view["tree_notebook_2"].insert_page(execution_history_notebook_widget, execution_history_tab_label, page_num)

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
        network_connections_ctrl.register_menu_bar_controller(menu_bar_controller)

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

        view['hide_right_bar_label'].set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                                           constants.FONT_SIZE_BIG,
                                                                                           constants.BUTTON_RIGHTA))
        view['hide_right_bar_eventbox'].connect('button_release_event', self.right_bar_hide_clicked)

        view['console_hide_label'].set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                                         constants.FONT_SIZE_BIG,
                                                                                         constants.BUTTON_DOWNA))
        view['console_hide_eventbox'].connect('button_release_event', self.console_hide_clicked)

        view['left_bar_return_button'].set_image(self.create_arrow_label(constants.BUTTON_RIGHTA))
        view['left_bar_return_button'].set_border_width(constants.BORDER_WIDTH)

        view['right_bar_return_button'].set_image(self.create_arrow_label(constants.BUTTON_LEFTA))
        view['right_bar_return_button'].set_border_width(constants.BORDER_WIDTH)

        view['console_return_button'].set_image(self.create_arrow_label(constants.BUTTON_UPA))
        view['console_return_button'].set_border_width(constants.BORDER_WIDTH)
        view['console_return_button'].set_size_request(0, constants.BUTTON_MIN_HEIGHT - 10)

        self.left_bar_child = view['left_h_pane'].get_child1()
        self.right_bar_child = view['top_level_h_pane'].get_child2()
        self.console_child = view['left_v_pane_2'].get_child2()

        view['debug_console_button_hbox'].reorder_child(view['button_show_error'], 0)
        view['debug_console_button_hbox'].reorder_child(view['button_show_warning'], 1)
        view['debug_console_button_hbox'].reorder_child(view['button_show_info'], 2)
        view['debug_console_button_hbox'].reorder_child(view['button_show_debug'], 3)

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
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
        elif awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.STOPPED:
            self.on_button_stop_shortcut_clicked(None)
            self.delay(100, self.get_controller('execution_history_ctrl').update)
        elif awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode is ExecutionMode.STEPPING:
            self.set_button_active(True, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)
            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.delay(100, self.get_controller('execution_history_ctrl').update)

    def create_arrow_label(self, icon):
        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                    constants.FONT_SIZE_BIG,
                                                                    icon))
        return label

    def create_label_box(self, text):
        hbox = gtk.HBox()
        label = gtk.Label(text)
        label.set_alignment(0.0, 0.5)
        inner_eventbox = gtk.EventBox()
        inner_label = gtk.Label()
        inner_label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                          constants.FONT_SIZE_BIG,
                                                                          constants.BUTTON_LEFTA))
        inner_eventbox.connect("button_release_event", self.left_bar_hide_clicked)
        inner_eventbox.add(inner_label)
        hbox.pack_start(label, True, True, 0)
        hbox.pack_start(inner_eventbox, False, True, 0)
        hbox.set_border_width(constants.BORDER_WIDTH_TEXTVIEW)
        return hbox

    def on_left_bar_return_button_clicked(self, widget, event=None):
        self.view['left_bar_return_button'].hide()
        self.view['left_h_pane'].add1(self.left_bar_child)

    def on_right_bar_return_button_clicked(self, widget, event=None):
        self.view['right_bar_return_button'].hide()
        self.view['top_level_h_pane'].add2(self.right_bar_child)

    def on_console_return_button_clicked(self, widget, event=None):
        self.view['console_return_button'].hide()
        self.view['left_v_pane_2'].add2(self.console_child)

    def left_bar_hide_clicked(self, widget, event=None):
        self.view['left_h_pane'].remove(self.left_bar_child)
        self.view['left_bar_return_button'].show()

    def right_bar_hide_clicked(self, widget, event=None):
        self.view['top_level_h_pane'].remove(self.right_bar_child)
        self.view['right_bar_return_button'].show()

    def console_hide_clicked(self, widget, event=None):
        self.view['left_v_pane_2'].remove(self.console_child)
        self.view['console_return_button'].show()

    def create_notebook_widget(self, title, widget):
        title_label = self.create_label_box(title)
        event_box = gtk.EventBox()
        vbox = gtk.VBox()
        vbox.pack_start(title_label, False, True, 0)
        vbox.pack_start(widget, True, True, 0)
        event_box.add(vbox)
        event_box.show_all()
        return event_box

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
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_backward_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_backward_step_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

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

    def delay(self, milliseconds, func):
        thread = threading.Timer(milliseconds / 1000.0, func)
        thread.start()