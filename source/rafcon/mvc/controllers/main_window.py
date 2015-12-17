import gtk
import threading

from rafcon.mvc.controllers import GlobalVariableManagerController, StateMachineTreeController, \
    StateMachineHistoryController, LibraryTreeController

from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.library_manager import LibraryManagerModel
from rafcon.mvc.shortcut_manager import ShortcutManager

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.states_editor import StatesEditorController
from rafcon.mvc.controllers.state_machines_editor import StateMachinesEditorController
from rafcon.mvc.controllers.menu_bar import MenuBarController
from rafcon.mvc.controllers.tool_bar import ToolBarController
from rafcon.mvc.controllers.top_tool_bar import TopToolBarController
from rafcon.mvc.controllers.execution_history import ExecutionHistoryTreeController

from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.mvc import gui_helper

from rafcon.mvc.singleton import global_variable_manager_model as gvm_model
import rafcon.statemachine.singleton
import rafcon.statemachine.config
from rafcon.mvc.config import global_gui_config as gui_config
from rafcon.network.network_config import global_net_config

from rafcon.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)
try:
    # run if not defined or variable True
    if global_net_config.get_config_value("NETWORK_CONNECTIONS") is None or global_net_config.get_config_value(
            "NETWORK_CONNECTIONS"):
        from rafcon.mvc.controllers.network_connections import NetworkController
        from rafcon.network.singleton import network_connections
except ImportError as e:
    logger.warn(
        "{1} Only local use of RAFCON will be possible due to missing network communication libraries -> {0}".format(
            e.message, global_net_config.get_config_value("NETWORK_CONNECTIONS") is None))
    # logger.error("%s, %s" % (e.message, traceback.format_exc()))
    global_net_config.set_config_value('NETWORK_CONNECTIONS', False)


class MainWindowController(ExtendedController):
    icons = {
        "Libraries": constants.SIGN_LIB,
        "State Tree": constants.ICON_TREE,
        "Global Variables": constants.ICON_GLOB,
        "History": constants.ICON_HIST,
        "Execution History": constants.ICON_EHIST,
        "Network": constants.ICON_NET
    }

    def __init__(self, state_machine_manager_model, view, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        rafcon.mvc.singleton.main_window_controller = self
        self.editor_type = editor_type
        self.shortcut_manager = None

        # state machine manager
        assert isinstance(state_machine_manager_model, StateMachineManagerModel)
        state_machine_manager = state_machine_manager_model.state_machine_manager

        # execution engine
        self.state_machine_execution_engine = rafcon.statemachine.singleton.state_machine_execution_engine
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
        library_manager_model = LibraryManagerModel(rafcon.statemachine.singleton.library_manager)
        library_controller = LibraryTreeController(library_manager_model, view.library_tree,
                                                   state_machine_manager_model)
        self.add_controller('library_controller', library_controller)
        # view['library_vbox'].remove(view['library_tree_placeholder'])

        # view['library_vbox'].pack_start(view.library_tree, True, True, 0)

        library_tree_tab = view['library_vbox']
        page_num = view["tree_notebook_1"].page_num(library_tree_tab)
        view["tree_notebook_1"].remove_page(page_num)

        library_tab_label = gtk.Label('Libraries')
        library_notebook_widget = self.create_notebook_widget('LIBRARIES', view.library_tree,
                                                              border=constants.BORDER_WIDTH_TEXTVIEW)
        view["tree_notebook_1"].insert_page(library_notebook_widget, library_tab_label, page_num)

        view['add_link_menu_entry'].connect("activate", library_controller.insert_button_clicked, None, False)
        view['add_template_menu_entry'].connect("activate", library_controller.insert_button_clicked, None, True)

        view['main_window'].add_events(
            gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK | gtk.gdk.BUTTON_MOTION_MASK |
            gtk.gdk.KEY_PRESS_MASK | gtk.gdk.KEY_RELEASE_MASK | gtk.gdk.POINTER_MOTION_MASK)

        ######################################################
        # statemachine tree
        ######################################################
        # remove placeholder tab

        state_machine_tree_tab = view['state_machine_tree_placeholder']
        page_num = view["tree_notebook_1"].page_num(state_machine_tree_tab)
        view["tree_notebook_1"].remove_page(page_num)
        # append new tab
        # TODO: this is not always the active state machine
        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)

        state_machine_tab_label = gtk.Label('State Tree')
        state_machine_notebook_widget = self.create_notebook_widget('STATE TREE', view.state_machine_tree,
                                                                    border=constants.BORDER_WIDTH_TEXTVIEW)
        view["tree_notebook_1"].insert_page(state_machine_notebook_widget, state_machine_tab_label, page_num)

        ######################################################
        # state editor
        ######################################################
        states_editor_ctrl = StatesEditorController(state_machine_manager_model,  # or self.model,
                                                    view.states_editor,
                                                    editor_type)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)

        state_editor_label = view['state_editor_label']
        view['state_editor_label_hbox'].remove(state_editor_label)
        new_label = gui_helper.create_label_with_text_and_spacing(state_editor_label.get_text(),
                                                                  font_size=constants.FONT_SIZE_BIG,
                                                                  letter_spacing=constants.LETTER_SPACING_1PT)
        new_label.set_alignment(0., .5)
        view['state_editor_label_hbox'].pack_start(new_label, True, True, 0)
        view['state_editor_label_hbox'].reorder_child(new_label, 0)

        ######################################################
        # state machines editor
        ######################################################
        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                   view.state_machines_editor)
        self.add_controller('state_machines_editor_ctrl', state_machines_editor_ctrl)

        graphical_editor_label = view['graphical_editor_label']
        view['graphical_editor_label_event_box'].remove(graphical_editor_label)
        new_label = gui_helper.create_label_with_text_and_spacing(graphical_editor_label.get_text(),
                                                                  font_size=constants.FONT_SIZE_BIG,
                                                                  letter_spacing=constants.LETTER_SPACING_1PT)
        new_label.set_alignment(0., .5)
        view['graphical_editor_label_event_box'].add(new_label)

        ######################################################
        # global variable editor
        ######################################################
        # remove placeholder tab
        global_variables_tab = view['global_variables_placeholder']
        page_num = view["tree_notebook_1"].page_num(global_variables_tab)
        view["tree_notebook_1"].remove_page(page_num)
        # append new tab
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)

        global_variables_tab_label = gtk.Label('Global Variables')
        global_variables_notebook_widget = self.create_notebook_widget('GLOBAL VARIABLES',
                                                                       view.global_var_manager_view.get_top_widget(),
                                                                       border=constants.BORDER_WIDTH_TEXTVIEW)
        view["tree_notebook_1"].insert_page(global_variables_notebook_widget, global_variables_tab_label, page_num)

        view["tree_notebook_1"].set_current_page(0)

        ######################################################
        # state machine edition history
        ######################################################
        # remove placeholder tab
        history_tab = view['history_vbox_placeholder']
        page_num = view["tree_notebook_2"].page_num(history_tab)
        view["tree_notebook_2"].remove_page(page_num)
        # append new tab
        state_machine_history_controller = StateMachineHistoryController(state_machine_manager_model,
                                                                         view.state_machine_history)
        self.add_controller('state_machine_history_controller', state_machine_history_controller)
        history_label = gtk.Label('History')
        history_notebook_widget = self.create_notebook_widget("HISTORY", view.state_machine_history.get_top_widget(),
                                                              border=constants.BORDER_WIDTH_TEXTVIEW)
        view["tree_notebook_2"].insert_page(history_notebook_widget, history_label, page_num)

        # history_tab_label = gtk.Label('History')
        # history_notebook_widget = self.create_notebook_widget('HISTORY', gtk.Label("Placeholder"))
        # view["tree_notebook_2"].insert_page(history_notebook_widget, history_tab_label, page_num)

        ######################################################
        # network controller
        ######################################################
        if global_net_config.get_config_value('NETWORK_CONNECTIONS', False):
            from rafcon.mvc.controllers.network_connections import NetworkController
            from rafcon.network.singleton import network_connections
            network_connections_ctrl = NetworkController(state_machine_manager_model,
                                                         view.network_connections_view)
            network_connections.initialize()
            self.add_controller('network_connections_ctrl', network_connections_ctrl)

            # remove placehold in tab
            network_tab = view['network_placeholder']
            page_num = view['tree_notebook_2'].page_num(network_tab)
            view['tree_notebook_2'].remove_page(page_num)

            network_label = gtk.Label('Network')
            network_notebook_widget = self.create_notebook_widget("NETWORK",
                                                                  view.network_connections_view.get_top_widget(),
                                                                  use_scroller=False,
                                                                  border=constants.BORDER_WIDTH_TEXTVIEW)
            view['tree_notebook_2'].insert_page(network_notebook_widget, network_label, page_num)
        else:
            network_tab = view['network_tab']
            page_num = view["tree_notebook_2"].page_num(network_tab)
            view["tree_notebook_2"].remove_page(page_num)

        ######################################################
        # state machine execution history
        ######################################################
        # remove placeholder tab
        execution_history_tab = view['execution_history_placeholder']
        page_num = view["tree_notebook_2"].page_num(execution_history_tab)
        view["tree_notebook_2"].remove_page(page_num)
        # append new tab

        execution_history_ctrl = ExecutionHistoryTreeController(state_machine_manager_model,
                                                                view.execution_history_view,
                                                                state_machine_manager)
        self.add_controller('execution_history_ctrl', execution_history_ctrl)

        execution_history_tab_label = gtk.Label('Execution History')
        execution_history_notebook_widget = self.create_notebook_widget('EXECUTION HISTORY',
                                                                        view.execution_history_view.get_top_widget(),
                                                                        use_scroller=False,
                                                                        border=constants.BORDER_WIDTH_TEXTVIEW)
        view["tree_notebook_2"].insert_page(execution_history_notebook_widget, execution_history_tab_label, page_num)

        ######################################################
        # rotate all tab labels by 90 degrees and make detachable
        ######################################################

        for i in range(view["tree_notebook_1"].get_n_pages()):
            child = view["tree_notebook_1"].get_nth_page(i)
            tab_label = view["tree_notebook_1"].get_tab_label(child)
            if gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
                tab_label_text = tab_label.get_text()
                view["tree_notebook_1"].set_tab_label(child, gui_helper.create_tab_header_label(tab_label_text,
                                                                                                self.icons))
            else:
                tab_label.set_angle(90)
            view["tree_notebook_1"].set_tab_reorderable(child, True)
            view["tree_notebook_1"].set_tab_detachable(child, True)

        for i in range(view["tree_notebook_2"].get_n_pages()):
            child = view["tree_notebook_2"].get_nth_page(i)
            tab_label = view["tree_notebook_2"].get_tab_label(child)
            if gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
                tab_label_text = tab_label.get_text()
                view["tree_notebook_2"].set_tab_label(child, gui_helper.create_tab_header_label(tab_label_text,
                                                                                                self.icons))
            else:
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
        view['main_window'].connect('destroy', self.get_controller("menu_bar_controller").on_destroy)

        # Connect left and right bar return buttons' signals to their corresponding methods
        view['left_bar_return_button'].connect('clicked', self.on_left_bar_return_button_clicked)
        view['right_bar_return_button'].connect('clicked', self.on_right_bar_return_button_clicked)

        # Connect Shortcut buttons' signals to their corresponding methods
        view['button_start_shortcut'].connect('toggled', self.on_button_start_shortcut_toggled)
        view['button_stop_shortcut'].connect('clicked', self.on_button_stop_shortcut_clicked)
        view['button_pause_shortcut'].connect('toggled', self.on_button_pause_shortcut_toggled)
        view['button_step_mode_shortcut'].connect('toggled', self.on_button_step_mode_shortcut_toggled)
        view['button_step_in_shortcut'].connect('clicked', self.on_button_step_in_shortcut_clicked)
        view['button_step_over_shortcut'].connect('clicked', self.on_button_step_over_shortcut_clicked)
        view['button_step_out_shortcut'].connect('clicked', self.on_button_step_out_shortcut_clicked)
        view['button_step_backward_shortcut'].connect('clicked', self.on_button_step_backward_shortcut_clicked)

        # Connect Debug console buttons' signals to their corresponding methods
        view['button_show_debug'].connect('toggled', self.on_debug_content_change)
        view['button_show_info'].connect('toggled', self.on_debug_content_change)
        view['button_show_warning'].connect('toggled', self.on_debug_content_change)
        view['button_show_error'].connect('toggled', self.on_debug_content_change)
        view['console_return_button'].connect('clicked', self.on_console_return_button_clicked)

        # hide not usable buttons
        self.view['step_buttons'].hide()

    def highlight_execution_of_current_sm(self, active):
        notebook = self.get_controller('state_machines_editor_ctrl').view['notebook']
        page_num = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_current_page()
        page = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_nth_page(page_num)
        label = notebook.get_tab_label(page).get_children()[0]
        # print rc_style.fg[gtk.STATE_NORMAL]
        if active:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
        else:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        label_string = str(rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        label_string = label_string.replace("STATE_MACHINE_EXECUTION_STATUS.", "")
        self.view['execution_status_label'].set_text(label_string)

        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STARTED:
            self.set_button_active(True, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.PAUSED:
            self.set_button_active(True, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.set_button_active(False, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STOPPED:
            self.on_button_stop_shortcut_clicked(None)
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.highlight_execution_of_current_sm(False)
            self.view['step_buttons'].hide()
        else:  # all step modes
            self.set_button_active(True, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)
            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].show()

    @staticmethod
    def create_arrow_label(icon):
        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                    constants.FONT_SIZE_BIG,
                                                                    icon))
        return label

    def create_label_box(self, text):
        hbox = gtk.HBox()
        label = gui_helper.create_label_with_text_and_spacing(text, font_size=constants.FONT_SIZE_BIG,
                                                              letter_spacing=constants.LETTER_SPACING_1PT)
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

    def create_notebook_widget(self, title, widget, use_scroller=True, border=10):
        title_label = self.create_label_box(title)
        event_box = gtk.EventBox()
        vbox = gtk.VBox()
        vbox.pack_start(title_label, False, True, 0)
        if use_scroller:
            scroller = gtk.ScrolledWindow()
            scroller.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
            alig = gtk.Alignment(0., 0., 1., 1.)
            alig.set_padding(0, 0, border, 0)
            alig.add(widget)
            alig.show()
            scroller.add_with_viewport(alig)
            vbox.pack_start(scroller, True, True, 0)
        else:
            vbox.pack_start(widget, True, True, 0)
        event_box.add(vbox)
        event_box.show_all()
        return event_box

    # Shortcut buttons

    def on_button_start_shortcut_toggled(self, widget, event=None):
        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
            self.get_controller("menu_bar_controller").on_start_activate(None)

            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)
        else:
            logger.info("Statemachine running")
            self.set_button_active(True, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)

    def on_button_pause_shortcut_toggled(self, widget, event=None):
        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.PAUSED:
            self.get_controller("menu_bar_controller").on_pause_activate(None)

            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
            self.set_button_active(False, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)
        else:
            logger.info("Statemachine paused")
            self.set_button_active(True, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)

    def on_button_stop_shortcut_clicked(self, widget, event=None):
        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            self.get_controller("menu_bar_controller").on_stop_activate(None)

        self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
        self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
        self.set_button_active(False, self.view['button_step_mode_shortcut'], self.on_button_step_mode_shortcut_toggled)

    def on_button_step_mode_shortcut_toggled(self, widget, event=None):
        em = rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode
        if em is StateMachineExecutionStatus.STARTED or\
            em is StateMachineExecutionStatus.STOPPED or\
            em is StateMachineExecutionStatus.PAUSED:
            self.get_controller("menu_bar_controller").on_step_mode_activate(None)

            self.set_button_active(False, self.view['button_pause_shortcut'], self.on_button_pause_shortcut_toggled)
            self.set_button_active(False, self.view['button_start_shortcut'], self.on_button_start_shortcut_toggled)
        else:
            logger.info("Statemachine stepping")
            self.set_button_active(True, self.view['button_step_mode_shortcut'],
                                   self.on_button_step_mode_shortcut_toggled)

    def on_button_step_in_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_step_into_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_over_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_step_over_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_out_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_step_out_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_out_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_step_out_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_backward_shortcut_clicked(self, widget, event=None):
        self.get_controller("menu_bar_controller").on_backward_step_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def set_button_active(self, active, button, func):
        button.handler_block_by_func(func)
        button.set_active(active)
        button.handler_unblock_by_func(func)

    def on_debug_content_change(self, widget, data=None):
        if self.view['button_show_info'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_INFO', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_INFO', False)
        if self.view['button_show_debug'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_DEBUG', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_DEBUG', False)
        if self.view['button_show_warning'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_WARNING', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_WARNING', False)
        if self.view['button_show_error'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_ERROR', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_ERROR', False)
        # gui_config.save_configuration()
        self.view.logging_view.update_filtered_buffer()

    @staticmethod
    def delay(milliseconds, func):
        thread = threading.Timer(milliseconds / 1000.0, func)
        thread.start()
