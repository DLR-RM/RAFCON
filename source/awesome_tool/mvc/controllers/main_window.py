import traceback

import gtk

from awesome_tool.mvc.controllers import GlobalVariableManagerController, StateMachineTreeController, LibraryTreeController
import awesome_tool.statemachine.singleton
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers.states_editor import StatesEditorController
from awesome_tool.mvc.controllers.state_machines_editor import StateMachinesEditorController
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel, Selection
from awesome_tool.mvc.models.library_manager import LibraryManagerModel
from awesome_tool.mvc.shortcut_manager import ShortcutManager
from awesome_tool.mvc.views.state_machines_editor import StateMachinesEditorView
from awesome_tool.mvc.views.states_editor import StatesEditorView
from awesome_tool.utils import log
logger = log.get_logger(__name__)
import awesome_tool.statemachine.config
from awesome_tool.mvc.controllers.menu_bar_controller import MenuBarController


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

        # connect destroy main window
        view.get_top_widget().connect("destroy", self.on_main_window_destroy)

        ######################################################
        # logging view
        ######################################################
        self.console_scroller = view['left_v_pane'].get_child2()
        view['left_v_pane'].remove(self.console_scroller)
        view.logging_view.get_top_widget().show()
        view['left_v_pane'].add2(view.logging_view.get_top_widget())

        ######################################################
        # library tree
        ######################################################
        library_manager_model = LibraryManagerModel(awesome_tool.statemachine.singleton.library_manager)
        library_controller = LibraryTreeController(library_manager_model, view.library_tree)
        self.add_controller('library_controller', library_controller)
        view['library_vbox'].remove(view['library_tree_placeholder'])
        view['library_vbox'].pack_start(view.library_tree, True, True, 0)
        view['add_library_button'].connect("clicked", library_controller.add_library_button_clicked,
                                           state_machine_manager_model)

        ######################################################
        # statemachine tree
        ######################################################
        #remove placeholder tab

        state_machine_tree_tab = view['state_machine_tree_placeholder']
        page_num = view["tree_notebook"].page_num(state_machine_tree_tab)
        view["tree_notebook"].remove_page(page_num)
        #append new tab
        #TODO: this is not always the active state machine
        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)
        state_machine_label = gtk.Label('Statemachine')
        view["tree_notebook"].insert_page(view.state_machine_tree, state_machine_label, page_num)

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
        page_num = view["tree_notebook"].page_num(global_variables_tab)
        view["tree_notebook"].remove_page(page_num)
        #append new tab
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)
        global_variables_label = gtk.Label('Global Variables')
        view["tree_notebook"].insert_page(view.global_var_manager_view.get_top_widget(), global_variables_label, page_num)

        ######################################################
        # status bar
        ######################################################
        # add some data to the status bar
        status_bar1 = view["statusbar1"]
        status_bar1.push(0, "The awesome tool")
        status_bar2 = view["statusbar2"]
        status_bar2.push(0, "is awesome :-)")
        status_bar3 = view["statusbar3"]
        status_bar3_string = "Execution status: " + \
                             str(awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        status_bar3.push(0, status_bar3_string)

        ######################################################
        # status bar
        ######################################################
        self.menu_bar_controller = MenuBarController(state_machine_manager_model,
                                                     view.menu_bar,
                                                     state_machines_editor_ctrl)
        self.add_controller("menu_bar_controller", self.menu_bar_controller)

        # connect toolbar buttons
        view['button_save'].connect("clicked", self.menu_bar_controller.on_save_activate)
        view['button_new'].connect("clicked", self.menu_bar_controller.on_new_activate)
        view['button_open'].connect("clicked", self.menu_bar_controller.on_open_activate)
        view['button_refresh'].connect("clicked", self.on_refresh_all_activate)
        view['button_refresh_libs'].connect("clicked", self.on_refresh_libraries_activate)

        ######################################################
        # setup correct sizes
        ######################################################
        view['top_h_pane'].set_position(200)
        view['left_v_pane'].set_position(700)

    def register_view(self, view):
        self.shortcut_manager = ShortcutManager(self.view['main_window'])
        self.register_actions(self.shortcut_manager)

        view['main_window'].connect('destroy', gtk.main_quit)

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        status_bar3 = self.view["statusbar3"]
        status_bar3_string = "Execution status: " + \
                             str(awesome_tool.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        status_bar3.push(0, status_bar3_string)

    def on_main_window_destroy(self, widget, data=None):
        self.view.logging_view.quit_flag = True
        logger.debug("Main window destroyed")
        log.debug_filter.set_logging_test_view(None)
        log.error_filter.set_logging_test_view(None)
        awesome_tool.statemachine.config.global_config.save_configuration()
        gtk.main_quit()

    ######################################################
    # toolbar functionality
    ######################################################
    def on_refresh_libraries_activate(self, widget, data=None):
        """
        Deletes and reloads all libraries from the filesystem.
        :param widget: the main widget
        :param data: optional data
        :return:
        """
        awesome_tool.statemachine.singleton.library_manager.refresh_libraries()

    def on_refresh_all_activate(self, widget, data=None):
        """
        Reloads all libraries and thus all state machines as well.
        :param widget: the main widget
        :param data: optional data
        :return:
        """
        if len(awesome_tool.statemachine.singleton.global_storage.ids_of_modified_state_machines) > 0:
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "Are you sure you want to reload the libraries and thus all state_machines. " \
                             "The following state machines were modified and not saved: "
            for sm_id in awesome_tool.statemachine.singleton.global_storage.ids_of_modified_state_machines:
                message_string = "%s %s " % (message_string, str(sm_id))
            message_string = "%s \n(Note: all state machines that are freshly created and have never been saved " \
                             "before will be deleted!)" % message_string
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_refresh_message_dialog_response_signal)
            message.show()
        else:
            self.refresh_libs_and_statemachines()

    def on_refresh_message_dialog_response_signal(self, widget, response_id):
        if response_id == 42:
            self.refresh_libs_and_statemachines()
        else:
            logger.debug("Refresh canceled")
        widget.destroy()

    def refresh_libs_and_statemachines(self):
        """
        Deletes all libraries and state machines and reloads them freshly from the file system.
        :return:
        """
        awesome_tool.statemachine.singleton.library_manager.refresh_libraries()

        # delete dirty flags for state machines
        awesome_tool.statemachine.singleton.global_storage.reset_dirty_flags()

        # create a dictionary from state machine id to state machine path
        state_machine_id_to_path = {}
        sm_keys = []
        for sm_id, sm in awesome_tool.statemachine.singleton.state_machine_manager.state_machines.iteritems():
            # the sm.base_path is only None if the state machine has never been loaded or saved before
            if sm.base_path is not None:
                #print sm.root_state.script.path
                # cut the last directory from the path
                path_items = sm.root_state.script.path.split("/")
                new_path = path_items[0]
                for i in range(len(path_items) - 2):
                    new_path = "%s/%s" % (new_path, path_items[i+1])
                #print new_path
                state_machine_id_to_path[sm_id] = new_path
                sm_keys.append(sm_id)

        self.get_controller('states_editor_ctrl').close_all_tabs()
        self.get_controller('state_machines_editor_ctrl').close_all_tabs()

        # reload state machines from file system
        awesome_tool.statemachine.singleton.state_machine_manager.refresh_state_machines(sm_keys, state_machine_id_to_path)

