import gtk
from gtkmvc import Controller
from utils import log
logger = log.get_logger(__name__)
from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController,\
    StateDataPortEditorController, GlobalVariableManagerController, ExternalModuleManagerController,\
    SourceEditorController, SingleWidgetWindowController, StateEditorController, StateMachineTreeController,\
    LibraryTreeController
import statemachine.singleton
from mvc.controllers.states_editor import StatesEditorController
from mvc.controllers.state_machines_editor import StateMachinesEditorController

import traceback

from mvc.controllers.state_data_flows import StateDataFlowsEditorController, StateDataFlowsListController


def setup_key_binding(key_map, view, widget):
        """
        @brief read config.py and add key accelerators according to the
        name of field key_map, adds AccelGroup to widget and add accelerator to dict of view.
        """
        accelgroup = gtk.AccelGroup()
        widget.add_accel_group(accelgroup)

        for item in key_map.iteritems():
            key, mod = gtk.accelerator_parse(str(item[1]))
            try:
                if not view[item[0]]:
                    view[item[0]] = gtk.MenuItem()
                view[item[0]].add_accelerator("activate", accelgroup, key, mod, gtk.ACCEL_VISIBLE)
            except:
                logger.warn(traceback.format_exc())
                pass


class MainWindowController(Controller):

    # state_machine_execution_engine = None
    #
    # __observables__ = ("state_machine_execution_engine",)

    def __init__(self, state_machine_manager_model, view, emm_model, gvm_model, editor_type='egg'):
        Controller.__init__(self, state_machine_manager_model, view)

        self.state_machine_execution_engine = statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        self.root_state_model = self.model.state_machines.values()[0].root_state
        self.em_module = self.model.state_machines.values()[0].root_state
        self.em_module = self.model.state_machines.values()[0].root_state
        top_h_pane = view['top_h_pane']
        left_v_pane = view['left_v_pane']
        right_v_pane = view['right_v_pane']

        view.get_top_widget().connect("destroy", self.on_main_window_destroy)

        ######################################################
        # logging view
        ######################################################
        left_v_pane = view["left_v_pane"]
        console_scroller = left_v_pane.get_child2()
        left_v_pane.remove(console_scroller)
        view.logging_view.get_top_widget().show()
        left_v_pane.add2(view.logging_view.get_top_widget())
        #console_scroller.add(view.logging_view["textview"])

        ######################################################
        # library tree
        ######################################################
        tree_notebook = view["tree_notebook"]
        #remove placeholder tab
        library_tree_tab = view['library_tree_placeholder']
        page_num = tree_notebook.page_num(library_tree_tab)
        tree_notebook.remove_page(page_num)
        #append new tab
        self.library_controller = LibraryTreeController(None, view.library_tree)
        libraries_label = gtk.Label('Libraries')
        tree_notebook.insert_page(view.library_tree, libraries_label, page_num)

        ######################################################
        # statemachine tree
        ######################################################
        #remove placeholder tab
        state_machine_tree_tab = view['state_machine_tree_placeholder']
        page_num = tree_notebook.page_num(state_machine_tree_tab)
        tree_notebook.remove_page(page_num)
        #append new tab
        self.state_machine_tree_controller = StateMachineTreeController(self.model.root_state,
                                                                        view.state_machine_tree)
        state_machine_label = gtk.Label('Statemachine Tree')
        tree_notebook.insert_page(view.state_machine_tree, state_machine_label, page_num)

        ######################################################
        # state editor
        ######################################################
        # this_model = filter(lambda model: model.state.name == 'State3', self.model.root_state.states.values()).pop()
        self.states_editor_ctrl = StatesEditorController(self.model.state_machines.values()[0].root_state, view.states_editor, editor_type)

        ######################################################
        # graphical editor
        ######################################################

        self.state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                        view.state_machines_editor,
                                                                        self.state_machine_tree_controller,
                                                                        self.states_editor_ctrl)
        #self.state_machines_editor_ctrl.add_graphical_state_machine_editor(state_machine_manager_model.root_state)

        # self.graphical_editor_ctrl = GraphicalEditorController(self.model.root_state, view.graphical_editor_view)
        # #self.graphical_editor = GraphicalEditorController(model, view.graphical_editor_window.get_top_widget())
        # #test = SingleWidgetWindowController(model, view.graphical_editor_window, GraphicalEditorController)

        ######################################################
        # external module editor
        ######################################################
        em_global_notebook = view["right_bottom_notebook"]
        #remove placeholder tab
        external_modules_tab = view['external_modules_placeholder']
        page_num = em_global_notebook.page_num(external_modules_tab)
        em_global_notebook.remove_page(page_num)
        #append new tab
        self.external_modules_controller = ExternalModuleManagerController(emm_model, view.external_module_manager_view)
        external_modules_label = gtk.Label('External Modules')
        em_global_notebook.insert_page(view.external_module_manager_view.get_top_widget(), external_modules_label, page_num)

        ######################################################
        # global variable editor
        ######################################################
        #remove placeholder tab
        global_variables_tab = view['global_variables_placeholder']
        page_num = em_global_notebook.page_num(global_variables_tab)
        em_global_notebook.remove_page(page_num)
        #append new tab
        self.external_modules_controller = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        global_variables_label = gtk.Label('Global Variables')
        em_global_notebook.insert_page(view.global_var_manager_view.get_top_widget(), global_variables_label, page_num)

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
                             str(statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        status_bar3.push(0, status_bar3_string)

        ######################################################
        # setupt correct sizes
        ######################################################
        top_h_pane.set_position(200)
        left_v_pane.set_position(700)
        right_v_pane.set_position(600)
        self.key_map = {
            'copy'          : '<Control>C',
            'paste'         : '<Control>V',
            'cut'           : '<Control>X',
            'delete'        : 'Delete',
            'add'           : '<mod>A',
            'add_state'     : '<Control>A',
            'delete_state'  : '<Control>D',
            'group_states'  : '<Control>G',
            'ungroup_states': '<Control>U',
            'entry_point'   : '<Control>E',
            # 'exit_point'    : '<Control>R',
            'fit'           : '<Control>space',
            'info'          : '<Control>q',
            'start'         : 'F5',
            'step_mode'     : 'F6',  # debug
            'pause'         : 'F7',
            'stop'          : 'F8',
            'step'          : 'F4',
            'backward_step_mode': 'F9',
            # 'reload'        : 'F1',
            # 'reload_env'    : 'F2',
            'undo'          : '<Control>Z',
            'redo'          : '<Control>Y',
            # 'center_view'   : '<Alt>space',
        }

    def on_main_window_destroy(self, widget, data=None):
        logger.debug("Main window destroyed")
        gtk.main_quit()

    def register_view(self, view):
        setup_key_binding(self.key_map, view, view.get_top_widget())

        view['add'].connect('activate', self.delegate_action, StateDataFlowsListController.on_add)
        view['add_state'].connect('activate', self.delegate_action, StateDataFlowsListController.on_add)
        view['main_window'].connect('destroy', gtk.main_quit)

    @Controller.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        status_bar3 = self.view["statusbar3"]
        status_bar3_string = "Execution status: " + \
                             str(statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        status_bar3.push(0, status_bar3_string)

    def on_about_activate(self, widget, data=None):
        pass

    def on_backward_step_mode_activate(self, widget, data=None):
        logger.debug("Backward execution step not implemented yet!")
        pass

    def on_step_activate(self, widget, data=None):
        logger.debug("Execution step ...")
        statemachine.singleton.state_machine_execution_engine.step()

    def on_step_mode_activate(self, widget, data=None):
        logger.debug("Activate execution engine step mode ...")
        statemachine.singleton.state_machine_execution_engine.step_mode()

    def on_stop_activate(self, widget, data=None):
        logger.debug("Stop execution engine ...")
        statemachine.singleton.state_machine_execution_engine.stop()

    def on_pause_activate(self, widget, data=None):
        logger.debug("Pause execution engine ...")
        statemachine.singleton.state_machine_execution_engine.pause()

    def on_start_activate(self, widget, data=None):
        logger.debug("Start execution engine ...")
        statemachine.singleton.state_machine_execution_engine.start()

    def on_grid_toggled(self, widget, data=None):
        pass

    def on_redo_activate(self, widget, data=None):
        pass

    def on_undo_activate(self, widget, data=None):
        pass

    def on_ungroup_states_activate(self, widget, data=None):
        pass

    def on_group_states_activate(self, widget, data=None):
        pass

    def on_delete_state_activate(self, widget, data=None):
        logger.debug("Delete selected state now ...")

    def delegate_action(self, event, method, *arg):
        """
        @brief when a signal is detected, this call the callback
        corresponding to the currently active bubble area.
        """

        idx = self.states_editor_ctrl.view.notebook.get_current_page()
        page = self.states_editor_ctrl.view.notebook.get_nth_page(idx)
        ctrl = None
        for identifier, pdict in self.states_editor_ctrl.tabs.iteritems():
            if page is pdict['page']:
                ctrl = pdict['ctrl']
        method(ctrl.data_flows_ctrl.df_list_ctrl, event, *arg)

    def on_add_state_activate(self, widget, method=None, *arg):
        logger.debug("Add state in selected state now ..." + str(widget) + str(method))

    def on_delete_activate(self, widget, data=None):
        logger.debug("Delete something selected now ...")

    def on_paste_activate(self, widget, data=None):
        pass

    def on_copy_activate(self, widget, data=None):
        pass

    def on_cut_activate(self, widget, data=None):
        pass

    def on_quit_activate(self, widget, data=None):
        pass

    def on_menu_properties_activate(self, widget, data=None):
        pass

    def on_save_as_activate(self, widget, data=None):
        pass

    def on_save_activate(self, widget, data=None):
        pass

    def on_open_activate(self, widget, data=None):
        pass

    def on_new_activate(self, widget, data=None):
        pass
