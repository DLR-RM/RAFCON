import traceback

import gtk

from mvc.controllers import GlobalVariableManagerController, StateMachineTreeController, LibraryTreeController
import statemachine.singleton
from mvc.controllers.extended_controller import ExtendedController
from mvc.controllers.states_editor import StatesEditorController
from mvc.controllers.state_machines_editor import StateMachinesEditorController
from mvc.models.state_machine_manager import StateMachineManagerModel
from mvc.shortcut_manager import ShortcutManager
from utils import log
logger = log.get_logger(__name__)


class MainWindowController(ExtendedController):

    def __init__(self, state_machine_manager_model, view, gvm_model, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        assert isinstance(state_machine_manager_model, StateMachineManagerModel)

        self.shortcut_manager = None
        state_machine_manager = state_machine_manager_model.state_machine_manager
        active_state_machine_id = state_machine_manager.active_state_machine_id
        active_state_machine = None
        for state_machine_id in state_machine_manager_model.state_machines:
            if state_machine_id == active_state_machine_id:
                active_state_machine = state_machine_manager_model.state_machines[state_machine_id]
                break

        if active_state_machine is None:
            raise AttributeError("No active state machine found")

        self.state_machine_execution_engine = statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        top_h_pane = view['top_h_pane']
        left_v_pane = view['left_v_pane']

        view.get_top_widget().connect("destroy", self.on_main_window_destroy)

        tree_notebook = view["tree_notebook"]

        ######################################################
        # logging view
        ######################################################
        self.console_scroller = left_v_pane.get_child2()
        left_v_pane.remove(self.console_scroller)
        view.logging_view.get_top_widget().show()
        left_v_pane.add2(view.logging_view.get_top_widget())
        #console_scroller.add(view.logging_view["textview"])

        ######################################################
        # library tree
        ######################################################
        library_controller = LibraryTreeController(active_state_machine.root_state, view.library_tree)
        self.add_controller('library_controller', library_controller)
        view['library_vbox'].remove(view['library_tree_placeholder'])
        view['library_vbox'].pack_start(view.library_tree, True, True, 0)
        view['add_library_button'].connect("clicked", library_controller.add_library_button_clicked,
                                           state_machine_manager_model)

        view['button_save'].connect("clicked", self.on_save_activate)

        ######################################################
        # statemachine tree
        ######################################################
        #remove placeholder tab
        state_machine_tree_tab = view['state_machine_tree_placeholder']
        page_num = tree_notebook.page_num(state_machine_tree_tab)
        tree_notebook.remove_page(page_num)
        #append new tab
        state_machine_tree_controller = StateMachineTreeController(active_state_machine, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)
        state_machine_label = gtk.Label('Statemachine')
        tree_notebook.insert_page(view.state_machine_tree, state_machine_label, page_num)

        ######################################################
        # state editor
        ######################################################
        # this_model = filter(lambda model: model.state.name == 'State3', self.model.root_state.states.values()).pop()
        # self.states_editor_ctrl = StatesEditorController(self.model.state_machines.values()[0].root_state,
        #                                                  view.states_editor,
        #                                                  editor_type)
        states_editor_ctrl = StatesEditorController(state_machine_manager_model,  # or self.model,
                                                         view.states_editor,
                                                         editor_type)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)
        ######################################################
        # state machines editor
        ######################################################

        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                        view.state_machines_editor,
                                                                        state_machine_tree_controller,
                                                                        states_editor_ctrl)
        self.add_controller('state_machines_editor_ctrl', state_machines_editor_ctrl)
        #self.state_machines_editor_ctrl.add_graphical_state_machine_editor(state_machine_manager_model.root_state)

        # self.graphical_editor_ctrl = GraphicalEditorController(self.model.root_state, view.graphical_editor_view)
        # #self.graphical_editor = GraphicalEditorController(model, view.graphical_editor_window.get_top_widget())
        # #test = SingleWidgetWindowController(model, view.graphical_editor_window, GraphicalEditorController)

        ######################################################
        # global variable editor
        ######################################################
        #remove placeholder tab
        global_variables_tab = view['global_variables_placeholder']
        page_num = tree_notebook.page_num(global_variables_tab)
        tree_notebook.remove_page(page_num)
        #append new tab
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)
        global_variables_label = gtk.Label('Global Variables')
        tree_notebook.insert_page(view.global_var_manager_view.get_top_widget(), global_variables_label, page_num)

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
        self.key_map = {
            'copy'          : '<Control>C',     # TODO should not be that hart for full states
            'paste'         : '<Control>V',     # TODO should not be that hart for full states
            'cut'           : '<Control>X',     # TODO should not be that hart for full states
            'delete'        : 'Delete',         # DONE - partly TODO could delete DataPorts and Outcomes
            # 'add'           : '<mod>A',
            'add_state'     : '<Control>A',     # DONE
            'delete_state'  : '<Control>D',     # DONE
            'group_states'  : '<Control>G',     # TODO search all states selected and create hierarchy-state
            'ungroup_states': '<Control>U',     # TODO search all states, data_flows and trans and move them to parent
            'entry_point'   : '<Control>E',     # TODO ?
            # 'exit_point'    : '<Control>R',
            'fit'           : '<Control>space',
            'info'          : '<Control>q',
            'start'         : 'F5',             # DONE
            'step_mode'     : 'F6',             # DONE debug-mode
            'pause'         : 'F7',             # DONE
            'stop'          : 'F8',             # DONE
            'step'          : 'F4',             # DONE
            'backward_step_mode': 'F9',
            # 'reload'        : 'F1',
            # 'reload_env'    : 'F2',
            'undo'          : '<Control>Z',     # TODO History is still missing
            'redo'          : '<Control>Y',     # TODO History is still missing
            # 'center_view'   : '<Alt>space',
        }

    def on_main_window_destroy(self, widget, data=None):
        self.view.logging_view.quit_flag = True
        logger.debug("Main window destroyed")
        log.debug_filter.set_logging_test_view(None)
        log.error_filter.set_logging_test_view(None)
        gtk.main_quit()

    def register_view(self, view):
        self.shortcut_manager = ShortcutManager(self.view['main_window'])
        self.register_actions(self.shortcut_manager)

        view['main_window'].connect('destroy', gtk.main_quit)

    @ExtendedController.observe("execution_engine", after=True)
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

    def delegate_action(self, event, method, *arg):
        """
        Is a test function to be used in future to insert key-accelerator
        into the currently active widget (e.g. Delete to tree/list widget of DataFlows).
        """

        idx = self.child_controllers['states_editor_ctrl'].view.notebook.get_current_page()
        page = self.child_controllers['states_editor_ctrl'].view.notebook.get_nth_page(idx)
        ctrl = None
        for identifier, pdict in self.child_controllers['states_editor_ctrl'].tabs.iteritems():
            if page is pdict['page']:
                ctrl = pdict['ctrl']
        method(ctrl.data_flows_ctrl.df_list_ctrl, event, *arg)

    def on_delete_state_activate(self, widget, data=None):
        logger.debug("Delete selected state now ...")
        selection = self.child_controllers['state_machines_editor_ctrl'].model.state_machines.values()[0].selection
        selected_state_model = selection.get_selected_state()

        if selected_state_model and selected_state_model.parent is not None:
            selected_state_model.parent.state.remove_state(selected_state_model.state.state_id)
            selection.remove(selected_state_model)

    def on_add_state_activate(self, widget, method=None, *arg):

        selection = self.child_controllers['state_machines_editor_ctrl'].model.state_machines.values()[0].selection
        selected_state_model = selection.get_selected_state()
        logger.debug("Add state in selected state %s now ..." % selected_state_model.state.name)
        # logger.info("Add state %s now ..." % selected_state_model)

        if selected_state_model and isinstance(selected_state_model, ContainerStateModel):
            state = ExecutionState("~")
            state_id = selected_state_model.state.add_state(state)
            if state_id:
                state_model = selected_state_model.states[state.state_id]
                #logger.info("create exec_State: %s" % state)
                selection.set([state_model])
        else:
            logger.warning("Add state FAILED: State has to be inheritor of type ContainerState!!!")

    def on_delete_activate(self, widget, data=None):
        logger.debug("Delete something that is selected now ...")
        #logger.debug("focus is here: %s" % self.view['main_window'].get_focus())

        selection = self.child_controllers['state_machines_editor_ctrl'].model.state_machines.values()[0].selection
        if selection.get_selected_state() and selection.get_selected_state().parent is not None:
            selected_state_model = selection.get_selected_state()
            selected_state_model.parent.state.remove_state(selected_state_model.state.state_id)
            selection.remove(selected_state_model)
            logger.debug("Delete State: %s, %s" % (selected_state_model.state.state_id,
                                                   selected_state_model.state.name))
        elif len(selection) == 1 and selection.get_num_data_flows() == 1:
            data_flow = selection.get_data_flows()[0].data_flow
            selection.get_data_flows()[0].parent.state.remove_data_flow(data_flow.data_flow_id)
            selection.remove(selection.get_data_flows()[0])
            logger.debug("Delete DataFlow: from %s to %s" % (data_flow.from_state,
                                                             data_flow.to_state))
        elif len(selection) == 1 and selection.get_num_transitions() == 1:
            transition = selection.get_transitions()[0].transition
            selection.get_transitions()[0].parent.state.remove_transition(transition.transition_id)
            selection.remove(selection.get_transitions()[0])
            logger.debug("Delete Transition: from %s, %s to %s, %s" % (transition.from_state, transition.from_outcome,
                                                                       transition.to_state, transition.to_outcome))
        else:
            logger.debug("in selection is nothing deletable: %s" % selection)

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
        statemachine.singleton.global_storage.save_statemachine_as_yaml(
            self.model.get_active_state_machine_model().root_state.state,
            statemachine.singleton.global_storage.base_path,
            delete_old_state_machine=False)

        self.model.get_active_state_machine_model().root_state.store_meta_data_for_state()

    def on_open_activate(self, widget, data=None):
        pass

    def on_new_activate(self, widget, data=None):
        pass
