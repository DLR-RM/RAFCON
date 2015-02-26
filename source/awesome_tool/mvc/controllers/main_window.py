import traceback

import gtk

from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController,\
    StateDataPortEditorController, GlobalVariableManagerController, ExternalModuleManagerController,\
    SourceEditorController, SingleWidgetWindowController, StateEditorController, StateMachineTreeController,\
    LibraryTreeController
import statemachine.singleton
from mvc.controllers.extended_controller import ExtendedController
from mvc.controllers.states_editor import StatesEditorController
from mvc.controllers.state_machines_editor import StateMachinesEditorController
from statemachine.states.execution_state import ExecutionState
from mvc.models.container_state import ContainerStateModel
from mvc.models.state_machine import StateMachineModel
from mvc.models.state_machine_manager import StateMachineManagerModel

from utils import log
logger = log.get_logger(__name__)

from functools import partial

def setup_key_binding(key_map, view, widget):
        """
        Read the Key_map and add key accelerators according to the
        name of field key_map, adds AccelGroup to widget and add accelerator to dict of view.

        :param key_map dict that holds as key the name of menu-button and as value the accelerator-key
        :param view View in which the widget-dict is used to update MenuItems and to insert new (not visualized ones)
        :param widget gtk.Window that could add accelerator-groups and have a menu
        """

        accelgroup = gtk.AccelGroup()
        widget.add_accel_group(accelgroup)

        for item in key_map.iteritems():
            key, mod = gtk.accelerator_parse(str(item[1]))
            try:
                if not view[item[0]]:
                    view[item[0]] = gtk.MenuItem()
                    logger.info("NOT_IN key-accelerator %s insert a not visualized MenuItem with connected accelerator-key" % str(item))
                view[item[0]].add_accelerator("activate", accelgroup, key, mod, gtk.ACCEL_VISIBLE)
            except:
                logger.warn(traceback.format_exc())
                pass


class ShortcutManager():
    """Handles shortcuts

    Holds a mapping between shortcuts and action. Actions can be subscribed to. When a listed shortcut is triggered,
    all subscribers are notified.
    """
    __action_to_shortcuts = dict()


    def __init__(self, window):
        # Setup window to listen for accelerators
        self.main_window = window
        self.accel_group = gtk.AccelGroup()
        self.main_window.add_accel_group(self.accel_group)

        self.__init_shortcuts()
        self.__register_shortcuts()
        self.__action_to_callbacks = dict()


    def __init_shortcuts(self):
        self.__action_to_shortcuts = {
            'copy': '<Control>C',
            'paste': '<Control>V',
            'cut': '<Control>X',
            'add': '<Control>A',
            'delete': ['<Control>D', 'Delete'],
            'group': '<Control>G',
            'ungroup': '<Control>U',
            'entry': '<Control>E',
            'fit': '<Control>space',
            'info': '<Control>q',
            'start': 'F5',
            'step_mode': 'F6',
            'pause': 'F7',
            'stop': 'F8',
            'step': 'F4',
            'backward_step_mode': 'F9',
            'reload': 'F5',
            'undo': '<Control>Z',
            'redo': ['<Control>Y', '<Control><Shift>Z']
        }

    def __register_shortcuts(self):
        for action in self.__action_to_shortcuts:
            # Make sure, all shortcuts are in a list
            shortcuts = self.__action_to_shortcuts[action]
            if not isinstance(shortcuts, list):
                shortcuts = [shortcuts]
                self.__action_to_shortcuts[action] = shortcuts
            # Now register the shortcuts in the window to trigger the shortcut signal
            for shortcut in shortcuts:
                keyval, modifier_mask = gtk.accelerator_parse(shortcut)
                if keyval == 0 and modifier_mask == 0:  # No valid shortcut
                    continue
                callback = partial(self.__on_shortcut, action)  # Bind the action to the callback function
                self.accel_group.connect_group(keyval, modifier_mask, gtk.ACCEL_VISIBLE, callback)

    def __on_shortcut(self, action, accel_group, window, keyval, modifier_mask):
        print "shortcut triggered", action, accel_group, keyval, modifier_mask
        self.trigger_action(action, keyval, modifier_mask)

    def _get_action_for_shortcut(self, lookup_shortcut):
        for action in self.__action_to_shortcuts:
            shortcuts = self.__action_to_shortcuts[action]
            for shortcut in shortcuts:
                if shortcut == lookup_shortcut:
                    return action
        return None

    def add_callback(self, action, callback):
        """Adds a callback function to an action

        The method checks whether both action and callback are valid. If so, the callback is added to the list of
        functions called when the action is triggered.
        :param action: An action like 'add', 'copy', 'info'
        :param callback: A callback function, which is called when action is triggered. It retrieves the event as
        parameter
        :return: True is the parameters are valid and the callback is registered, False else
        """
        if action in self.__action_to_shortcuts:  # Is the action valid?
            if hasattr(callback, '__call__'):  # Is the callback really a function?
                if action not in self.__action_to_callbacks:
                    self.__action_to_callbacks[action] = []
                assert isinstance(self.__action_to_callbacks[action], list)
                self.__action_to_callbacks[action].append(callback)
                return True
        return False

    def get_shortcut_for_action(self, action):
        """Get the shortcut(s) for the specified action
        :param action: An action like 'add', 'copy', 'info'
        :return: None, if no action is not valid or no shortcut is exiting, a single shortcut or a list of shortcuts
        if one or more shortcuts are registered for that action.
        """
        if action in self.__action_to_shortcuts:
            return self.__action_to_shortcuts[action]
        return None

    def trigger_action(self, action, keyval, modifier_mask):
        """Calls the appropriate callback function for the given action

        :param action:
        :param event:
        :return:
        """

        if action in self.__action_to_callbacks:
            for callback_function in self.__action_to_callbacks[action]:
                try:
                    callback_function(keyval, modifier_mask)
                except Exception as e:
                    logger.error('Exception while calling callback methods for action "{0}": {1}'.format(
                        action, e.message))


class MainWindowController(ExtendedController):

    def __init__(self, state_machine_manager_model, view, emm_model, gvm_model, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        assert isinstance(state_machine_manager_model, StateMachineManagerModel)

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
        right_v_pane = view['right_v_pane']

        view.get_top_widget().connect("destroy", self.on_main_window_destroy)

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
        tree_notebook = view["tree_notebook"]
        #remove placeholder tab
        library_tree_tab = view['library_tree_placeholder']
        page_num = tree_notebook.page_num(library_tree_tab)
        tree_notebook.remove_page(page_num)
        #append new tab
        library_controller = LibraryTreeController(active_state_machine.root_state, view.library_tree)
        self.child_controllers['library_controller'] = library_controller
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
        state_machine_tree_controller = StateMachineTreeController(active_state_machine, view.state_machine_tree)
        self.child_controllers['state_machine_tree_controller'] = state_machine_tree_controller
        state_machine_label = gtk.Label('Statemachine Tree')
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
        self.child_controllers['states_editor_ctrl'] = states_editor_ctrl
        ######################################################
        # state machines editor
        ######################################################

        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                        view.state_machines_editor,
                                                                        state_machine_tree_controller,
                                                                        states_editor_ctrl)
        self.child_controllers['state_machines_editor_ctrl'] = state_machines_editor_ctrl
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
        external_modules_controller = ExternalModuleManagerController(emm_model, view.external_module_manager_view)
        self.child_controllers['external_modules_controller'] = external_modules_controller
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
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_manager_view)
        self.child_controllers['global_variable_manager_ctrl'] = global_variable_manager_ctrl
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
        statemachine.singleton.external_module_manager.stop_all_modules()
        gtk.main_quit()

    def register_view(self, view):
        setup_key_binding(self.key_map, view, view.get_top_widget())

        self.shortcut_manager = ShortcutManager(self.view['main_window'])
        self.register_actions(self.shortcut_manager)

        #view['add'].connect('activate', self.delegate_action, StateDataFlowsListController.on_add)
        #view['add_state'].connect('activate', self.delegate_action, StateDataFlowsListController.on_add)
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
            selected_state_model.state.add_state(state)
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
        pass

    def on_open_activate(self, widget, data=None):
        pass

    def on_new_activate(self, widget, data=None):
        pass
