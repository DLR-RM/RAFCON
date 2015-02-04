import gtk
from gtkmvc import Controller
from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController,\
    StateDataPortEditorController, GlobalVariableManagerController, ExternalModuleManagerController,\
    SourceEditorController, SingleWidgetWindowController,StateEditorController, StateMachineTreeController,\
    LibraryTreeController
import statemachine.singleton


class MainWindowController(Controller):

    state_machine_execution_engine = None

    __observables__ = ("state_machine_execution_engine",)

    def __init__(self, root_state_model, view, em_module, gvm_model):
        Controller.__init__(self, root_state_model, view)
        self.state_machine_execution_engine = statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        self.root_state_model = root_state_model
        self.em_module = root_state_model
        self.em_module = root_state_model

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
        #print tree_notebook.get_tab_label(self.library_tree_tab).get_text()
        #print tree_notebook.page_num(self.library_tree_tab)
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
        self.state_machine_tree_controller = StateMachineTreeController(root_state_model, view.state_machine_tree)
        state_machine_label = gtk.Label('Statemachine Tree')
        tree_notebook.insert_page(view.state_machine_tree, state_machine_label, page_num)

        ######################################################
        # graphical editor
        ######################################################
        graphical_editor_frame = view['graphical_editor_frame']
        self.graphical_editor = GraphicalEditorController(root_state_model, view.graphical_editor_view)
        graphical_editor_frame.add(view.graphical_editor_view['main_frame'])
        #self.graphical_editor = GraphicalEditorController(model, view.graphical_editor_window.get_top_widget())
        #test = SingleWidgetWindowController(model, view.graphical_editor_window, GraphicalEditorController)

        ######################################################
        # state editor
        ######################################################
        this_model = filter(lambda model: model.state.name == 'State3', root_state_model.states.values()).pop()
        self.state_editor = StateEditorController(root_state_model, view.state_editor)  # .get_top_widget())

        ######################################################
        # external module editor
        ######################################################
        em_global_notebook = view["em_global_notebook"]
        #remove placeholder tab
        external_modules_tab = view['external_modules_placeholder']
        page_num = em_global_notebook.page_num(external_modules_tab)
        em_global_notebook.remove_page(page_num)
        #append new tab
        self.external_modules_controller = ExternalModuleManagerController(em_module, view.external_module_manager_view)
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

    @Controller.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        status_bar3 = self.view["statusbar3"]
        status_bar3_string = "Execution status: " + \
                            str(statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        status_bar3.push(0, status_bar3_string)

    def register_view(self, view):
        view['main_window'].connect('destroy', gtk.main_quit)

    def on_about_activate(self, widget, data=None):
        pass

    def on_backward_step_mode_activate(self, widget, data=None):
        pass

    def on_step_activate(self, widget, data=None):
        pass

    def on_step_mode_activate(self, widget, data=None):
        pass

    def on_stop_activate(self, widget, data=None):
        pass

    def on_pause_activate(self, widget, data=None):
        pass

    def on_start_activate(self, widget, data=None):
        pass

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
        pass

    def on_add_state_activate(self, widget, data=None):
        pass

    def on_delete_activate(self, widget, data=None):
        pass

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