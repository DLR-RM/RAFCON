
import sys
import gtk
import logging
from utils import log
from mvc.models import StateModel, ContainerStateModel, GlobalVariableManagerModel, ExternalModuleManagerModel
from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController,\
    StateDataPortEditorController, GlobalVariableManagerController, ExternalModuleManagerController,\
    SourceEditorController, SingleWidgetWindowController,StateEditorController, StateMachineTreeController,\
    LibraryTreeController, MainWindowController
from mvc.views import StatePropertiesView, ContainerStateView, GraphicalEditorView, StateDataportEditorView,\
    GlobalVariableEditorView, ExternalModuleManagerWindowView, ExternalModuleManagerView,  SourceEditorView, \
    SingleWidgetWindowView, StateEditorView, LoggingView, StateMachineTreeView, LibraryTreeView, MainWindowView
from mvc.views.single_widget_window import TestButtonsView
from mvc.views.state_transitions import StateTransitionsEditorView
from mvc.controllers.state_transitions import StateTransitionsEditorController
from mvc.views.state_data_flows import StateDataFlowsEditorView
from mvc.controllers.state_data_flows import StateDataFlowsEditorController
from mvc.views.connections_editor import StateConnectionsEditorView
from mvc.controllers.connections_editor import StateConnectionsEditorController
from mvc.views.state_outcomes import StateOutcomesEditorView
from mvc.controllers.state_outcomes import StateOutcomesEditorController
from mvc.views.state_overview import StateOverviewView
from mvc.controllers.state_overview import StateOverviewController
from mvc.views.state_editor import StateEditorEggView, StateEditorLDView
from mvc.controllers.state_editor import StateEditorEggController, StateEditorLDController

from mvc.models.state_machine_manager import StateMachineManagerModel
from statemachine.state_machine_manager import StateMachineManager
from statemachine.states.state import State, DataPort
from statemachine.states.execution_state import ExecutionState
from statemachine.states.container_state import ContainerState
from statemachine.transition import Transition
from statemachine.data_flow import DataFlow
from statemachine.external_modules.external_module import ExternalModule
import statemachine.singleton
from statemachine.states.hierarchy_state import HierarchyState


def setup_path():
    """Sets up the python include paths to include needed directories"""
    import os.path
    import sys

    #sys.path.insert(1, '.')
    #sys.path.insert(0, reduce(os.path.join, (TOPDIR, "resources", "external")))
    #sys.path.insert(0, os.path.join(TOPDIR, "src"))
    return


def check_requirements():
    """Checks versions and other requirements"""
    import gtkmvc
    gtkmvc.require("1.99.1")
    return


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def create_models(*args, **kargs):
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    state1 = State('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "int", 0)
    state1.add_outcome('success', 0)
    state2 = State('State2')
    input_my_input_state2 = state2.add_input_data_port("my_input", "int", 0)
    input_long_state2 = state2.add_input_data_port("longlonginputname", "int", 0)
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_my_output_state2 = state2.add_output_data_port("my_output", "int")
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = State('Nested')
    output_state4 = state4.add_output_data_port("out", "int")
    state4.add_outcome("success", 0)
    state5 = State('Nested2')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = ContainerState(name='State3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = ContainerState(name="Container")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "int", 0)
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, -2, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, -2, None, -2)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"

    ctr_state.add_input_data_port("input_data1", "str", "default_value1")
    ctr_state.add_input_data_port("input_data2", "str", "default_value2")
    ctr_state.add_input_data_port("input_data3", "str", "default_value3")

    ctr_state.add_output_data_port("output_data1", "str", "default_value1")
    ctr_state.add_output_data_port("output_data2", "str", "default_value2")
    ctr_state.add_output_data_port("output_data3", "str", "default_value3")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped_variable1", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("scoped_variable2", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("scoped_variable3", "str", "default_value1")

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    # this is not allowed as the output port is already connected
    #ctr_state.add_data_flow(ctr_state.state_id, scoped_variable2_ctr_state, ctr_state.state_id, output_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    ctr_model = ContainerStateModel(ctr_state)

    external_module_manager_model = ExternalModuleManagerModel()
    sys.path.insert(0, '../../test_scripts')
    em = ExternalModule(name="External Module 1", module_name="external_module_test", class_name="TestModule")
    external_module_manager_model.external_module_manager.add_external_module(em)
    external_module_manager_model.external_module_manager.external_modules["External Module 1"].connect([])
    external_module_manager_model.external_module_manager.external_modules["External Module 1"].start()
    em = ExternalModule(name="External Module 2", module_name="external_module_test2", class_name="TestModule2")
    external_module_manager_model.external_module_manager.add_external_module(em)

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return ctr_model, logger, ctr_state, global_var_manager_model, external_module_manager_model

import time
def id_generator2():
    time.sleep(0.001)
    return int(time.time()*1000)


if __name__ == '__main__':
    statemachine.singleton.library_manager.initialize()
    setup_path()
    check_requirements()
    #logging_view = SingleWidgetWindowView(LoggingView, width=500, height=200, title='Logging')
    #setup_logger(logging_view['main_frame'])
    logging_view = LoggingView()
    setup_logger(logging_view)
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = create_models()
    this_model = filter(lambda model: model.state.name == 'State3', ctr_model.states.values()).pop()

    statemachine.singleton.state_machine_manager.root_state = ctr_state
    sm_manager_model = StateMachineManagerModel(statemachine.singleton.state_machine_manager)
    main_window_view = MainWindowView(logging_view)
    main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model,
                                                  editor_type='ld')

    # sdev = StateDataportEditorView()
    # StateDataPortEditorController(ctr_model, sdev)

    # a view, whose buttons can trigger arbitrary function that are needed for testing purposes
    #test_buttons_view = TestButtonsView(ctr_model)

    #state_machine_tree = SingleWidgetWindowView(StateMachineTreeView, width=500, height=200, title='State Machine Tree')
    #state_machine_model = SingleWidgetWindowController(ctr_model, state_machine_tree, StateMachineTreeController)

    #library_tree = SingleWidgetWindowView(LibraryTreeView, width=300, height=200, title='Library Tree')
    #library_controller = SingleWidgetWindowController(None, library_tree, LibraryTreeController)

    #src_view = SingleWidgetWindowView(SourceEditorView, width=550, height=500, title='Source Editor')
    #src_ctrl = SingleWidgetWindowController(ctr_model, src_view, SourceEditorController)

    # external_module_manager_view = SingleWidgetWindowView(ExternalModuleManagerView, width=500, height=200, title='External Module Manager')
    # external_module_manger_controller = SingleWidgetWindowController(emm_model, external_module_manager_view, ExternalModuleManagerController)
    # external_module_manger_controller.set_source_view(src_view.widget_view)

    # prop_view = SingleWidgetWindowView(StateOverviewView, width=400, height=100, title='Properties Editor')
    # prop_ctrl = SingleWidgetWindowController(this_model, prop_view, StateOverviewController)

    # dp_editor_view = StateDataportEditorView()
    # dp_editor_ctrl = StateDataPortEditorController(this_model, dp_editor_view)

    # oc_editor_view = SingleWidgetWindowView(StateOutcomesEditorView, width=500, height=200, title='Outcomes Editor')
    # oc_editor_ctrl = SingleWidgetWindowController(ctr_model, oc_editor_view, StateOutcomesEditorController)
    # #oc_editor_ctrl = SingleWidgetWindowController(this_model, oc_editor_view, StateOutcomesEditorController)

    #state_editor_view = SingleWidgetWindowView(StateEditorView, width=550, height=500, title='Source Editor')
    #state_editor_ctrl = SingleWidgetWindowController(ctr_model, state_editor_view, StateEditorController)
    # #state_editor_ctrl = SingleWidgetWindowController(this_model, state_editor_view, StateEditorController)

    # state_editor_view = SingleWidgetWindowView(StateEditorEggView, width=550, height=500, title='Source Editor')
    # state_editor_ctrl = SingleWidgetWindowController(this_model, state_editor_view, StateEditorEggController)

    # state_editor_view = SingleWidgetWindowView(StateEditorLDView, width=550, height=500, title='Source Editor')
    # state_editor_ctrl = SingleWidgetWindowController(ctr_model, state_editor_view, StateEditorLDController)

    # trans_editor_view = SingleWidgetWindowView(StateTransitionsEditorView, width=550, height=400, title='Transitions Editor')
    # trans_editor_ctrl = SingleWidgetWindowController(this_model, trans_editor_view, StateTransitionsEditorController)

    # df_editor_view = SingleWidgetWindowView(StateDataFlowsEditorView, width=550, height=400, title='Data Flows Editor')
    # df_editor_ctrl = SingleWidgetWindowController(this_model, df_editor_view, StateDataFlowsEditorController)

    # scon_editor_view = SingleWidgetWindowView(StateConnectionsEditorView, width=550, height=400, title='Connections Editor')
    # scon_editor_ctrl = SingleWidgetWindowController(this_model, scon_editor_view, StateConnectionsEditorController)

    # global_variables_view = SingleWidgetWindowView(GlobalVariableEditorView, width=500, height=200, title='Global Variable Manager')
    # global_variables_controller = SingleWidgetWindowController(gvm_model, global_variables_view, GlobalVariableManagerController)

    # graphical_editor_view = SingleWidgetWindowView(GraphicalEditorView, title="Graphical Editor", pos=1)
    # graphical_editor_ctrl = SingleWidgetWindowController(ctr_model, graphical_editor_view, GraphicalEditorController)

    gtk.main()
    logger.debug("after gtk main")
