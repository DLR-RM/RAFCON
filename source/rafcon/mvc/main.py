from twisted.internet import gtk2reactor
gtk2reactor.install()
from twisted.internet import reactor

import sys
import logging
import signal
import gtk

from rafcon.utils import log
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
import rafcon.statemachine.singleton
import rafcon.mvc.singleton

from rafcon.statemachine.state_machine import StateMachine


def check_requirements():
    """Checks versions and other requirements"""
    import gtkmvc
    gtkmvc.require("1.99.1")
    return


def create_models(*args, **kargs):
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    # logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = ExecutionState('Nested')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = HierarchyState(name="Container")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"

    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    # this is not allowed as the output port is already connected
    # ctr_state.add_data_flow(ctr_state.state_id, scoped_variable2_ctr_state, ctr_state.state_id, output_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    global_var_manager_model = rafcon.mvc.singleton.global_variable_manager_model
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return state5, logger, ctr_state, global_var_manager_model


if __name__ == '__main__':
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    rafcon.statemachine.singleton.library_manager.initialize()
    check_requirements()

    [execution_state, logger, ctr_state, gvm_model] = create_models()

    state_machine = StateMachine(ctr_state)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')

    # ctr_model = ContainerStateModel(ctr_state)
    # sdev = StateDataportEditorView()
    # StateDataPortEditorController(ctr_model, sdev)

    # a view, whose buttons can trigger arbitrary function that are needed for testing purposes
    # test_buttons_view = TestButtonsView(ctr_model)

    # state_machine_tree = SingleWidgetWindowView(StateMachineTreeView, width=500, height=200, title='State Machine
    # Tree')
    # state_machine_model = SingleWidgetWindowController(ctr_model, state_machine_tree, StateMachineTreeController)

    # library_tree = SingleWidgetWindowView(LibraryTreeView, width=300, height=200, title='Library Tree')
    # library_controller = SingleWidgetWindowController(None, library_tree, LibraryTreeController)

    # src_view = SingleWidgetWindowView(SourceEditorView, width=550, height=500, title='Source Editor')
    # src_ctrl = SingleWidgetWindowController(ctr_model, src_view, SourceEditorController)

    # prop_view = SingleWidgetWindowView(StateOverviewView, width=400, height=100, title='Properties Editor')
    # prop_ctrl = SingleWidgetWindowController(this_model, prop_view, StateOverviewController)

    # dp_editor_view = StateDataportEditorView()
    # dp_editor_ctrl = StateDataPortEditorController(ContainerStateModel(ctr_state), dp_editor_view)

    # oc_editor_view = SingleWidgetWindowView(StateOutcomesEditorView, width=500, height=200, title='Outcomes Editor')
    # oc_editor_ctrl = SingleWidgetWindowController(ctr_model, oc_editor_view, StateOutcomesEditorController)
    # #oc_editor_ctrl = SingleWidgetWindowController(this_model, oc_editor_view, StateOutcomesEditorController)

    # state_editor_view = SingleWidgetWindowView(StateEditorView, width=550, height=500, title='State Editor View')
    # state_editor_ctrl = SingleWidgetWindowController(ctr_model, state_editor_view, StateEditorController)
    # #state_editor_ctrl = SingleWidgetWindowController(this_model, state_editor_view, StateEditorController)

    # state_editor_view = SingleWidgetWindowView(StateEditorEggView, width=550, height=500, title='State Editor Egg View')
    # state_editor_ctrl = SingleWidgetWindowController(this_model, state_editor_view, StateEditorEggController)

    # state_editor_view = SingleWidgetWindowView(StateEditorLDView, width=550, height=500, title='State Editor LD View')
    # state_editor_ctrl = SingleWidgetWindowController(ctr_model, state_editor_view, StateEditorController)

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

    reactor.run()
    gtk.main()
    logger.debug("after gtk main")
